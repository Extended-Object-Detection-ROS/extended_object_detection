#include "ObjectBase.h"

using namespace std;
using namespace cv;

namespace eod{

    ObjectBase::ObjectBase(){
        loaded = false;
    }
    
    string ObjectBase::getPathAttribute(TiXmlElement * attr, const char * at_name){
        const char* path = attr->Attribute(at_name);
        if( path == NULL or strlen(path) == 0)
            return "";
        if(path[0] != '/')
            return object_base_path + "/" + string(path);
        return string(path);            
    }
    
    bool ObjectBase::loadFromXML(string filename){
        
        size_t found = filename.find_last_of("/");
        object_base_path = filename.substr(0,found);
        
        TiXmlDocument *doc = new TiXmlDocument(filename.c_str());
        loaded = doc->LoadFile();

        if (!loaded)
        {
            printf("Could not load  %s file. Error='%s'.\n", filename.c_str(), doc->ErrorDesc());
            return loaded;
        }

        loaded = loadFromXMLa(doc, object_base_path);
        if( loaded ) printf("Attributes have been readed sucsessfuly.\n");
        else printf("Error reading attributes.\n");

        loaded &= loadFromXMLso(doc);
        if( loaded ) printf("Simple objects have been readed sucsessfuly.\n");
        else printf("Error reading simple_objects.\n");

        loaded &= loadFromXMLr(doc);
        if( loaded ) printf("Relationship list has been readed sucsessfuly.\n");
        else printf("Error reading relations.\n");
        
        loaded &= loadFromXMLsNM(doc);
        if( loaded ) printf("Complex objects have been loaded successfully!\n");
        else printf("Error reading NM-scenes.\n");

        doc->~TiXmlDocument();

        checkIDobj();
        //checkIDsc();

        return loaded;
    }
    
    // ------------------
    // attributes
    // ------------------
    bool ObjectBase::loadFromXMLa(TiXmlDocument *doc, string object_base_path)
    {

        TiXmlElement *base = doc->FirstChildElement("AttributeLib");
        if (!base) return false;

        TiXmlElement *attr = base->FirstChildElement("Attribute");
        
        while (attr){
            Attribute* tmpA;
            
            string type_name = attr->Attribute("Type");            
            AttributeTypes type = getAttributeTypeFromName(type_name);
            
            if (type == UNK_A){
                attr = attr->NextSiblingElement("Attribute");
                continue;
            }
            
            string name = attr->Attribute("Name");
            if( name == "" ){
                printf("Error! Some attributes has no name!");
                attr = attr->NextSiblingElement("Attribute");
                continue;
            }
            
            double weight = 1;
            attr->Attribute("Weight",&weight);                 
            
            switch (type) {
            case HSV_COLOR_A:
            {
                int hm = 0, hM = 0, sm = 0, sM = 0, vm = 0, vM = 0;
                attr->Attribute("Hmin", &hm);
                attr->Attribute("Hmax", &hM);
                attr->Attribute("Smin", &sm);
                attr->Attribute("Smax", &sM);
                attr->Attribute("Vmax", &vM);
                attr->Attribute("Vmin", &vm);                             
                tmpA = new HsvColorAttribute(hm, hM, sm, sM, vm, vM);                
                break;
            }                
            case HAAR_CASCADE_A:
            {
                string cascade_path = attr->Attribute("Cascade");
                if(cascade_path.c_str()[0] != '/')
                    cascade_path = object_base_path + "/" + cascade_path;
                tmpA = new HaarCascadeAttribute(cascade_path );
                break;
            }   
            case SIZE_A:
            {
                double  mPc = 0 , MPc = 100; 
                attr->Attribute("MaxAreaPc", &MPc);
                attr->Attribute("MinAreaPc", &mPc);                
                tmpA = new SizeAttribute(mPc, MPc);                
                break;
            }   
            case HIST_COLOR_A:
            {
                string hist_path = attr->Attribute("Histogram");
                if( hist_path.c_str()[0] != '/')
                    hist_path = object_base_path + "/" + hist_path;
                tmpA = new HistColorAttribute(hist_path);
                break;
            }   
            case DIMEN_A:
            {
                double minR = 1, maxR = 1;
                int rotated;
                attr->Attribute("minRatio", &minR);
                attr->Attribute("maxRatio", &maxR);                
                minR = minR > 0 ? minR : 1;
                maxR = maxR > 0 ? maxR : 1;
                tmpA = new DimentionAttribute(minR, maxR);                
                break;
            }   
            case HOUGH_A:
            {
                int HoughType = 0;
                attr->Attribute("HoughType", &HoughType);
                if( HoughType != 0 and HoughType != 1)
                    HoughType = 0;
                tmpA = new HoughAttribute(HoughType);
                
                double dp = 1, md = 10, p1 = 200, p2 = 100;
                int mr = 0, Mr = 0;
                attr->Attribute("dp", &dp);
                attr->Attribute("md", &md);
                attr->Attribute("p1", &p1);
                attr->Attribute("p2", &p2);
                
                attr->Attribute("mr", &mr);
                attr->Attribute("Mr", &Mr);
                
                ((HoughAttribute*)tmpA)->SetParamsCircle(dp, md, p1, p2, mr, Mr);
                
                break;
            }
            case BASIC_MOTION_A:
                tmpA = new BasicMotionAttribute();
                break;                            
            case ARUCO_A:
            {
                int IDmarker = -1;
                attr->Attribute("IDmarker", &IDmarker);
                int dictionary = 0;
                attr->Attribute("Dict", &dictionary);
                double markerLen = 0;
                attr->Attribute("Lenght", &markerLen);                                
                tmpA = new ArucoAttribute(dictionary, IDmarker, markerLen);
                break;            
            }   
            case POSE_A:
            {
                double x_min = 0, x_max = 1, y_min = 0, y_max = 1;
                attr->Attribute("x_min", &x_min);
                attr->Attribute("x_max", &x_max);
                attr->Attribute("y_min", &y_min);
                attr->Attribute("y_max", &y_max);
                tmpA = new PoseAttribute(x_min, x_max, y_min, y_max);
                break;            
            }
#ifdef USE_TF
            case TF_A:
            {
                int obj_id = -1;
                attr->Attribute("obj_id", &obj_id);
                
                if( obj_id == -1){
                    printf("Provide obj_id parameter to TensorFLow attribute!\n");
                    attr = attr->NextSiblingElement("Attribute");
                    continue;
                }
                
                string graph_path = attr->Attribute("graph");
                if(graph_path.c_str()[0] != '/')
                    graph_path = object_base_path + "/" + graph_path;
                
                string labels_path = attr->Attribute("labels");
                if(labels_path.c_str()[0] != '/')
                    labels_path = object_base_path + "/" + labels_path;
                
                tmpA = new TensorFlowAttribute(graph_path, labels_path, obj_id);
                break;
            }
#endif            
#ifdef USE_OPENCV_CONTRIB
            case FEATURE_A:
            {
                FEATURE_DETECTOR featureExtractorType;
                int min_matches = 50;
                double height = 0;
                string featureExtractor = attr->Attribute("featureExtractorType");
                if( featureExtractor == "SIFT"){
                    featureExtractorType = FD_SIFT;
                }
                else if( featureExtractor == "SURF"){
                    featureExtractorType = FD_SURF;
                }
                else if( featureExtractor == "ORB"){
                    featureExtractorType = FD_ORB;
                }
                else{
                    printf("Unknown or unspecified feature extractor type, used SIFT instead as default!\n");
                    featureExtractorType = FD_SIFT;
                }                
                
                attr->Attribute("min_matches", &min_matches);
                attr->Attribute("height", &height);                
                string image_path = attr->Attribute("sample_image");
                if(image_path.c_str()[0] != '/')
                    image_path = object_base_path + "/" + image_path;
                
                tmpA = new FeatureAttribute(featureExtractorType, image_path, min_matches, height);
                break;
            }            
#endif       
#if (CV_MAJOR_VERSION > 3)
            case DNN_A:
            {
                string framework = attr->Attribute("framework");
                
                string weights = attr->Attribute("weights");
                if(weights.c_str()[0] != '/')
                    weights = object_base_path + "/" + weights;
                string config = attr->Attribute("config");
                if(config.c_str()[0] != '/')
                    config = object_base_path + "/" + config;
                const char* labels = attr->Attribute("labels");
                string labels_str;
                if( !labels )
                    labels = "";
                else{
                    
                    if( labels[0] != '/')
                        labels_str = object_base_path + "/" + labels;
                }                
                                
                int inpWidth = 300;
                int inpHeight = 300;
                
                attr->Attribute("inputWidth", &inpWidth);                
                attr->Attribute("inputHeight", &inpHeight);    
                
                int obj_id = -1;
                attr->Attribute("obj_id", &obj_id);    
                
                int forceCuda = 0;
                attr->Attribute("forceCuda", &forceCuda);                
                
                tmpA = new DnnAttribute(obj_id, framework, weights, config, inpWidth, inpHeight, labels_str, (forceCuda != 0));
                break;
            }
            case QR_A:
            {
                tmpA = new QrAttribute();
                break;
            }
#endif
#ifdef USE_ZBAR
            case QR_ZBAR_A:
            {
                double real_len = -1;                
                const char* info = attr->Attribute("Info");
                if(!info)
                    info = "";
                attr->Attribute("Lenght", &real_len);                 
                tmpA = new QrZbarAttribute(real_len, string(info));
                break;
            }
#endif
            case LOG_AND_A:
            {
                string attributeA = attr->Attribute("A");
                string attributeB = attr->Attribute("B");
                double iou = 0.75;
                attr->Attribute("iou",&iou);
                
                Attribute* aA = getByNameA(attributeA);                
                if( aA == NULL ){
                    printf("Can't find attribute %s in base needed for LogicAnd attribute!\n", attributeA.c_str());
                    attr = attr->NextSiblingElement("Attribute");
                    continue;
                }
                Attribute* aB = getByNameA(attributeB);
                if( aB == NULL ){
                    printf("Can't find attribute %s in base needed for LogicAnd attribute!\n", attributeB.c_str());
                    attr = attr->NextSiblingElement("Attribute");
                    continue;
                }
                tmpA = new AndAttribute(aA, aB, iou);                
                break;
            }
            case LOG_NOT_A:
            {
                string attributeA = attr->Attribute("A");                                
                Attribute* aA = getByNameA(attributeA);                
                
                if( aA == NULL ){
                    printf("Can't find attribute %s in base needed for LogicNot attribute!\n", attributeA.c_str());
                    attr = attr->NextSiblingElement("Attribute");
                    continue;
                }
                
                tmpA = new NotAttribute(aA);                
                break;
            }
            case LOG_OR_A:
            {
                string attributeA = attr->Attribute("A");
                string attributeB = attr->Attribute("B");
                double iou = 0.75;
                attr->Attribute("iou",&iou);
                
                Attribute* aA = getByNameA(attributeA);                
                if( aA == NULL ){
                    printf("Can't find attribute %s in base needed for LogicOr attribute!\n", attributeA.c_str());
                    attr = attr->NextSiblingElement("Attribute");
                    continue;
                }
                Attribute* aB = getByNameA(attributeB);
                if( aB == NULL ){
                    printf("Can't find attribute %s in base needed for LogicOr attribute!\n", attributeB.c_str());
                    attr = attr->NextSiblingElement("Attribute");
                    continue;
                }
                tmpA = new OrAttribute(aA, aB, iou);                
                break;
            }
            case LOG_XOR_A:
            {
                break;
            }
            case BLOB_A:
            {
                int minThreshold =10, maxThreshold = 200, blobColor = 0, minArea = 1500; 
                double minCircularity = 0.1, minConvexity = 0.87, minInertiaRatio = 0.01;
                
                attr->Attribute("minThreshold", &minThreshold);                
                attr->Attribute("maxThreshold", &maxThreshold);                
                attr->Attribute("blobColor", &blobColor);                
                attr->Attribute("minArea", &minArea);                
                
                attr->Attribute("minCircularity", &minCircularity);                
                attr->Attribute("minConvexity", &minConvexity);                
                attr->Attribute("minInertiaRatio", &minInertiaRatio);                
                
                tmpA = new BlobAttribute();
                ((BlobAttribute*)tmpA)->SetParams(minThreshold, maxThreshold, blobColor, minArea, minCircularity, minConvexity, minInertiaRatio);
                break;                
            }
            case DEPTH_A:
            {
                double depth_scale = 0.001;
                attr->Attribute("depthScale", &depth_scale);                
                tmpA = new DepthAttribute(depth_scale);
                break;
            }
            case ROUGH_DIST_A:
            {
                double realW = 0, realH = 0;
                attr->Attribute("realWidth",&realW);
                attr->Attribute("realHeight",&realH);
                tmpA = new RoughDistAttribute(realW, realH);
                break;
            }
            case DIST_A:
            {
                double md = -1, Md = -1;
                attr->Attribute("minDist", &md);
                attr->Attribute("maxDist", &Md);
                tmpA = new DistanceAttribute(md, Md);
                break;
            }
#if (USE_DLIB)
            case FACE_DLIB_A:
            {
                string base_dir_path = getPathAttribute(attr, "base_dir_path");
                
                string base_file_path = getPathAttribute(attr, "base_file_path");
                
                string sp_path = getPathAttribute(attr, "sp_path");
                
                string net_path = getPathAttribute(attr, "net_path");                                           
                
                tmpA = new FaceDlibAttribute(base_dir_path, base_file_path, sp_path, net_path);
                break;
            }
#endif
            default:
            {
                attr = attr->NextSiblingElement("Attribute");
                continue;
            }
                
            }                                    
            double prob = 0.75;
            attr->Attribute("Probability",&prob);  
            tmpA->setProbability(prob);
            
            tmpA->Name = name;            
            tmpA->Weight = weight;
            tmpA->parent_base  = this;            
            
            // contour and 3d
            string contour;
            if( TIXML_SUCCESS == attr->QueryValueAttribute("Contour",&contour) ){
                transform(contour.begin(), contour.end(), contour.begin(),[](unsigned char c){ return tolower(c); });
                if( contour == "false" )
                    tmpA->returnContours = false;
            }
            
            // clusterization
            TiXmlElement *cluseter_el = attr->FirstChildElement("Clusterization");
            if( cluseter_el ){
                string clusterTypeStr = cluseter_el->Attribute("Type");
                ClusterizationType type = getClusterizationTypeFromName(clusterTypeStr);
                if( type == FOREL_C ){
                    double R = 0, eps = 0;
                    cluseter_el->Attribute("R", &R);
                    cluseter_el->Attribute("eps", &eps);
                    tmpA->clusterization_method = new ClusterForel(R, eps);
                }
            }
            
            // filtering
            TiXmlElement *filter_el = attr->FirstChildElement("Filter");
            while( filter_el ){
                string filterTypeStr = filter_el->Attribute("Type");
                
                FilterTypes filterType = getFilterTypeFromString(filterTypeStr);
                if( filterType == INSIDER_F ){
                    InsiderFilter* tmpF = new InsiderFilter();
                    tmpA->filters.push_back(tmpF);
                }
                else if( filterType == IOU_F ){
                    double threshold = 0.75;
                    filter_el->Attribute("threshold",&threshold);
                    IOUFilter* tmpF = new IOUFilter(threshold);
                    tmpA->filters.push_back(tmpF);
                }
                else if(filterType == ROI_F ){
                    int x = 0, y = 0, w = 0, h = 0;
                    filter_el->Attribute("x",&x);
                    filter_el->Attribute("y",&y);
                    filter_el->Attribute("w",&w);
                    filter_el->Attribute("h",&h);
                    Rect roi = Rect(x, y, w, h);                    
                    ROIFilter* tmpF = new ROIFilter(roi);
                    tmpA->filters.push_back(tmpF);
                }
                else if( filterType == UNK_F ){
                    printf("Unknown filter name %s in Attribute %s!\n",filterTypeStr.c_str(), name.c_str());
                }                
                filter_el = filter_el->NextSiblingElement("Filter");
            }
                                    
            attributes.push_back(tmpA);

            attr = attr->NextSiblingElement("Attribute");
        }
        return true;
    }
    
    // ------------------
    // ------------------
    // simple_objects
    // ------------------
    // ------------------
    bool ObjectBase::loadFromXMLso(TiXmlDocument *doc)
    {

        TiXmlElement *baseO = doc->FirstChildElement("SimpleObjectBase");
        if (!baseO) return false;

        TiXmlElement *obj = baseO->FirstChildElement("SimpleObject");

        while (obj){
            // get common params
            string object_name = obj->Attribute("Name");
            int ID;
            obj->Attribute("ID",&ID);            
            SimpleObject* temp;

#if CV_MAJOR_VERSION > 3          
            // TRACKER
            // see if there are some tracking settings
            TiXmlElement *tracker_el = obj->FirstChildElement("Tracker");
            if( tracker_el ){
                // create specifiec tracker object instance
                string trackerType(tracker_el->GetText());
                temp = new eodTracker(object_name, trackerType);
                double iou_threshold = 0.5;
                double decay_rate = 0.1;                
                tracker_el->Attribute("IOU", &iou_threshold);
                tracker_el->Attribute("decay", &decay_rate);
                ((eodTracker*)temp)->iou_threshold = iou_threshold;
                ((eodTracker*)temp)->decay = decay_rate;
                if( tracker_el->NextSiblingElement("Tracker") )
                    printf("Object %s has more that one Tracker insnace, every except first will be ignored!",object_name.c_str());                
            }
            else{
                // create regular object
                temp = new SimpleObject(object_name);            
            }    
#else
            temp = new SimpleObject(object_name);            
#endif
            
            temp->ID = ID;                        
            TiXmlElement *attr = obj->FirstChildElement("Attribute");
            while (attr){
                Attribute* tempA = getByNameA( attr->GetText() );
                if( tempA == NULL ){
                    printf("There are no attribute with name %s! \n",attr->GetText());
                }
                else{
                    
                    const char* channel_char = attr->Attribute("Channel");       
                    AttributeChannel channel;
                    if(!channel_char)
                        channel = RGB;
                    else{
                        string channel_str = string(channel_char);
                        transform(channel_str.begin(), channel_str.end(), channel_str.begin(),[](unsigned char c){ return tolower(c); });
                        if( channel_str == "rgb" )
                            channel = RGB;
                        else if( channel_str == "depth" )
                            channel = DEPTH;
                        else{
                            channel = RGB;
                            printf("Unknown channel %s in attribute %s, used RGB as default!\n",channel_char, attr->GetText());
                        }                            
                    }
                    
                    string type = attr->Attribute("Type");  //TODO unsigned char*
                    transform(type.begin(), type.end(), type.begin(),[](unsigned char c){ return tolower(c); });
                    
                    if( type == "check"){
                        temp->AddModeAttribute( CHECK, channel, tempA );
                    }
                    else if( type == "detect" ){
                        temp->AddModeAttribute( DETECT, channel, tempA );
                    }
                    else if( type == "extract" ){
                        temp->AddModeAttribute( EXTRACT, channel, tempA );
                    }
                    else{
                        printf("Unknown mode %s in %s simple object!\n",type.c_str(), object_name.c_str());
                        obj = obj->NextSiblingElement("SimpleObject");
                        continue;
                    }
                }
                attr = attr->NextSiblingElement("Attribute");
            }            
            /*
            int cluster_i;
            obj->Attribute("cluseter",&cluster_i);
            bool cluster = bool(cluster_i);
            temp->cluster = cluster;
            if (cluster){
                int r_cluster;
                obj->Attribute("r_cluster",&r_cluster);
                temp->r_cluster = r_cluster;
            } 
            */
            // COMMON PARAMS HERE
            double Probability = 0.75;
            obj->Attribute("Probability",&Probability);
            temp->Probability = Probability;
            double IOU = 0.75;
            obj->Attribute("IOU",&IOU);
            temp->iou_threshold_d = IOU;
                        
            const char* identify_mode_c = obj->Attribute("Mode");
            if( identify_mode_c == NULL ){                
                temp->identify_mode = STRONG;
            }
            else{                
                string identify_mode(identify_mode_c);
                transform(identify_mode.begin(), identify_mode.end(), identify_mode.begin(),[](unsigned char c){ return tolower(c); });
                
                if( identify_mode == "hard" ){
                    temp->identify_mode = STRONG;
                }
                else if( identify_mode == "soft" ){
                    temp->identify_mode = WEAK;
                }
                else{
                    temp->identify_mode = STRONG;
                }
            }                                                
            
            const char* merging_policy_c = obj->Attribute("MergingPolicy");
            if( merging_policy_c == NULL ){
                temp->merging_policy = INTERSECTION_MP;
            }
            else{
                string merging_policy(merging_policy_c);
                transform(merging_policy.begin(), merging_policy.end(), merging_policy.begin(),[](unsigned char c){ return tolower(c); });
                
                if( merging_policy == "intersection" ){
                    temp->merging_policy = INTERSECTION_MP;
                }
                else if( merging_policy == "union" ){
                    temp->merging_policy = UNION_MP;
                }
                else{
                    temp->merging_policy = INTERSECTION_MP;
                }
            }
            
            simple_objects.push_back(temp);
            obj = obj->NextSiblingElement("SimpleObject");
        }
        return true;
    }


    bool ObjectBase::checkIDobj(){
      vector<int>ids;
      for(int i = 0 ; i < simple_objects.size(); i ++ ){
        for( int j = 0 ; j < ids.size();j++){
            if( simple_objects[i]->ID == ids[j] ){
                printf("Warning: some simple_objects in base have same ID.\n");
                return false;
            }
        }
        ids.push_back(simple_objects[i]->ID);	
      }
      return true;
    }    
    
  // --------------------
  // relationships
  // --------------------
  bool ObjectBase::loadFromXMLr(TiXmlDocument *doc){
        TiXmlElement *baseO = doc->FirstChildElement("RelationLib");
        if (!baseO) return false;

        TiXmlElement *rel = baseO->FirstChildElement("RelationShip");
        while( rel ){
            RelationShip* tmp_r;
            string type_name = rel->Attribute("Type");            
            RelationTypes type = getRelationTypeFromName(type_name);
            
            switch(type)
            {
                case IM_RANGE_R:
                    tmp_r = new ImageRangeRelation(rel);
                    break;
                case LOG_AND_R:
                {
                    string A_name = rel->Attribute("A");
                    RelationShip* A = getByNameR(A_name);
                    if( !A ){
                        rel = rel->NextSiblingElement("RelationShip");                                                    
                        printf("Error! Relation with name %s has not been found in ObjectBase!\n",A_name.c_str());
                        continue;                            
                    }                    
                    string B_name = rel->Attribute("B");
                    RelationShip* B = getByNameR(B_name);
                    if( !B ){
                        rel = rel->NextSiblingElement("RelationShip");                                                    
                        printf("Error! Relation with name %s has not been found in ObjectBase!\n",B_name.c_str());
                        continue;                            
                    }        
                    tmp_r = new AndRelation(A, B);
                    break;
                }
                case LOG_OR_R:
                {
                    string A_name = rel->Attribute("A");
                    RelationShip* A = getByNameR(A_name);
                    if( !A ){
                        rel = rel->NextSiblingElement("RelationShip");                                                    
                        printf("Error! Relation with name %s has not been found in ObjectBase!\n",A_name.c_str());
                        continue;                            
                    }                    
                    string B_name = rel->Attribute("B");
                    RelationShip* B = getByNameR(B_name);
                    if( !B ){
                        rel = rel->NextSiblingElement("RelationShip");                                                    
                        printf("Error! Relation with name %s has not been found in ObjectBase!\n",B_name.c_str());
                        continue;                            
                    }        
                    tmp_r = new OrRelation(A, B);
                    break;
                }
                case LOG_NOT_R:
                {
                    string A_name = rel->Attribute("A");
                    RelationShip* A = getByNameR(A_name);
                    if( !A ){
                        rel = rel->NextSiblingElement("RelationShip");                                                    
                        printf("Error! Relation with name %s has not been found in ObjectBase!\n",A_name.c_str());
                        continue;                            
                    }                                        
                    tmp_r = new NotRelation(A);
                    break;
                }
                case TD_RANGE_R:
                    tmp_r = new ThreeDimRangeRelation(rel);
                    break;
                case SPACE_IN_R:
                    tmp_r = new SpaceInRelation();
                    break;
                case SPACE_OUT_R:
                    tmp_r = new SpaceOutRelation();
                    break;
                case SPACE_UP_R:
                    tmp_r = new SpaceUpRelation();
                    break;
                case SPACE_DOWN_R:
                    tmp_r = new SpaceDownRelation();
                    break;
                case SPACE_LEFT_R:
                    tmp_r = new SpaceLeftRelation();
                    break;
                case SPACE_RIGHT_R:
                    tmp_r = new SpaceRightRelation();
                    break;
                
                
                default:
                    rel = rel->NextSiblingElement("RelationShip");                                    
                    printf("Error! Relation of type %s is unknown!\n",type_name.c_str());
                    continue;
            }
                     
            if( !tmp_r->setName(rel->Attribute("Name")) ){                
                rel = rel->NextSiblingElement("RelationShip");                                                    
                printf("Error! Relation of type %s has no \'Name\' attribute provided!\n",type_name.c_str());
                continue;
            }
                
            relations.push_back(tmp_r);
            rel = rel->NextSiblingElement("RelationShip");                                    
        }
        return true;
    }    
    
    // -------------------
    //  Complex Objects 
    // -------------------
    bool ObjectBase::loadFromXMLsNM (TiXmlDocument *doc){

        TiXmlElement *baseO = doc->FirstChildElement("ComplexObjectBase");

        if (!baseO) return false;

        TiXmlElement *scene = baseO->FirstChildElement("ComplexObject");


        while (scene){
            ComplexObject* tmpNs = new ComplexObject();
            // simple_objects READ
            TiXmlElement *object = scene->FirstChildElement("SimpleObject");
            String name = scene->Attribute("Name");
            tmpNs->name = name;
            int ID;
            scene->Attribute("ID",&ID);
            tmpNs->ID = ID;            

            while (object){
                tmpNs->AddObject(getByName(object->Attribute("Class")),object->Attribute("InnerName"));
                object = object ->NextSiblingElement("SimpleObject");
            }

            // RELATIONS READ
            TiXmlElement *relation = scene->FirstChildElement("Relation");
            while(relation){
                
                RelationShip* r = getByNameR(relation->Attribute("Relationship"));
                if( !r ){
                    printf("RelationShip %s has not been found!\n",relation->Attribute("Relationship"));
                }
                
                tmpNs->AddRelation(r,relation->Attribute("Obj1"),relation->Attribute("Obj2"));

                relation = relation->NextSiblingElement("Relation");
            }
            complex_objects.push_back(tmpNs);
            scene = scene->NextSiblingElement("ComplexObject");
        }
        return true;
    }

    
    SimpleObject* ObjectBase::getByName(string objectname){
        for( int i = 0 ; i < simple_objects.size(); i++){
            if( simple_objects[i]->name == objectname)
                return simple_objects[i];
        }
        return NULL;
    }

    ComplexObject* ObjectBase::getByNameNS(string scenename){
        for( int i = 0 ; i <  complex_objects.size(); i++){
            if( complex_objects[i]->name == scenename)
                return complex_objects[i];
        }
        return NULL;
    }

    Attribute* ObjectBase::getByNameA(string aname){
        for( int i = 0 ; i < attributes.size(); i++){
            if( attributes[i]->Name == aname)
                return attributes[i];
        }
        return NULL;
    }
    
    RelationShip* ObjectBase::getByNameR(string relname){
        for( int i = 0 ; i < relations.size(); i++){
            if( relations[i]->Name == relname)
                return relations[i];
        }
        return NULL;
    }

    void ObjectBase::printContent(){
        printf("Base contetents %lu simple_objects.\n",simple_objects.size());
        for( int i = 0 ; i < simple_objects.size(); i++){
            simple_objects[i]->printInfo();
        }
        printf("Base contetents %lu relation%c.\n",relations.size(), relations.size() == 1 ? ' ' : 's');
        for( int i = 0 ; i < relations.size(); i++){
            //printf("Relation %s",relations[i]->Name.c_str());
            //relations[i]->printInfo();	   
        }
    }

    void ObjectBase::printDetected(){
        printf("++++++++++++++STEP++++++++++++++\n");
        for( int i = 0 ; i < simple_objects.size(); i++){
            simple_objects[i]->printInfo();
            for(size_t j = 0 ; j < simple_objects[i]->objects.size(); j++){
                printf("(%i, %i, %i, %i) ",simple_objects[i]->objects[j].x,simple_objects[i]->objects[j].y,simple_objects[i]->objects[j].height,simple_objects[i]->objects[j].width);
            }
        }
    }

    void ObjectBase::clear(){
        simple_objects.clear();
        complex_objects.clear();
        attributes.clear();
        relations.clear();
    }

    void ObjectBase::clear_stuff(){
        for( size_t i= 0 ; i < simple_objects.size() ; i++){
            simple_objects[i]->objects.clear();            
        }
    }
    
    // camera params    
    void ObjectBase::setCameraParams(cv::Mat cameraMatrix_, cv::Mat distCoeffs_){
        cameraMatrix = cameraMatrix_;
        distCoeffs = distCoeffs_;
    }
    
    cv::Mat ObjectBase::getCameraMatrix(){
        return cameraMatrix;
    }
    
    cv::Mat ObjectBase::getDistortionCoeff(){
        return distCoeffs;
    }
    
    SimpleObject* ObjectBase::getSimpleObjectByID(int id){
        for( size_t i = 0 ; i < simple_objects.size(); i++ ){
            if( id == simple_objects[i]->ID )
                return simple_objects[i];
        }
        return NULL;
    }
    
    ComplexObject* ObjectBase::getComplexObjectByID(int id){
        for( size_t i = 0 ; i < complex_objects.size(); i++ ){
            if( id == complex_objects[i]->ID )
                return complex_objects[i];
        }
        return NULL;
    }

}
