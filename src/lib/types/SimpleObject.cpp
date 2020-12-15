#include "SimpleObject.h"
#include "geometry_utils.h"
#include "drawing_utils.h"

using namespace std;
using namespace cv;

namespace eod{

    //-----------------------------------------------------------------
    // Object constructor
    //-----------------------------------------------------------------
    
    SimpleObject::SimpleObject(){
        defaultInit();        
    }

    SimpleObject::SimpleObject(string name_){
        defaultInit();
        name = name_;
    }
    
    void SimpleObject::defaultInit(){
        identified = false;
        totalWeight = 0;
        borderPc = 0.02;
        image_samples = 0;       
        identify_mode = STRONG;        
        merging_policy = INTERSECTION_MP;
    }

    //-----------------------------------------------------------------
    // SimpleObject interfaces
    //-----------------------------------------------------------------
    
    void SimpleObject::AddModeAttribute(AttributeMode mode, AttributeChannel channel, Attribute* attr){
        mode_attributes.push_back(make_pair(make_pair(mode, channel), attr));
    }
    
    //-----------------------------------------------------------------
    // Handle instruments
    //-----------------------------------------------------------------

    void SimpleObject::printInfo(){
    }

    string SimpleObject::getInfoStr(){
        string infoStr;
        return infoStr;
    }
    
    //-----------------------------------------------------------------
    // Ordinary Detection Stuff
    //-----------------------------------------------------------------
    vector<ExtendedObjectInfo> SimpleObject::Identify(const Mat& frame, const Mat& depth, int seq){
        if( identify_mode == STRONG ){
            IdentifyStrong(frame, depth, seq);
        }
        else if( identify_mode == WEAK ){
            IdentifyWeak(frame, depth, seq);
        }        
        return objects;
    }
    
    vector<ExtendedObjectInfo> SimpleObject::IdentifyStrong(const Mat& frame, const Mat& depth, int seq){
        // speed up fo only for one attribute objects
        if( mode_attributes.size() == 1 ){
            objects.clear(); // need I this?
            if( mode_attributes[0].first.first == DETECT ){
                if( mode_attributes[0].first.second == RGB)
                    objects = mode_attributes[0].second->Detect(frame, seq);
                else if( mode_attributes[0].first.second == DEPTH)
                    objects = mode_attributes[0].second->Detect(depth, seq);
                for( size_t i = 0 ; i < objects.size(); i++)
                    //objects[i].calcTotalScore();                
                    objects[i].mergeAllData(merging_policy);
            }
            // else TODO how about to use checker on whole image in such case
            return objects;
        }
                
        // more than one
        vector<ExtendedObjectInfo> prev;
        for (size_t i = 0; i < mode_attributes.size(); i++){
            if( mode_attributes[i].first.first == DETECT ){
                vector<ExtendedObjectInfo> rects;
                if( mode_attributes[i].first.second == RGB)
                    rects = mode_attributes[i].second->Detect(frame, seq);
                else if( mode_attributes[i].first.second == DEPTH)
                    rects = mode_attributes[i].second->Detect(depth, seq);
                
                if( rects.size()  == 0 ){
                    objects.clear();
                    return objects;
                }
                if( i == 0 ){
                    prev = rects;
                    objects = rects;
                }
                else{
                    prev = objects;   
                    Mat_<double> closenessMapD = createClosenessMap(&prev, &rects, iou_threshold_d);
                    Mat mask(closenessMapD.size(), CV_8UC1, Scalar(255,255,255));            
                    objects.clear();
                    while( true ){
                        double min, max = 0;
                        Point min_loc, max_loc;
                        minMaxLoc(closenessMapD, &min, &max, &min_loc, &max_loc, mask);
                        if( max == 0 ) break;                               
                            
                        ExtendedObjectInfo newone;
                        if( merging_policy == INTERSECTION_MP )
                            newone = prev[max_loc.y] & rects[max_loc.x];   
                        else if( merging_policy == UNION_MP )
                            newone = prev[max_loc.y] | rects[max_loc.x];   
                        
                        objects.push_back(newone); 
                        mask.row(max_loc.y).setTo(Scalar(0,0,0));                                    
                        mask.col(max_loc.x).setTo(Scalar(0,0,0));                                
                    }
                    mask.release();
                    closenessMapD.release();
                    if( objects.size() == 0 ){                    
                        return objects;
                    }
                    prev.clear();                                 
                }
            }
            else if(mode_attributes[i].first.first == CHECK){              
                if( mode_attributes[i].first.second == RGB)
                    mode_attributes[i].second->Check(frame, &objects);
                else if( mode_attributes[i].first.second == DEPTH)
                    mode_attributes[i].second->Check(depth, &objects);
                if( objects.size() == 0 ){
                    return objects;                           
                }
            }           
            else if(mode_attributes[i].first.first == EXTRACT){                   
                if( mode_attributes[i].first.second == RGB)
                    mode_attributes[i].second->Extract(frame, &objects);
                else if( mode_attributes[i].first.second == DEPTH)
                    mode_attributes[i].second->Extract(depth, &objects);
            }
        }
        for( size_t i = 0 ; i < objects.size(); i++)
            //objects[i].calcTotalScore();
            objects[i].mergeAllData(merging_policy);
        return objects;
    }

    
    vector<ExtendedObjectInfo> SimpleObject::IdentifyWeak(const Mat& frame, const Mat& depth, int seq){
        ExtendedObjectInfo fake_empty_rect = ExtendedObjectInfo(0,0,frame.size().width, frame.size().height);
        vector<ExtendedObjectInfo> prev;
        
        objects.clear();
        for (size_t i = 0; i < mode_attributes.size(); i++){            
            if( mode_attributes[i].first.first == DETECT ){                                
                if( i == 0 ){
                    if( mode_attributes[0].first.second == RGB)
                        objects = mode_attributes[0].second->Detect(frame, seq);
                    else if( mode_attributes[0].first.second == DEPTH)
                        objects = mode_attributes[0].second->Detect(depth, seq);
                }
                else{
                    prev = objects;                
                    vector<ExtendedObjectInfo> rects;
                    if( mode_attributes[i].first.second == RGB)
                        rects = mode_attributes[i].second->Detect(frame, seq);
                    else if( mode_attributes[i].first.second == DEPTH)
                        rects = mode_attributes[i].second->Detect(depth, seq);
                    
                    // matcher
                    Mat_<double> closenessMapD = createClosenessMap(&prev, &rects, iou_threshold_d);
                    Mat mask(closenessMapD.size(), CV_8UC1, Scalar(255,255,255));            
                    objects.clear();
                    vector<int> taken_prev;
                    vector<int> taken_rects;
                    while(true){
                        double min, max = 0;
                        Point min_loc, max_loc;
                        minMaxLoc(closenessMapD, &min, &max, &min_loc, &max_loc, mask);
                        if( max == 0 ) break;                               
                        ExtendedObjectInfo newone;
                        if( merging_policy == INTERSECTION_MP)
                            newone = prev[max_loc.y] & rects[max_loc.x];                        
                        else
                            newone = prev[max_loc.y] | rects[max_loc.x];                        
                        
                        objects.push_back(newone); 
                        mask.row(max_loc.y).setTo(Scalar(0,0,0));                                    
                        mask.col(max_loc.x).setTo(Scalar(0,0,0)); 
                        taken_prev.push_back(max_loc.y);
                        taken_rects.push_back(max_loc.x);
                    }
                    mask.release();
                    closenessMapD.release();
                    // care about untaken prevs
                    for( size_t j = 0 ; j < prev.size() ; j++ ){
                        if( find(taken_prev.begin(), taken_prev.end(), j) == taken_prev.end() ){
                            // set 0 for currnet attribute
                            prev[j].setScoreWeight(0, mode_attributes[i].second->Weight);
                            objects.push_back(prev[j]);
                        }
                    }
                    // care about untaken rects
                    for( size_t j = 0 ; j < rects.size() ; j++ ){
                        if( find(taken_rects.begin(), taken_rects.end(), j) == taken_rects.end() ){
                            ExtendedObjectInfo tmp = fake_empty_rect & rects[j];
                            objects.push_back(tmp);
                        }
                    }        
                    taken_prev.clear();
                    taken_rects.clear();
                }                                                
            }
            else if( mode_attributes[i].first.first == CHECK ){
                for( size_t j = 0 ; j < objects.size() ; j++ ){
                    bool check_result;
                    if( mode_attributes[i].first.second == RGB)
                        check_result = mode_attributes[i].second->Check2(frame, objects[j]);
                    else if( mode_attributes[i].first.second == DEPTH)
                        check_result = mode_attributes[i].second->Check2(depth, objects[j]);
                    if( ! check_result ){
                        objects[j].setScoreWeight(0, mode_attributes[i].second->Weight);
                    }
                }
            }            
            else if(mode_attributes[i].first.first == EXTRACT){
                if( mode_attributes[0].first.second == RGB)
                    mode_attributes[0].second->Extract(frame, &objects);
                else if( mode_attributes[0].first.second == DEPTH)
                    mode_attributes[0].second->Extract(depth, &objects);
            }
            fake_empty_rect.setScoreWeight(0, mode_attributes[i].second->Weight);                                                
        }
            
        // remove low score objects
        auto it = objects.begin();
        while (it != objects.end() ){   
            //it->calcTotalScore();
            it->mergeAllData(merging_policy);
            if( it->total_score < Probability ){
                it = objects.erase(it);
            }
            else
                ++it;
        }        
        return objects;
    }
      
    
    void SimpleObject::draw(Mat& image, Scalar col  ){
        for (size_t i = 0; i < objects.size(); i++){
            drawOne(image, &objects[i], col);
        }        
    }
    
    void SimpleObject::drawOne(Mat& image, ExtendedObjectInfo* obj, Scalar col){
        // draw basic rectangle, contour and so on            
        obj->draw(image, col);
        
        // add main object info
        string objectInfo = to_string(ID) +": "+ name + " ["+to_string(roundf(obj->total_score * 100) / 100).substr(0,4)+"]"+(obj->track_id == -1 ? "" : "("+to_string(obj->track_id)+")") ;                                    
        Point prevBr = drawFilledRectangleWithText(image, obj->tl(), objectInfo, col);
        
        //int real_indx = 0; // KOSTYLISH :C
        for(size_t j = 0 ; j < mode_attributes.size() ; j++ ){
            //if( mode_attributes[j].first.first == EXTRACT )
              //  continue;
            string symbol;
            if(mode_attributes[j].first.first == DETECT)
                symbol = " + ";
            else if(mode_attributes[j].first.first == CHECK)
                symbol = " - ";
            else
                symbol = " > ";
            
            string attributeName = symbol + mode_attributes[j].second->Name + " ["+(j < obj->scores_with_weights.size() ? to_string(roundf(obj->scores_with_weights[j].first * 100) / 100).substr(0,4) : "-1" )+"]" + (j >= obj->sub_id.size() || obj->sub_id[j] == -1? "" : "{"+to_string(obj->sub_id[j])+"}") + (j >= obj->extracted_info.size() || obj->extracted_info[j] == "" ? "" : "("+obj->extracted_info[j]+")");
            prevBr = drawFilledRectangleWithText(image, Point(obj->tl().x, prevBr.y), attributeName, col);
            
        }        
   }

}
