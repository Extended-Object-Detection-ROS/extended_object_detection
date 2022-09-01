#include "eod_node.h"
#include <cstdint>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_utils.h"
#include <geometry_msgs/TransformStamped.h>


geometry_msgs::Vector3 fromCvVector(const cv::Vec3d& cv_vector){
    geometry_msgs::Vector3 ros_vector;
    ros_vector.x = cv_vector[0];
    ros_vector.y = cv_vector[1];
    ros_vector.z = cv_vector[2];
    return ros_vector;
}


geometry_msgs::Point fromVector(const geometry_msgs::Vector3& vector){
    geometry_msgs::Point point;
    point.x = vector.x;
    point.y = vector.y;
    point.z = vector.z;
    return point;
}


EOD_ROS::EOD_ROS(ros::NodeHandle nh, ros::NodeHandle nh_p){
    nh_ = nh;
    nh_p_ = nh_p;
    
    frame_sequence = 0;
    
    rgb_it_ = new image_transport::ImageTransport(nh_);    
            
    sub_rgb_.subscribe(*rgb_it_, "camera/image_raw", 10);
    sub_info_.subscribe(nh_, "camera/info", 10);    
        
    detect_rate_values = new boost::circular_buffer<double>(10);
    
    // get params
    nh_p_.param("subscribe_depth", subscribe_depth, false);
    nh_p_.param("rate_limit_sec", rate_limit_sec, 0.1);
    nh_p_.param("publish_image_output", publish_image_output, false);
    nh_p_.param("use_actual_time", use_actual_time, false);
    nh_p_.param("publish_markers", publish_markers, false);
    nh_p_.param("broadcast_tf", broadcast_tf, false);
    nh_p_.param("allowed_lag_sec", allowed_lag_sec, 0.0);
            
    std::string object_base_path;
    nh_p_.getParam("object_base",object_base_path);
    
    // load base
    object_base = new eod::ObjectBase();
    if( !object_base->loadFromXML(object_base_path) ){
        ROS_ERROR("Error during loading object base in path '%s'!", object_base_path.c_str());
        std::exit(-1);
    }
    
    // // object selection
    std::vector<int>selected_on_start_simple_objects;
    nh_p_.getParam("selected_on_start_simple_objects",selected_on_start_simple_objects);    
    if( selected_on_start_simple_objects.size() == 0 ){
        //ROS_INFO("All objects selected.");
        for( size_t i = 0 ; i < object_base->simple_objects.size() ; i++ )
            selected_simple_objects.push_back(object_base->simple_objects[i]);        
    }
    else{
        if(selected_on_start_simple_objects[0] != -1){
            for( size_t i = 0 ; i < object_base->simple_objects.size() ; i++ ){
                if( find(selected_on_start_simple_objects.begin(), selected_on_start_simple_objects.end(), object_base->simple_objects[i]->ID) != selected_on_start_simple_objects.end() )
                    selected_simple_objects.push_back(object_base->simple_objects[i]);
            }        
        }        
    }
    ROS_INFO("Selected to detect on start %li simple objects", selected_simple_objects.size());
#ifdef USE_IGRAPH
    std::vector<int>selected_on_start_complex_objects;
    nh_p_.getParam("selected_on_start_complex_objects",selected_on_start_complex_objects);    
    if( selected_on_start_complex_objects.size() == 0 ){
        //ROS_INFO("All objects selected.");
        for( size_t i = 0 ; i < object_base->complex_objects_graph.size() ; i++ )
            selected_complex_objects.push_back(object_base->complex_objects_graph[i]);        
    }
    else{
        if(selected_on_start_complex_objects[0] != -1){
            for( size_t i = 0 ; i < object_base->complex_objects_graph.size() ; i++ ){
                if( find(selected_on_start_complex_objects.begin(), selected_on_start_complex_objects.end(), object_base->complex_objects_graph[i]->ID) != selected_on_start_complex_objects.end() )
                    selected_complex_objects.push_back(object_base->complex_objects_graph[i]);
            }        
        }        
    }
    ROS_INFO("Selected to detect on start %li complex objects", selected_complex_objects.size());    
#endif
    
    set_simple_objects_srv_ = nh_p_.advertiseService("set_simple_objects", &EOD_ROS::set_simple_objects_cb, this);
#ifdef USE_IGRAPH
    set_complex_objects_srv_ = nh_p_.advertiseService("set_complex_objects", &EOD_ROS::set_complex_objects_cb, this);
#endif
    // set up publishers
    simple_objects_pub_ = nh_p_.advertise<extended_object_detection::SimpleObjectArray>("simple_objects",1);
#ifdef USE_IGRAPH
    complex_objects_pub_ = nh_p_.advertise<extended_object_detection::ComplexObjectArray>("complex_objects",1);
#endif
    if( publish_markers){
        simple_objects_markers_pub_ = nh_p_.advertise<visualization_msgs::MarkerArray>("simple_objects_markers",1);
#ifdef USE_IGRAPH
        complex_objects_markers_pub_ = nh_p_.advertise<visualization_msgs::MarkerArray>("complex_objects_markers",1);
        scenes_markers_pub_ = nh_p_.advertise<visualization_msgs::MarkerArray>("scenes_markers",1);
#endif
    }
    
    private_it_ = new image_transport::ImageTransport(nh_p_);
    output_image_pub_ = private_it_->advertise("detected_image", 1);
        
    // set up message filters
    if( !subscribe_depth){
        //ROS_INFO("Configuring filter on rgb image and info...");            
        rgb_sync_.reset( new RGBSynchronizer(RGBInfoSyncPolicy(10), sub_rgb_, sub_info_) );
        rgb_sync_->registerCallback(boost::bind(&EOD_ROS::rgb_info_cb, this, boost::placeholders::_1,  boost::placeholders::_2));                                
    }
    else{
        //ROS_INFO("Configuring filter on rgbd images and infos...");        
        sub_depth_.subscribe(*rgb_it_, "depth/image_raw", 10);
        sub_depth_info_.subscribe(nh_, "depth/info", 10);
        
        rgbd_sync_.reset( new RGBDSynchronizer(RGBDInfoSyncPolicy(10), sub_rgb_, sub_info_, sub_depth_, sub_depth_info_) );
        rgbd_sync_->registerCallback(boost::bind(&EOD_ROS::rgbd_info_cb, this, boost::placeholders::_1,  boost::placeholders::_2, boost::placeholders::_3,  boost::placeholders::_4));        
    }          
    //ROS_INFO("Configured!");
}


bool EOD_ROS::check_time(const ros::Time& stamp){
    if( frame_sequence == 0)
        return true;        
    return (stamp - prev_detected_time).toSec() > rate_limit_sec;
}


bool EOD_ROS::check_lag(const ros::Time& stamp, double& lag){     
    if( allowed_lag_sec == 0)
        return true;
    lag = (ros::Time::now() - stamp).toSec();
    return lag <= allowed_lag_sec;
}


cv::Mat EOD_ROS::getK(const sensor_msgs::CameraInfoConstPtr& info_msg){
    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    for (size_t i=0; i<3; i++) {
        for (size_t j=0; j<3; j++) {
            K.at<double>(i, j) = info_msg->K[i*3+j];
        }
    }
    return K;
}


cv::Mat EOD_ROS::getD(const sensor_msgs::CameraInfoConstPtr& info_msg){
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
    for (size_t i=0; i<5; i++) {        
        D.at<double>(0,i) = info_msg->D[i];        
    }
    return D;
}


void EOD_ROS::rgb_info_cb(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& rgb_info){
    //ROS_INFO("Got Image!");
    // CHECK RATE
    if( !check_time(ros::Time::now()) ) {
        ROS_WARN("Skipped frame");
        return;
    }    
    double lag;
    if( !check_lag(rgb_image->header.stamp, lag) ) {
        ROS_WARN("Dropped frame, lag = %f", lag);
        return;
    }    
    cv::Mat rgb;
    try{
        rgb = cv_bridge::toCvCopy(rgb_image, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rgb_image->encoding.c_str());
        return;
    }    
    eod::InfoImage ii = eod::InfoImage(rgb, getK(rgb_info), getD(rgb_info) );
    detect(ii, eod::InfoImage(), rgb_image->header);
}


void EOD_ROS::rgbd_info_cb(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& rgb_info, const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_info){
    //ROS_INFO("Got RGBD!");    
    // CHECK RATE       
    if( !check_time(ros::Time::now()) ) {
        ROS_WARN("Skipped frame");
        return;
    }
    double lag;
    if( !check_lag(rgb_image->header.stamp, lag) ) {
        ROS_WARN("Dropped frame, lag = %f", lag);
        return;
    }
    // TODO add possibility to exclude old stamp images (if detection goes to slow)    
    cv::Mat rgb;
    try{
        rgb = cv_bridge::toCvCopy(rgb_image, "bgr8")->image;          
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rgb_image->encoding.c_str());
        return;
    }    
    cv::Mat depth;    
    if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1){    
        depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        depth.convertTo(depth, CV_32F);
        depth *= 0.001f;
    }
    else if(depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1){        
        depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    }
    else{
        ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_image->encoding.c_str());
    }    
    //ROS_INFO("Depth type is %i", depth.type());
    eod::InfoImage ii_rgb = eod::InfoImage(rgb, getK(rgb_info), getD(rgb_info) );
    eod::InfoImage ii_depth = eod::InfoImage(depth, getK(depth_info), getD(depth_info) );            
    detect(ii_rgb, ii_depth, rgb_image->header);
}


void EOD_ROS::detect(const eod::InfoImage& rgb, const eod::InfoImage& depth, std_msgs::Header header){
    ROS_INFO("Detecting...");
    if( frame_sequence != 0 ){
        detect_rate_values->push_back((ros::Time::now() - prev_detected_time).toSec());
    }
    prev_detected_time = ros::Time::now();
    
    cv::Mat image_to_draw;                
    if(publish_image_output)
        image_to_draw = rgb.clone();
    
    // detect simple objects
    extended_object_detection::SimpleObjectArray simples_msg;
    for (auto& s_it : selected_simple_objects){        
        s_it->Identify(rgb, depth, frame_sequence);                        
        for(auto& eoi : s_it->objects)
            simples_msg.objects.push_back(eoi_to_base_object(s_it->name, s_it->ID, &eoi, rgb.K()));
        if(publish_image_output)
            s_it->draw(image_to_draw, cv::Scalar(0, 255, 0));
    }
    // detect complex objects
#ifdef USE_IGRAPH
    extended_object_detection::ComplexObjectArray complex_msg;
    for(auto& c_it : selected_complex_objects){
        //ROS_INFO("Identifying complex...");
        c_it->Identify(rgb, depth, frame_sequence);
        //for(auto& eoi : c_it->complex_objects){
        for( size_t i = 0 ; i < c_it->complex_objects.size() ; i++ ){            
            extended_object_detection::ComplexObject cmplx_msg;
            
            cmplx_msg.complex_object = eoi_to_base_object(c_it->name, c_it->ID, &(c_it->complex_objects[i]), rgb.K());
            
            for( auto& name_eoi : c_it->simple_objects[i] ){// DANGER: I belive size complex_objects == size simple_objects
                cmplx_msg.simple_objects.push_back(eoi_to_base_object(name_eoi.first, -1, name_eoi.second, rgb.K()));
            }                        
            complex_msg.objects.push_back(cmplx_msg);
        }
        if(publish_image_output)
            c_it->drawAll(image_to_draw, cv::Scalar(255, 255, 0), 2);
    }
    
    visualization_msgs::MarkerArray scene_marker_array_msg;
    for(auto& scene_it : object_base->scenes){
        std::vector<std::pair<double, std::vector<std::pair<eod::SceneObject*, eod::ExtendedObjectInfo*>>>> scenes = scene_it->Identify(rgb, depth, frame_sequence);
        
        int ns = 0;
        for( auto& scene : scenes ){            
            scene_to_markers(scene, ns, scene_marker_array_msg);
            //scene_marker_array_msg.markers.insert();            
            ns++;            
        }
    }
    scenes_markers_pub_.publish(scene_marker_array_msg);
#endif    
    simples_msg.header = header;
    if( use_actual_time )
        simples_msg.header.stamp = ros::Time::now();
        
    simple_objects_pub_.publish(simples_msg);
#ifdef USE_IGRAPH
    complex_objects_pub_.publish(complex_msg);
#endif
    
    if(broadcast_tf){
        geometry_msgs::TransformStamped trsfrm;
        trsfrm.header = header;
        int id = 0;
        std::string prev_name = "";
        for( auto& bo : simples_msg.objects){            
            if( prev_name != bo.type_name)
                id = 0;
            trsfrm.child_frame_id = bo.type_name + "_" + std::to_string(id);
            trsfrm.transform = bo.transform;
            transform_broadcaster_.sendTransform(trsfrm);
            id++;
            prev_name = bo.type_name;
        }
#ifdef USE_IGRAPH
        for( auto& cbo : complex_msg.objects){
            if( prev_name != cbo.complex_object.type_name)
                id = 0;
            trsfrm.child_frame_id = cbo.complex_object.type_name + "_" + std::to_string(id);
            trsfrm.transform = cbo.complex_object.transform;
            transform_broadcaster_.sendTransform(trsfrm);
            id++;
            prev_name = cbo.complex_object.type_name;    
        }
#endif                        
    }
    
    if(publish_markers){
        visualization_msgs::MarkerArray mrk_array_msg;    
        int id_cnt = 0;        
        for(auto& bo : simples_msg.objects){            
            mrk_array_msg.markers.push_back(base_object_to_marker_arrow(bo, rgb.K(), header, cv::Scalar(0, 255, 0),id_cnt));
            mrk_array_msg.markers.push_back(base_object_to_marker_frame(bo, rgb.K(), header, cv::Scalar(0, 255, 0),id_cnt));
            mrk_array_msg.markers.push_back(base_object_to_marker_text(bo, rgb.K(), header, cv::Scalar(0, 255, 0),id_cnt));
            id_cnt++;
        }
        simple_objects_markers_pub_.publish(mrk_array_msg);
#ifdef USE_IGRAPH
        visualization_msgs::MarkerArray cmplx_mrk_array_msg;
        id_cnt = 0;        
        for(auto& co : complex_msg.objects){            
            cmplx_mrk_array_msg.markers.push_back(base_object_to_marker_arrow(co.complex_object, rgb.K(), header, cv::Scalar(0, 255, 255),id_cnt));
            cmplx_mrk_array_msg.markers.push_back(base_object_to_marker_frame(co.complex_object, rgb.K(), header, cv::Scalar(0, 255, 255),id_cnt));
            cmplx_mrk_array_msg.markers.push_back(base_object_to_marker_text(co.complex_object, rgb.K(), header, cv::Scalar(0, 255, 255),id_cnt));
            id_cnt++;
            for( auto& so : co.simple_objects){
                cmplx_mrk_array_msg.markers.push_back(base_object_to_marker_frame(so, rgb.K(), header, cv::Scalar(0, 255, 255),id_cnt));
                id_cnt++;
            }
        }
        
        complex_objects_markers_pub_.publish(cmplx_mrk_array_msg);
#endif
    }        
    if(publish_image_output){
        sensor_msgs::ImagePtr detected_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_to_draw).toImageMsg();
        output_image_pub_.publish(detected_image_msg);
    }        
    frame_sequence++;    
}

extended_object_detection::BaseObject EOD_ROS::eoi_to_base_object( std::string name, int id,  eod::ExtendedObjectInfo* eoi, const cv::Mat& K){
    //ROS_INFO("Forming...");
    extended_object_detection::BaseObject bo;
    // common
    bo.type_id = id;
    bo.type_name = name;
    bo.score = eoi->total_score;
    // extracted info
    for( auto& exi : eoi->extracted_info){
        bo.extracted_info.keys.push_back(exi.first);
        bo.extracted_info.values.push_back(exi.second);        
    }        
    // translation
    if( eoi->tvec.size() > 0 ){
        bo.transform.translation.x = eoi->tvec[0][0];
        bo.transform.translation.y = eoi->tvec[0][1];
        bo.transform.translation.z = eoi->tvec[0][2];
    }
    else{
        bo.transform.translation = fromCvVector(eod::get_translation(eoi->getCenter(), K, 1));
    }
    // rotation
    if (eoi->rvec.size() > 0 ){
        double quaternion[4];
        cv::Mat rotMat;
        cv::Rodrigues(eoi->rvec[0], rotMat);
        eod::getQuaternion( rotMat, quaternion);
        bo.transform.rotation.x = quaternion[0];
        bo.transform.rotation.y = quaternion[1];
        bo.transform.rotation.z = quaternion[2];
        bo.transform.rotation.w = quaternion[3];
    }   
    else // use zero quaternion
        bo.transform.rotation.w = 1;
    
    // rect
    bo.rect.left_bottom.x = eoi->x;
    bo.rect.left_bottom.y = eoi->y;
    bo.rect.right_up.x = eoi->x + eoi->width;
    bo.rect.right_up.y = eoi->y + eoi->height;
    
    // translation to rect's corners
    geometry_msgs::Vector3 temp_translation;
    for( auto& corner_p : eoi->getCorners() ){
        bo.rect.cornerTranslates.push_back(fromCvVector(eod::get_translation(eod::float2intPoint(corner_p), K, bo.transform.translation.z)));
    }             
        
    //TODO tracks    
    return bo;
}


visualization_msgs::Marker EOD_ROS::base_object_to_marker_arrow(extended_object_detection::BaseObject& base_object, const cv::Mat& K, std_msgs::Header header, cv::Scalar color, int id){
    visualization_msgs::Marker mrk;
    mrk.header = header;    
    mrk.ns = base_object.type_name +"_arrow";
    mrk.id = id;
    mrk.type = visualization_msgs::Marker::ARROW; 
    mrk.lifetime = ros::Duration(get_detect_rate());
    mrk.points.push_back(geometry_msgs::Point());
    geometry_msgs::Point end = fromVector(base_object.transform.translation);
    mrk.points.push_back(end);
    mrk.scale.x = 0.01;
    mrk.scale.y = 0.02;
    mrk.scale.z = 0.1; 
    mrk.pose.orientation.w = 1;
    mrk.color.r = color[0]/255;
    mrk.color.g = color[1]/255;
    mrk.color.b = color[2]/255;
    mrk.color.a = 1;
    return mrk;
}


visualization_msgs::Marker EOD_ROS::base_object_to_marker_frame(extended_object_detection::BaseObject& base_object, const cv::Mat& K, std_msgs::Header header, cv::Scalar color, int id){
    visualization_msgs::Marker mrk;
    mrk.header = header;    
    mrk.ns = base_object.type_name +"_frame";
    mrk.id = id;
    mrk.type = visualization_msgs::Marker::LINE_STRIP;    
    mrk.lifetime = ros::Duration(get_detect_rate());
    for( auto& corner : base_object.rect.cornerTranslates )
        mrk.points.push_back(fromVector(corner));
    mrk.points.push_back(mrk.points[0]);    
    mrk.scale.x = 0.02;
    mrk.scale.y = 0.02;
    mrk.scale.z = 0.1; 
    mrk.pose.orientation.w = 1;
    mrk.color.r = color[0]/255;
    mrk.color.g = color[1]/255;
    mrk.color.b = color[2]/255;
    mrk.color.a = 1;
    return mrk;
}


visualization_msgs::Marker EOD_ROS::base_object_to_marker_text(extended_object_detection::BaseObject& base_object, const cv::Mat& K, std_msgs::Header header, cv::Scalar color, int id){
    visualization_msgs::Marker mrk;
    mrk.header = header;    
    mrk.ns = base_object.type_name +"_text";
    mrk.id = id;
    mrk.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
    mrk.lifetime = ros::Duration(get_detect_rate());
    mrk.pose.position = fromVector(base_object.transform.translation);    
    mrk.pose.position.y = base_object.rect.cornerTranslates[0].y - 0.14; // place text upper top frame part
    mrk.text = std::to_string(base_object.type_id)+":"+base_object.type_name+"["+std::to_string(base_object.score).substr(0,4)+"]";
    mrk.scale.x = 0.02;
    mrk.scale.y = 0.02;
    mrk.scale.z = 0.1; 
    mrk.pose.orientation.w = 1;
    mrk.color.r = color[0]/255;
    mrk.color.g = color[1]/255;
    mrk.color.b = color[2]/255;
    mrk.color.a = 1;
    return mrk;
}


int EOD_ROS::find_simple_obj_index_by_id(int id){
    for( int i = 0 ; i < selected_simple_objects.size() ; i++ ){
        if( selected_simple_objects[i]->ID == id )
            return i;            
    }
    return -1;
}


bool EOD_ROS::set_simple_objects_cb(extended_object_detection::SetObjects::Request &req, extended_object_detection::SetObjects::Response &res){
    
    if( req.remove_all && req.add_all ){/*do literally nothing*/}
    else{
        if( req.remove_all ){
            selected_simple_objects.clear();    
        }
        else if( req.add_all ){
            selected_simple_objects.clear();
            selected_simple_objects.assign(object_base->simple_objects.begin(), object_base->simple_objects.end());            
        }
        else{
            for( auto& change : req.changes ){
                int object_id = abs(change);
                eod::SimpleObject* so = object_base->getSimpleObjectByID(object_id);
                if(so){
                    // add
                    if( change > 0 ){
                        if( find_simple_obj_index_by_id(object_id) != -1 ){
                            // already selected
                            ROS_WARN("[set_simple_objects srv] object with id %i is already selected", object_id);
                        }
                        else{
                            selected_simple_objects.push_back(so);
                        }
                    }
                    // remove
                    else{
                        int no_in_list = find_simple_obj_index_by_id(object_id);
                        if( no_in_list != -1 ){
                            selected_simple_objects.erase(selected_simple_objects.begin()+no_in_list);
                        }
                        else{
                            // already removed
                            ROS_WARN("[set_simple_objects srv] object with id %i is already removed", object_id);
                        }
                    }
                }
                else{
                    ROS_ERROR("[set_simple_objects srv] no object in base with id %i",object_id);
                }
            }
        }
    }
    for(auto& so : selected_simple_objects )
        res.result.push_back(so->ID);
    return true;
}


#ifdef USE_IGRAPH
int EOD_ROS::find_complex_obj_index_by_id(int id){
    for( int i = 0 ; i < selected_complex_objects.size() ; i++ ){
        if( selected_complex_objects[i]->ID == id )
            return i;            
    }
    return -1;
}


bool EOD_ROS::set_complex_objects_cb(extended_object_detection::SetObjects::Request &req, extended_object_detection::SetObjects::Response &res){
    
    if( req.remove_all && req.add_all ){/*do literally nothing*/}
    else{
        if( req.remove_all ){
            selected_complex_objects.clear();    
        }
        else if( req.add_all ){
            selected_complex_objects.clear();
            selected_complex_objects.assign(object_base->complex_objects_graph.begin(), object_base->complex_objects_graph.end());            
        }
        else{
            for( auto& change : req.changes ){
                int object_id = abs(change);
                eod::ComplexObjectGraph* co = object_base->getComplexObjectGraphByID(object_id);
                if(co){
                    // add
                    if( change > 0 ){
                        if( find_complex_obj_index_by_id(object_id) != -1 ){
                            // already selected
                            ROS_WARN("[set_simple_objects srv] object with id %i is already selected", object_id);
                        }
                        else{
                            selected_complex_objects.push_back(co);
                        }
                    }
                    // remove
                    else{
                        int no_in_list = find_complex_obj_index_by_id(object_id);
                        if( no_in_list != -1 ){
                            selected_complex_objects.erase(selected_complex_objects.begin()+no_in_list);
                        }
                        else{
                            // already removed
                            ROS_WARN("[set_simple_objects srv] object with id %i is already removed", object_id);
                        }
                    }
                }
                else{
                    ROS_ERROR("[set_simple_objects srv] no object in base with id %i",object_id);
                }
            }
        }
    }
    for(auto& co : selected_complex_objects )
        res.result.push_back(co->ID);
    return true;
}

void EOD_ROS::scene_to_markers(std::pair<double, std::vector<std::pair<eod::SceneObject*, eod::ExtendedObjectInfo*>>> scene, int ns, visualization_msgs::MarkerArray &scene_marker_array_msg){
    // calc center
    double cx = 0, cy = 0, cz = 0;
    for( auto& obj : scene.second ){
        cx += obj.first->x;
        cy += obj.first->y;
        cz += obj.first->z;
    }
    cx /= scene.second.size();
    cy /= scene.second.size();
    cz /= scene.second.size();
    cz += 1.5;
    // main text
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.ns = std::to_string(ns)+"_main_text";
    marker.lifetime = ros::Duration(get_detect_rate());
    marker.pose.position.x = cx;
    marker.pose.position.y = cy;
    marker.pose.position.z = cz + 0.4;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.4;
    marker.pose.orientation.w = 1;
    marker.color.g = 1;
    marker.color.a = 1;
    marker.text = std::to_string(ns) + ":" +std::to_string(scene.first);
    scene_marker_array_msg.markers.push_back(marker);
    
    // add objects
    int id = 0;
    for( auto& obj : scene.second ){
        scene_marker_array_msg.markers.push_back(scene_object_to_cylinder_marker(obj.first, ns, id));
        scene_marker_array_msg.markers.push_back(scene_object_to_text_marker(obj.first, ns, id));        
        scene_marker_array_msg.markers.push_back(scene_object_to_line_marker(obj.first, ns, id, cx, cy, cz));
        id++;
    }        
}

visualization_msgs::Marker EOD_ROS::scene_object_to_line_marker(eod::SceneObject* scene_obj, int ns, int id, double x, double y, double z){
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.ns = std::to_string(ns)+"_line";
    marker.id = id;
    marker.lifetime = ros::Duration(get_detect_rate());
    geometry_msgs::Point start;
    start.x = scene_obj->x;
    start.y = scene_obj->y;
    start.z = scene_obj->z;
    marker.points.push_back(start);
    geometry_msgs::Point end;
    end.x = x;
    end.y = y;
    end.z = z;
    marker.points.push_back(end);
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.2; 
    marker.pose.orientation.w = 1;
    marker.color.g = 1;
    marker.color.a = 1;
    return marker;    
}

visualization_msgs::Marker EOD_ROS::scene_object_to_cylinder_marker(eod::SceneObject* scene_obj, int ns, int id){
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.ns = std::to_string(ns)+"_cylinder";
    marker.id = id;
    marker.lifetime = ros::Duration(get_detect_rate());
    marker.pose.position.x = scene_obj->x;
    marker.pose.position.y = scene_obj->y;
    marker.pose.position.z = scene_obj->z;
    marker.scale.x = scene_obj->r*2;
    marker.scale.y = scene_obj->r*2;
    marker.scale.z = scene_obj->h;
    marker.pose.orientation.w = 1;
    marker.color.g = 1;
    marker.color.a = 1;
    return marker;
}

visualization_msgs::Marker EOD_ROS::scene_object_to_text_marker(eod::SceneObject* scene_obj, int ns, int id){
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.ns = std::to_string(ns)+"_text";
    marker.id = id;
    marker.lifetime = ros::Duration(get_detect_rate());
    marker.pose.position.x = scene_obj->x;
    marker.pose.position.y = scene_obj->y;
    marker.pose.position.z = scene_obj->z + scene_obj->h * 1.5;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.3;
    marker.pose.orientation.w = 1;
    marker.color.g = 1;
    marker.color.a = 1;
    marker.text = scene_obj->name;
    return marker;
}
#endif


double EOD_ROS::get_detect_rate(){
    if( detect_rate_values->empty() )
        return 0;
    double sum_rate = 0;
    for(auto& rate : *detect_rate_values)
        sum_rate += rate;
    return sum_rate / detect_rate_values->size();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "extended_object_detection_node");
    ros::NodeHandle nh_, nh_p_("~");
    
    ROS_INFO("Extended object detector started...");
    
    EOD_ROS eod_ros(nh_, nh_p_);
    
    ros::spin();
                
    return 0;
}
