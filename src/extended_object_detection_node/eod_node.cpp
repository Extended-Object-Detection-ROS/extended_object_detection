#include "eod_node.h"
#include <cstdint>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_utils.h"


geometry_msgs::Vector3 fromCvVector(cv::Vec3d cv_vector){
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
        
    // get params
    nh_p_.param("subscribe_depth", subscribe_depth, false);
    nh_p_.param("rate_limit_sec", rate_limit_sec, 0.1);
    nh_p_.param("publish_image_output", publish_image_output, false);
    nh_p_.param("use_actual_time", use_actual_time, false);
    nh_p_.param("publish_markers", publish_markers, false);
        
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
        ROS_INFO("All objects selected.");
        for( size_t i = 0 ; i < object_base->simple_objects.size() ; i++ ){
            selected_simple_objects.push_back(object_base->simple_objects[i]);
        }
    }
    else{
        if(selected_on_start_simple_objects[0] != -1){
            for( size_t i = 0 ; i < object_base->simple_objects.size() ; i++ ){
                if( find(selected_on_start_simple_objects.begin(), selected_on_start_simple_objects.end(), object_base->simple_objects[i]->ID) != selected_on_start_simple_objects.end() )
                    selected_simple_objects.push_back(object_base->simple_objects[i]);
            }        
        }        
    }
    ROS_INFO("Selected to detect on start %li objects", selected_simple_objects.size());
    
    set_simple_objects_srv_ = nh_p_.advertiseService("set_simple_objects", &EOD_ROS::set_simple_objects_cb, this);
            
    // set up publishers
    simple_objects_pub_ = nh_p_.advertise<extended_object_detection::SimpleObjectArray>("simple_objects",1);
    if( publish_markers)
        simple_objects_markers_pub_ = nh_p_.advertise<visualization_msgs::MarkerArray>("simple_objects_markers",1);
    
    private_it_ = new image_transport::ImageTransport(nh_p_);
    output_image_pub_ = private_it_->advertise("detected_image", 1);
        
    // set up message filters
    if( !subscribe_depth){
        ROS_INFO("Configuring filter on rgb image and info...");        
    
        rgb_sync_.reset( new RGBSynchronizer(RGBInfoSyncPolicy(10), sub_rgb_, sub_info_) );
        rgb_sync_->registerCallback(boost::bind(&EOD_ROS::rgb_info_cb, this, boost::placeholders::_1,  boost::placeholders::_2));                                
    }
    else{
        ROS_INFO("Configuring filter on rgbd images and infos...");
        
        sub_depth_.subscribe(*rgb_it_, "depth/image_raw", 10);
        sub_depth_info_.subscribe(nh_, "depth/info", 10);
        
        rgbd_sync_.reset( new RGBDSynchronizer(RGBDInfoSyncPolicy(10), sub_rgb_, sub_info_, sub_depth_, sub_depth_info_) );
        rgbd_sync_->registerCallback(boost::bind(&EOD_ROS::rgbd_info_cb, this, boost::placeholders::_1,  boost::placeholders::_2, boost::placeholders::_3,  boost::placeholders::_4));        
    }          
    ROS_INFO("Configured!");
}

bool EOD_ROS::check_time(ros::Time stamp){
    if( frame_sequence == 0)
        return true;        
    return (stamp - prev_detected_time).toSec() > rate_limit_sec;
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
    ROS_INFO("Get Image!");
    
    // CHECK RATE    
    if( !check_time(ros::Time::now()) ) {
        ROS_WARN("Skipped frame");
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
    
    eod::InfoImage ii = eod::InfoImage(rgb, getK(rgb_info), getD(rgb_info) );    
    detect(ii, eod::InfoImage(), rgb_image->header);    
}

void EOD_ROS::rgbd_info_cb(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& rgb_info, const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_info){
    ROS_INFO("Got RGBD!");
    
    // CHECK RATE       
    if( !check_time(ros::Time::now()) ) {
        ROS_WARN("Skipped frame");
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
    }
    else if(depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1){        
        depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    }
    else{
        ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_image->encoding.c_str());
    }    
    eod::InfoImage ii_rgb = eod::InfoImage(rgb, getK(rgb_info), getD(rgb_info) );
    eod::InfoImage ii_depth = eod::InfoImage(depth, getK(depth_info), getD(depth_info) );            
    detect(ii_rgb, ii_depth, rgb_image->header);
}


void EOD_ROS::detect(const eod::InfoImage& rgb, const eod::InfoImage& depth, std_msgs::Header header){
    ROS_INFO("Detecting...");
    prev_detected_time = ros::Time::now();
    
    cv::Mat image_to_draw;
    
    extended_object_detection::SimpleObjectArray simples_msg;
    
    
    if(publish_image_output)
        image_to_draw = rgb.clone();
    
    // detect simple objects
    for (auto& s_it : selected_simple_objects){
        //ROS_INFO("Identifiyng...");
        s_it->Identify(rgb, depth, frame_sequence);        
        //ROS_INFO("Adding...");
        add_data_to_simple_msg(&(*s_it), simples_msg, rgb.K());
        if(publish_image_output)
            s_it->draw(image_to_draw, cv::Scalar(0, 255, 0));
    }
    
    simples_msg.header = header;
    if( use_actual_time )
        simples_msg.header.stamp = ros::Time::now();
        
    simple_objects_pub_.publish(simples_msg);
    
    if(publish_markers){
        visualization_msgs::MarkerArray mrk_array_msg;    
        int id_cnt = 0;        
        for(auto& bo : simples_msg.objects){            
            mrk_array_msg.markers.push_back(base_object_to_marker_arrow(bo, rgb.K(), header, cv::Scalar(0, 255, 0),id_cnt));
            mrk_array_msg.markers.push_back(base_object_to_marker_frame(bo, rgb.K(), header, cv::Scalar(0, 255, 0),id_cnt));
            id_cnt++;
        }
        simple_objects_markers_pub_.publish(mrk_array_msg);
    }    
    
    if(publish_image_output){
        sensor_msgs::ImagePtr detected_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_to_draw).toImageMsg();
        output_image_pub_.publish(detected_image_msg);
    }        
    frame_sequence++;    
}


void EOD_ROS::add_data_to_simple_msg( eod::SimpleObject* so, extended_object_detection::SimpleObjectArray& msg, const cv::Mat& K){    
    for(auto& eoi : so->objects){
        msg.objects.push_back(eoi_to_base_object(so, &eoi, K));
    }            
}

extended_object_detection::BaseObject EOD_ROS::eoi_to_base_object( eod::SimpleObject* so,  eod::ExtendedObjectInfo* eoi, const cv::Mat& K){
    //ROS_INFO("Forming...");
    extended_object_detection::BaseObject bo;
    // common
    bo.type_id = so->ID;
    bo.type_name = so->name;
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
        cv::Rodrigues(eoi->rvec, rotMat);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extended_object_detection_node");
    ros::NodeHandle nh_, nh_p_("~");
    
    ROS_INFO("Extended object detector started...");
    
    EOD_ROS eod_ros(nh_, nh_p_);
    
    ros::spin();
                
    return 0;
}
