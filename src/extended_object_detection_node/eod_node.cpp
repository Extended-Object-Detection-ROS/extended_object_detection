#include "eod_node.h"
#include <cstdint>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Point.h>


geometry_msgs::Vector3 getUnitTranslation(cv::Point point, const cv::Mat& K){
    geometry_msgs::Vector3 unit_translate;
    unit_translate.x = (point.x - K.at<double>(0,2)) / K.at<double>(0,0);
    unit_translate.y = (point.y - K.at<double>(1,2)) / K.at<double>(1,1);
    unit_translate.z = 1;
    return unit_translate;
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
    nh_p_.param("publish_output", publish_output, false);
    nh_p_.param("use_actual_time", use_actual_time, false);
    
    
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
            
    // set up publishers
    simple_objects_pub_ = nh_p.advertise<extended_object_detection::SimpleObjectArray>("simple_objects",1);
        
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

void EOD_ROS::rgb_info_cb(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& rgb_info){
    ROS_INFO("Get Image!");
    
    // CHECK RATE
    //if( !check_time(rgb_image->header.stamp) ) {
    if( !check_time(ros::Time::now()) ) {
        ROS_WARN("Skipped frame");
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
    
    eod::InfoImage ii = eod::InfoImage(rgb, getK(rgb_info), cv::Mat() );    
    detect(ii, eod::InfoImage(), rgb_image->header);
    //detect(rgb, cv::Mat(), rgb_image->header);
}

void EOD_ROS::rgbd_info_cb(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& rgb_info, const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_info){
    ROS_INFO("Got RGBD!");
    
    // CHECK RATE   
    //if( !check_time(rgb_image->header.stamp) ) {
    if( !check_time(ros::Time::now()) ) {
        ROS_WARN("Skipped frame");
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
    
    eod::InfoImage ii_rgb = eod::InfoImage(rgb, getK(rgb_info), cv::Mat() );
    eod::InfoImage ii_depth = eod::InfoImage(depth, getK(depth_info), cv::Mat() );        
    
    detect(ii_rgb, ii_depth, rgb_image->header);
}

//void EOD_ROS::detect(const cv::Mat& rgb, const cv::Mat& depth, std_msgs::Header header){
void EOD_ROS::detect(const eod::InfoImage& rgb, const eod::InfoImage& depth, std_msgs::Header header){
    ROS_INFO("Detecting...");
    prev_detected_time = ros::Time::now();
    
    cv::Mat image_to_draw;
    
    extended_object_detection::SimpleObjectArray simples_msg;
    
    
    if(publish_output)
        image_to_draw = rgb.clone();
    
    // detect simple objects
    for (auto& s_it : selected_simple_objects){
        //ROS_INFO("Identifiyng...");
        s_it->Identify(rgb, depth, frame_sequence);        
        //ROS_INFO("Adding...");
        add_data_to_simple_msg(&(*s_it), simples_msg, rgb.K);
    }
    
    simples_msg.header = header;
    if( use_actual_time )
        simples_msg.header.stamp = ros::Time::now();
        
    simple_objects_pub_.publish(simples_msg);
    
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
    
    for( auto& exi : eoi->extracted_info){
        bo.extracted_info.keys.push_back(exi.first);
        bo.extracted_info.values.push_back(exi.second);        
    }
    
    // rect
    bo.rect.left_bottom.x = eoi->x;
    bo.rect.left_bottom.y = eoi->y;
    bo.rect.right_up.x = eoi->x + eoi->width;
    bo.rect.right_up.y = eoi->y + eoi->height;
    
    // transform
    if( eoi->tvec.size() > 0 ){
        bo.transform.translation.x = eoi->tvec[0][0];
        bo.transform.translation.y = eoi->tvec[0][1];
        bo.transform.translation.z = eoi->tvec[0][2];
    }
    else{
        bo.transform.translation = getUnitTranslation(eoi->getCenter(), K);      
    }
    
    
    //TODO tracks
    
    return bo;
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
