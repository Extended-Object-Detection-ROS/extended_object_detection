#include "eod_node.h"
#include <cstdint>
#include <sensor_msgs/image_encodings.h>

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
    }
    
    detect(rgb, cv::Mat());
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
    
    detect(rgb, depth);
}

void EOD_ROS::detect(const cv::Mat& rgb, const cv::Mat& depth){
    prev_detected_time = ros::Time::now();
    
    
    frame_sequence++;
    
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
