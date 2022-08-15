#include "eod_node.h"


EOD_ROS::EOD_ROS(ros::NodeHandle nh){
    nh_ = nh;    
    
    rgb_it_ = new image_transport::ImageTransport(nh_);    
            
    sub_rgb_.subscribe(*rgb_it_, "camera/image_raw", 1);
        
    nh_p_.param("subscribe_depth", subscribe_depth, false);        
    
    if( !subscribe_depth){
        ROS_INFO("Configuring filter on rgb and info...");
        sub_info_.subscribe(nh_, "camera/info", 1);                            
    
        sync_.reset( new Synchronizer(RGBInfoSyncPolicy(10), sub_rgb_, sub_info_) );
        sync_->registerCallback(boost::bind(&EOD_ROS::rgb_info_cb, this, boost::placeholders::_1,  boost::placeholders::_2));
                                
    }
    else{
        image_transport::SubscriberFilter sub_depth_;    
        
    }      
    message_filters::Synchronizer<RGBInfoSyncPolicy> sync(RGBInfoSyncPolicy(10), sub_rgb_, sub_info_);            
    ROS_INFO("Configured!");
}

void EOD_ROS::rgb_info_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info){
    ROS_INFO("Get Image!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extended_object_detection_node");
    ros::NodeHandle nh_;
    
    ROS_INFO("Extended object detector started...");
    
    EOD_ROS eod_ros(nh_);
    
    ros::spin();
                
    return 0;
}
