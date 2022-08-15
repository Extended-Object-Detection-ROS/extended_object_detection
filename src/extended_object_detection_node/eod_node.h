/*
 * Extneded Object Detection ROS Node
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

//#include "ObjectBase.h"


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> RGBInfoSyncPolicy;
        
typedef message_filters::Synchronizer<RGBInfoSyncPolicy> Synchronizer;


class EOD_ROS{
public:
    EOD_ROS(ros::NodeHandle nh);
    
    void rgb_info_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info);
    
private:
    // ros stuff
    ros::NodeHandle nh_, nh_p_;
    image_transport::ImageTransport *rgb_it_;
    image_transport::SubscriberFilter sub_rgb_;    
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    boost::shared_ptr<Synchronizer> sync_;
    
    // params
    bool subscribe_depth;        
    
};
