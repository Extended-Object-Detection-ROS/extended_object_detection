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

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "extended_object_detection/BaseObject.h"
#include "extended_object_detection/SimpleObjectArray.h"
#include "extended_object_detection/ComplexObjectArray.h"

#include "ObjectBase.h"


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> RGBInfoSyncPolicy;
        
typedef message_filters::Synchronizer<RGBInfoSyncPolicy> RGBSynchronizer;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> RGBDInfoSyncPolicy;

typedef message_filters::Synchronizer<RGBDInfoSyncPolicy> RGBDSynchronizer;


class EOD_ROS{
public:
    EOD_ROS(ros::NodeHandle nh, ros::NodeHandle nh_p);
        
    
private:
    // ros stuff
    ros::NodeHandle nh_, nh_p_;
    image_transport::ImageTransport *rgb_it_;
    image_transport::SubscriberFilter sub_rgb_;    
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    boost::shared_ptr<RGBSynchronizer> rgb_sync_;
    
    image_transport::ImageTransport *depth_it_;
    image_transport::SubscriberFilter sub_depth_;    
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depth_info_;
    boost::shared_ptr<RGBDSynchronizer> rgbd_sync_;
    
    ros::Publisher simple_objects_pub_;
    
    // params
    bool subscribe_depth;
    double rate_limit_sec;
    bool publish_output;
    bool use_actual_time;
    
    // vars
    int frame_sequence;     
    ros::Time prev_detected_time;
    
    // callbacks
    void rgb_info_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info);
    
    void rgbd_info_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info, const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_info);
    
    // functions
    //void detect(const cv::Mat& rgb, const cv::Mat& depth, std_msgs::Header header);
    void detect(const eod::InfoImage& rgb, const eod::InfoImage& depth, std_msgs::Header header);
    bool check_time(ros::Time stamp);
    void add_data_to_simple_msg( eod::SimpleObject*, extended_object_detection::SimpleObjectArray& msg, const cv::Mat& K);
    extended_object_detection::BaseObject eoi_to_base_object( eod::SimpleObject* so, eod::ExtendedObjectInfo* eoi, const cv::Mat& K);
    cv::Mat getK(const sensor_msgs::CameraInfoConstPtr& info_msg);
    
    // EOD
    eod::ObjectBase * object_base;
    std::vector<eod::SimpleObject*> selected_simple_objects;
#ifdef USE_IGRAPH
    std::vector<eod::ComplexObjectGraph*> selected_complex_objects;
#endif
    
    
};
