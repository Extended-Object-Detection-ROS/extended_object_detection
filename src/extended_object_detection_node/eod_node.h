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
#include <visualization_msgs/Marker.h>

#include "extended_object_detection/BaseObject.h"
#include "extended_object_detection/SimpleObjectArray.h"
#include "extended_object_detection/ComplexObjectArray.h"
#include "extended_object_detection/SetObjects.h"

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
    ros::Publisher simple_objects_markers_pub_;
    image_transport::ImageTransport *private_it_;
    image_transport::Publisher output_image_pub_;
    
    ros::ServiceServer set_simple_objects_srv_;
#ifdef USE_IGRAPH
    ros::ServiceServer set_complex_objects_srv_;
#endif
    
    // params
    bool subscribe_depth;
    double rate_limit_sec;
    bool publish_image_output;
    bool use_actual_time;
    bool publish_markers;
    
    // vars
    int frame_sequence;     
    ros::Time prev_detected_time;
    
    // callbacks
    void rgb_info_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info);
    
    bool set_simple_objects_cb(extended_object_detection::SetObjects::Request &req, extended_object_detection::SetObjects::Response &res);
#ifdef USE_IGRAPH
    bool set_complex_objects_cb(extended_object_detection::SetObjects::Request &req, extended_object_detection::SetObjects::Response &res);
#endif
    
    void rgbd_info_cb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info, const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_info);
    
    // functions
    //void detect(const cv::Mat& rgb, const cv::Mat& depth, std_msgs::Header header);
    void detect(const eod::InfoImage& rgb, const eod::InfoImage& depth, std_msgs::Header header);
    bool check_time(ros::Time stamp);
    //void add_data_to_simple_msg( eod::SimpleObject*, extended_object_detection::SimpleObjectArray& msg, const cv::Mat& K);
    extended_object_detection::BaseObject eoi_to_base_object(std::string name, int id, eod::ExtendedObjectInfo* eoi, const cv::Mat& K);
    cv::Mat getK(const sensor_msgs::CameraInfoConstPtr& info_msg);
    cv::Mat getD(const sensor_msgs::CameraInfoConstPtr& info_msg);    
    
    visualization_msgs::Marker base_object_to_marker_arrow(extended_object_detection::BaseObject& base_object, const cv::Mat& K, std_msgs::Header header, cv::Scalar color, int id);
    visualization_msgs::Marker base_object_to_marker_frame(extended_object_detection::BaseObject& base_object, const cv::Mat& K, std_msgs::Header header, cv::Scalar color, int id);
    
    int find_simple_obj_index_by_id(int id);
#ifdef USE_IGRAPH
    int find_complex_obj_index_by_id(int id);
#endif
    
    // EOD
    eod::ObjectBase * object_base;
    std::vector<eod::SimpleObject*> selected_simple_objects;
#ifdef USE_IGRAPH
    std::vector<eod::ComplexObjectGraph*> selected_complex_objects;
#endif
    
    
};
