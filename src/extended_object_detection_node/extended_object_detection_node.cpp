#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "extended_object_detection/SimpleObject.h"
#include "extended_object_detection/SimpleObjectArray.h"
#include "extended_object_detection/ComplexObject.h"
#include "extended_object_detection/ComplexObjectArray.h"
#include "extended_object_detection/Track.h"
#include "extended_object_detection/ExtractedInfo.h"
#include "extended_object_detection/ImagePoint.h"
#include "extended_object_detection/SetSimpleObjects.h"

#include "ObjectBase.h"
#include <math.h>
#include <mutex>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include "geometry_utils.h"
#include <algorithm>

using namespace eod;
using namespace std;
using namespace cv;
using namespace message_filters;

// globals
ObjectBase * objectBase = NULL;
static const std::string OUTPUT_WINDOW = "Object and scene detection node output";
ros::Publisher objects_pub_;

image_transport::Publisher detected_image_pub_;
ros::Publisher marker_array_simple_pub_;


vector<SimpleObject*> selected_to_detect_simple_objects;
vector<int> selected_on_start_simple;
#ifdef USE_IGRAPH
vector<ComplexObjectGraph*> selected_to_detect_complex_objects;
vector<int> selected_on_start_complex;
ros::Publisher scenes_pub_;
ros::Publisher marker_array_complex_pub_;
#endif

std::mutex mutex_image;

//settings
bool screenOutputFlag = true;  
bool publishImage = true;
bool publishMarkers = false;
vector<string> visualizationTypes;
double updateRateMs = 0;
bool subscribeDepth = false;
double depth_scale = 0.001;
int maxContourPoints = 10;
bool rotate_image_180 = false;

// Intrinsic and other cam parameters
double fx, fy, cx, cy;
int seq = 0;
Mat last_image;
Mat last_depth;
string image_frame_id = "";
string depth_frame_id = "";

// additional functions
geometry_msgs::Vector3 getUnitTranslation(Point point){
    geometry_msgs::Vector3 unit_translate;
    unit_translate.x = (point.x - cx) / fx;
    unit_translate.y = (point.y - cy) / fy;
    unit_translate.z = 1;
    return unit_translate;
}

geometry_msgs::Vector3 multiplyVectorScalar(geometry_msgs::Vector3 vector, double scalar){
    geometry_msgs::Vector3 new_vector;
    new_vector.x = vector.x * scalar;
    new_vector.y = vector.y * scalar;
    new_vector.z = vector.z * scalar;
    return new_vector;
}
    
    
//------------------------------------------------------
// SERVICES
//------------------------------------------------------
int findIdSimpleObjects(int id){
    for( size_t i = 0 ; i < selected_to_detect_simple_objects.size() ; i++ ){
        if( id == selected_to_detect_simple_objects[i]->ID )
            return i;        
    }
    return -1;
}

// set SimpleObjects srv
bool setSimpleObjects(extended_object_detection::SetSimpleObjects::Request &req, extended_object_detection::SetSimpleObjects::Response &res){
    bool return_value = true;
    if( req.remove_all && req.add_all ){
        // do nothing and return false
        //return_value = false;
    }
    else{
        if( req.remove_all ){
            selected_to_detect_simple_objects.clear();
            return_value = true;
        }
        else if( req.add_all ){
            selected_to_detect_simple_objects.clear();
            for( size_t i = 0 ; i < objectBase->simple_objects.size() ; i++ )
                selected_to_detect_simple_objects.push_back(objectBase->simple_objects[i]);
            return_value = true;
        }
        else{
            for( size_t i = 0 ; i < req.changes.size() ; i++ ){
                SimpleObject* so = objectBase->getSimpleObjectByID(abs(req.changes[i]));
                if(so){
                    if( req.changes[i] > 0 ){
                        // add
                        if( findIdSimpleObjects(req.changes[i]) != -1){
                            // already in list
                            //return_value = false;
                            ROS_WARN("[set_simple_objects srv] object with id %i is already selected",abs(req.changes[i]));
                        }
                        else{
                            selected_to_detect_simple_objects.push_back(so);
                        }
                    }
                    else{
                        // remove
                        int found_no = findIdSimpleObjects(abs(req.changes[i]));
                        if( found_no != -1){
                            selected_to_detect_simple_objects.erase(selected_to_detect_simple_objects.begin()+found_no);
                        }
                        else{
                            // already emoved
                            //return_value = false;
                            ROS_WARN("[set_simple_objects srv] object with id %i is already unselected",abs(req.changes[i]));
                        }
                    }
                }
                else{
                    ROS_ERROR("[set_simple_objects srv] no object in base with id %i",abs(req.changes[i]));
                }
            }
        }
    }
    for( size_t i = 0 ; i < selected_to_detect_simple_objects.size() ; i++ ){
        res.result.push_back(selected_to_detect_simple_objects[i]->ID);
    }
    return return_value;
}
    
#ifdef USE_IGRAPH    
int findIdComplexObjects(int id){
    for( size_t i = 0 ; i < selected_to_detect_complex_objects.size() ; i++ ){
        if( id == selected_to_detect_complex_objects[i]->ID )
            return i;        
    }
    return -1;
}

// set ComplexObjects srv
bool setComplexObjects(extended_object_detection::SetSimpleObjects::Request &req, extended_object_detection::SetSimpleObjects::Response &res){
    bool return_value = true;
    if( req.remove_all && req.add_all ){
        // do nothing and return false
        //return_value = false;
    }
    else{
        if( req.remove_all ){
            selected_to_detect_complex_objects.clear();
            return_value = true;
        }
        else if( req.add_all ){
            selected_to_detect_complex_objects.clear();
            for( size_t i = 0 ; i < objectBase->complex_objects_graph.size() ; i++ )
                selected_to_detect_complex_objects.push_back(objectBase->complex_objects_graph[i]);
            return_value = true;
        }
        else{
            for( size_t i = 0 ; i < req.changes.size() ; i++ ){
                ComplexObjectGraph* co = objectBase->getComplexObjectGraphByID(abs(req.changes[i]));
                if(co){
                    if( req.changes[i] > 0 ){
                        // add
                        if( findIdSimpleObjects(req.changes[i]) != -1){
                            // already in list
                            //return_value = false;
                            ROS_WARN("[set_simple_objects srv] object with id %i is already selected",abs(req.changes[i]));
                        }
                        else{
                            selected_to_detect_complex_objects.push_back(co);
                        }
                    }
                    else{
                        // remove
                        int found_no = findIdComplexObjects(abs(req.changes[i]));
                        if( found_no != -1){
                            selected_to_detect_complex_objects.erase(selected_to_detect_complex_objects.begin()+found_no);
                        }
                        else{
                            // already emoved
                            //return_value = false;
                            ROS_WARN("[set_simple_objects srv] object with id %i is already unselected",abs(req.changes[i]));
                        }
                    }
                }
                else{
                    ROS_ERROR("[set_simple_objects srv] no object in base with id %i",abs(req.changes[i]));
                }
            }
        }
    }
    for( size_t i = 0 ; i < selected_to_detect_complex_objects.size() ; i++ ){
        res.result.push_back(selected_to_detect_complex_objects[i]->ID);
    }
    return return_value;
}
#endif
    
//------------------------------------------------------
// IMAGE AND DEPTH AND INFO CALLBACKS
//------------------------------------------------------    

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    mutex_image.lock();
    try
    {
        last_image = cv_bridge::toCvCopy(msg, "bgr8")->image;          
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    mutex_image.unlock();
}

void rgbdCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg){
    mutex_image.lock();
    try{
        last_image = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;      
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rgb_msg->encoding.c_str());        
    }
    try{
        last_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;      //TYPE_16UC1        
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s'.", depth_msg->encoding.c_str());        
    }
    mutex_image.unlock();
}

void cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
{
    if( image_frame_id == "" ){
        image_frame_id = msg.header.frame_id;
        fx = msg.K[0];
        fy = msg.K[4];
        cx = msg.K[2];
        cy = msg.K[5]; 
        
        // Camera intrinsics
        Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
        // distortion coefficients
        Mat distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);  
        
        for (size_t i=0; i<3; i++) {
            for (size_t j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) = msg.K[i*3+j];
            }
        }
        for (size_t i=0; i<5; i++) {
            distortionCoeffs.at<double>(0,i) = msg.D[i];
        }
        
        objectBase->setCameraParams(cameraMatrix, distortionCoeffs);
    }
}

void depthInfoCallback(const sensor_msgs::CameraInfo& msg)
{
    depth_frame_id = msg.header.frame_id;
}

//------------------------------------------------------
// SIMPLE OBJECT MESSAGE
//------------------------------------------------------

extended_object_detection::SimpleObject ros_msg_from_extended(ExtendedObjectInfo* ext_obj){
    extended_object_detection::SimpleObject current_object;
    
    current_object.rect.left_bottom.x = ext_obj->x;
    current_object.rect.left_bottom.y = ext_obj->y;
    current_object.rect.right_up.x = ext_obj->x + ext_obj->width;
    current_object.rect.right_up.y = ext_obj->y + ext_obj->height;                         
    
    current_object.score = ext_obj->total_score;    
    current_object.track.id = ext_obj->track_id;

    
//     for( size_t i = 0 ; i < ext_obj->sub_id.size(); i++ ){
//         extended_object_detection::ExtractedInfo ei;
//         ei.sub_id = ext_obj->sub_id[i];
//         ei.text = ext_obj->extracted_info[i];
//         current_object.extracted_info.push_back(ei);
//     }
    for( auto const& exi : ext_obj->extracted_info){
        current_object.extracted_info.keys.push_back(exi.first);
        current_object.extracted_info.values.push_back(exi.second);        
    }
    
#if (CV_MAJOR_VERSION > 3)
    if( ext_obj->track_status == DETECTED )
        current_object.track.status = extended_object_detection::Track::DETECTED;
    else if(ext_obj->track_status == TRACKED )
        current_object.track.status = extended_object_detection::Track::TRACKED;
    else if(ext_obj->track_status == LOST )
        current_object.track.status = extended_object_detection::Track::LOST;
#else
    current_object.track.status = extended_object_detection::Track::DETECTED;
#endif        
    // if we have solved translation then use it
    if( ext_obj->tvec.size() > 0 ){                            
        // TODO: invite some algorithm for merging tvecs
        current_object.transform.translation.x = ((ext_obj->tvec[0]))[0];
        current_object.transform.translation.y = ((ext_obj->tvec[0]))[1];
        current_object.transform.translation.z = ((ext_obj->tvec[0]))[2];
    }
    // else calculate unit translation
    else
        current_object.transform.translation = getUnitTranslation(ext_obj->getCenter());                        
    
    // if we have solved rotation then use it
    if (ext_obj->rvec.size() > 0 ){
        double quaternion[4];
        Mat rotMat;
        Rodrigues(ext_obj->rvec, rotMat);
        getQuaternion( rotMat, quaternion);
        current_object.transform.rotation.x = quaternion[0];
        current_object.transform.rotation.y = quaternion[1];
        current_object.transform.rotation.z = quaternion[2];
        current_object.transform.rotation.w = quaternion[3];
    }
    // else use zero quaternion
    else
        current_object.transform.rotation.w = 1;
    
    geometry_msgs::Transform temp;
    // rotation to zero quaternion
    temp.rotation.w = 1;
    
    temp.translation = getUnitTranslation(Point(ext_obj->x, ext_obj->y));   
    if( ext_obj->tvec.size() > 0 )                            
        temp.translation = multiplyVectorScalar(temp.translation, ((ext_obj->tvec[0]))[2]);     
    current_object.rect.cornerTranslates.push_back(temp.translation);                                                
    
    temp.translation = getUnitTranslation(Point(ext_obj->x, ext_obj->y + ext_obj->height));
    if( ext_obj->tvec.size() > 0 )                            
        temp.translation = multiplyVectorScalar(temp.translation, ((ext_obj->tvec[0]))[2]);     
    current_object.rect.cornerTranslates.push_back(temp.translation);            
    
    temp.translation = getUnitTranslation(Point(ext_obj->x + ext_obj->width, ext_obj->y + ext_obj->height));
    if( ext_obj->tvec.size() > 0 )                            
        temp.translation = multiplyVectorScalar(temp.translation, ((ext_obj->tvec[0]))[2]);     
    current_object.rect.cornerTranslates.push_back(temp.translation);
    
    temp.translation = getUnitTranslation(Point(ext_obj->x + ext_obj->width, ext_obj->y));
    if( ext_obj->tvec.size() > 0 )                            
        temp.translation = multiplyVectorScalar(temp.translation, ((ext_obj->tvec[0]))[2]);     
    current_object.rect.cornerTranslates.push_back(temp.translation);
    
    if( ext_obj->contour.size() > 0 ){
        if( (maxContourPoints != -1 && ext_obj->contour[0].size() < maxContourPoints) || maxContourPoints == -1){                            
            for( size_t i = 0 ; i < ext_obj->contour[0].size() ; i++){   
                if( ext_obj->tvec.size() > 0 )
                    current_object.contour.contourTranslates.push_back(multiplyVectorScalar(getUnitTranslation(ext_obj->contour[0][i]),ext_obj->tvec[0][2]));
                else
                    current_object.contour.contourTranslates.push_back(getUnitTranslation(ext_obj->contour[0][i]));
            }            
        }
    }
    
    // track
//     for( size_t i = 0 ; i < ext_obj->track_history_image->size() ; i++ ){
//         printf("%i\n",i);
//         extended_object_detection::ImagePoint ip;
//         ip.x = ext_obj->track_history_image->at(i).x;
//         ip.y = ext_obj->track_history_image->at(i).y;
//         current_object.track.image_point_history.push_back(ip);
//     }
    
    return current_object;
}

//------------------------------------------------------
// COMPLEX OBJECT MESSAGE
//------------------------------------------------------
extended_object_detection::ComplexObject ros_msg_from_complex(ExtendedObjectInfo* complex, vector<ExtendedObjectInfo> inner_simples){
    
    extended_object_detection::ComplexObject current_object;
    current_object.score = 1; // TODO fix when total score will be available
    
    current_object.rect.left_bottom.x = complex->x;
    current_object.rect.left_bottom.y = complex->y;
    current_object.rect.right_up.x = complex->x + complex->width;
    current_object.rect.right_up.y = complex->y + complex->height;      
    
    // if we have solved translation then use it
    if( complex->tvec.size() > 0 ){                            
        // TODO: invite some algorithm for merging tvecs
        current_object.transform.translation.x = ((complex->tvec[0]))[0];
        current_object.transform.translation.y = ((complex->tvec[0]))[1];
        current_object.transform.translation.z = ((complex->tvec[0]))[2];
    }
    // else calculate unit translation
    else
        current_object.transform.translation = getUnitTranslation(complex->getCenter());                        
    
//     if (complex->rvec.size() > 0 ){
//         double quaternion[4];
//         Mat rotMat;
//         Rodrigues(complex->rvec, rotMat);
//         getQuaternion( rotMat, quaternion);
//         current_object.transform.rotation.x = quaternion[0];
//         current_object.transform.rotation.y = quaternion[1];
//         current_object.transform.rotation.z = quaternion[2];
//         current_object.transform.rotation.w = quaternion[3];
//     }
    current_object.transform.rotation.w = 1;
    
    if( inner_simples.size() > 0 ){
        for( size_t i = 0; i < inner_simples.size(); i++ ){
            current_object.objects.push_back(ros_msg_from_extended(&(inner_simples.at(i))));
        }
    }
    geometry_msgs::Transform temp;
    // rotation to zero quaternion
    temp.rotation.w = 1;
    
    temp.translation = getUnitTranslation(Point(complex->x, complex->y));   
    if( complex->tvec.size() > 0 )                            
        temp.translation = multiplyVectorScalar(temp.translation, ((complex->tvec[0]))[2]);     
    current_object.rect.cornerTranslates.push_back(temp.translation);                                                
    
    temp.translation = getUnitTranslation(Point(complex->x, complex->y + complex->height));
    if( complex->tvec.size() > 0 )                            
        temp.translation = multiplyVectorScalar(temp.translation, ((complex->tvec[0]))[2]);     
    current_object.rect.cornerTranslates.push_back(temp.translation);            
    
    temp.translation = getUnitTranslation(Point(complex->x + complex->width, complex->y + complex->height));
    if( complex->tvec.size() > 0 )                            
        temp.translation = multiplyVectorScalar(temp.translation, ((complex->tvec[0]))[2]);     
    current_object.rect.cornerTranslates.push_back(temp.translation);
    
    temp.translation = getUnitTranslation(Point(complex->x + complex->width, complex->y));
    if( complex->tvec.size() > 0 )                            
        temp.translation = multiplyVectorScalar(temp.translation, ((complex->tvec[0]))[2]);     
    current_object.rect.cornerTranslates.push_back(temp.translation);
    
    return current_object;    
}


//------------------------------------------------------
// SIMPLE OBJECT MARKERS
//------------------------------------------------------

visualization_msgs::MarkerArray marker_array_simple(vector<extended_object_detection::SimpleObject> objects){
    visualization_msgs::MarkerArray marker_array;    
    
    for( size_t j = 0 ; j < objects.size() ; j++ ){
        for( size_t i = 0 ; i < visualizationTypes.size() ; i++ ){
            // common settings
            visualization_msgs::Marker marker; 
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = image_frame_id;  
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(updateRateMs);            
            marker.id = j;
            marker.pose.orientation.w = 1;            
            marker.color.a = 1;
            if( objects[j].track.status == extended_object_detection::Track::DETECTED ){
                marker.color.g = 1;
            }
            else if( objects[j].track.status == extended_object_detection::Track::TRACKED ){
                marker.color.r = 1;
                marker.color.g = 1;
            }
            else if( objects[j].track.status == extended_object_detection::Track::LOST ){
                marker.color.r = 1;
            }                                           
            
            // arrow links from camera tf frame to center of object
            if( visualizationTypes[i] == "arrows" ){   
                marker.ns = "arrows";            
                
                marker.type = visualization_msgs::Marker::ARROW;                
                
                // append start point in 0 of frame 
                marker.points.push_back(geometry_msgs::Point());
                // append end point
                geometry_msgs::Point end;
                end.x = objects[j].transform.translation.x;
                end.y = objects[j].transform.translation.y;
                end.z = objects[j].transform.translation.z;
                marker.points.push_back(end);
                    
                // sizes
                marker.scale.x = 0.01;
                marker.scale.y = 0.02;
                marker.scale.z = 0.1;                                
                                
            }
            // bounding rects of objects
            else if( visualizationTypes[i] == "rects" ){   
                marker.ns = "rects";                
                
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                
                marker.scale.x = 0.02;
                
                for( size_t k = 0 ; k < objects[j].rect.cornerTranslates.size() ; k++){
                    geometry_msgs::Point pt;
                    pt.x = objects[j].rect.cornerTranslates[k].x;
                    pt.y = objects[j].rect.cornerTranslates[k].y;
                    pt.z = objects[j].rect.cornerTranslates[k].z;
                    marker.points.push_back(pt);
                }
                // link it
                marker.points.push_back(marker.points[0]);
                
            }
            // axis of object
            else if( visualizationTypes[i] == "axis" ){
                marker.ns = "axis";
                
                marker.type = visualization_msgs::Marker::LINE_LIST;
                
                // axis 
                float axis_len = 0.1;
                // OX
                marker.points.push_back(geometry_msgs::Point());
                geometry_msgs::Point ox;
                ox.x = axis_len;
                marker.points.push_back(ox);
                std_msgs::ColorRGBA r;
                r.r = 1;
                r.a = 1;
                marker.colors.push_back(r);
                marker.colors.push_back(r);
                // OY
                marker.points.push_back(geometry_msgs::Point());
                geometry_msgs::Point oy;
                oy.y = axis_len;
                marker.points.push_back(oy);
                std_msgs::ColorRGBA g;
                g.g = 1;
                g.a = 1;
                marker.colors.push_back(g);
                marker.colors.push_back(g);
                // OZ
                marker.points.push_back(geometry_msgs::Point());
                geometry_msgs::Point oz;
                oz.z = axis_len;
                marker.points.push_back(oz);
                std_msgs::ColorRGBA b;
                b.b = 1;
                b.a = 1;
                marker.colors.push_back(b);
                marker.colors.push_back(b);
                
                marker.scale.x = 0.01;
                
                // axis pose
                marker.pose.position.x = objects[j].transform.translation.x;
                marker.pose.position.y = objects[j].transform.translation.y;
                marker.pose.position.z = objects[j].transform.translation.z;
                
                marker.pose.orientation.x = objects[j].transform.rotation.x;
                marker.pose.orientation.y = objects[j].transform.rotation.y;
                marker.pose.orientation.z = objects[j].transform.rotation.z;
                marker.pose.orientation.w = objects[j].transform.rotation.w;
                
            }
            // main text                        
            else if( visualizationTypes[i] == "main_text" ){
                marker.ns = "main_text";
                
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                
                marker.pose.position.x = objects[j].transform.translation.x;
                // want to place it exacly on top of rect
                marker.pose.position.y = objects[j].rect.cornerTranslates[0].y - 0.14;
                marker.pose.position.z = objects[j].transform.translation.z;
                
                marker.scale.z = 0.1;
                
                marker.text = to_string(objects[j].type_id)+":"+ objects[j].type_name + "["+to_string( objects[j].score).substr(0,4)+"]";
                
            }
            // extracted_info
            /*else if( visualizationTypes[i] == "extracted_info" ){
                if( objects[j].extracted_info.size() > 0){
                    marker.ns = "extracted_info";
                    
                    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    
                    marker.pose.position.x = objects[j].transform.translation.x;
                    // want to place it exacly on top of rect, but lower then main text
                    marker.pose.position.y = objects[j].rect.cornerTranslates[0].y - 0.07;
                    marker.pose.position.z = objects[j].transform.translation.z;
                    
                    marker.scale.z = 0.1;
                    
                    for( size_t k = 0 ; k < objects[j].extracted_info.size() ; k++){
                        if( objects[j].extracted_info[k].text != "")
                            marker.text += ( objects[j].extracted_info[k].text + " ");                        
                    }
                    if( marker.text == "" )
                        continue;
                }
                else
                    continue;
                
            }   */    
            // contour
            else if( visualizationTypes[i] == "contour" ){
                if( objects[j].contour.contourTranslates.size() > 0){
                    marker.ns = "contour";
                    marker.scale.x = 0.01;
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    
                    for(size_t k = 0 ; k < objects[j].contour.contourTranslates.size() ; k++){
                        geometry_msgs::Point pt;
                        pt.x = objects[j].contour.contourTranslates[k].x;
                        pt.y = objects[j].contour.contourTranslates[k].y;
                        pt.z = objects[j].contour.contourTranslates[k].z;
                        marker.points.push_back(pt);
                    }
                    marker.points.push_back(marker.points[0]);                    
                }              
                else
                    continue;
            }
            // track TODO
//             else if( visualizationTypes[i] == "track" ){                
//                 
//                 
//             }
            else{
                // not  publish unknown types
                continue;
            }     
            marker_array.markers.push_back(marker);  
        }
    }    
    return marker_array;
}

visualization_msgs::MarkerArray marker_array_complex(extended_object_detection::ComplexObjectArray array){
    visualization_msgs::MarkerArray marker_array;    
    
    for( size_t j = 0 ; j < array.complex_objects.size() ; j++ ){
        for( size_t i = 0 ; i < visualizationTypes.size() ; i++ ){            
            visualization_msgs::Marker marker; 
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = image_frame_id;  
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(updateRateMs);            
            marker.id = j;
            marker.pose.orientation.w = 1;            
            marker.color.a = 1;
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 1;
            //outer rect
            if( visualizationTypes[i] == "rects" ){   
                marker.ns = "complex_rect";                
                marker.type = visualization_msgs::Marker::LINE_STRIP;                
                marker.scale.x = 0.03;
                
                for( size_t k = 0 ; k < array.complex_objects[j].rect.cornerTranslates.size() ; k++){
                    geometry_msgs::Point pt;
                    pt.x = array.complex_objects[j].rect.cornerTranslates[k].x;
                    pt.y = array.complex_objects[j].rect.cornerTranslates[k].y;
                    pt.z = array.complex_objects[j].rect.cornerTranslates[k].z;
                    marker.points.push_back(pt);
                }
                // link it
                marker.points.push_back(marker.points[0]);
            }
            else if( visualizationTypes[i] == "main_text" ){
                marker.ns = "complex_main_text";
                
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                
                marker.pose.position.x = array.complex_objects[j].transform.translation.x;
                // want to place it exacly on top of rect
                marker.pose.position.y = array.complex_objects[j].rect.cornerTranslates[0].y - 0.2;
                marker.pose.position.z = array.complex_objects[j].transform.translation.z;
                
                marker.scale.z = 0.12;
                
                marker.text = to_string(array.complex_objects[j].type_id)+":"+ array.complex_objects[j].type_name + "["+to_string( array.complex_objects[j].score).substr(0,4)+"]";
                
            }
            else
                continue;
            marker_array.markers.push_back(marker);  
        }
        
        visualization_msgs::MarkerArray simples = marker_array_simple(array.complex_objects[j].objects);
        for( size_t i = 0 ;i < simples.markers.size(); i++){
            simples.markers[i].color.r = 0;
            simples.markers[i].color.g = 1;
            simples.markers[i].color.b = 1;
        }
        marker_array.markers.insert(marker_array.markers.end(), simples.markers.begin(), simples.markers.end());
        
    }
    return marker_array;
}
//------------------------------------------------------
// MAIN CALLBACK, IMAGE PROCESSING, DETECTION ITSELF
//------------------------------------------------------

void video_process_cb(const ros::TimerEvent&){    
            
    mutex_image.lock();        
    
    if(!last_image.empty()){              
        if(image_frame_id == ""){
            ROS_ERROR("No camera intrinsic are provided, or camera frame_id is not set.");
            mutex_image.unlock();
            return;
        }
        
        if( rotate_image_180 ){
            rotate(last_image, last_image, 1);
        }
        
        Mat image2draw = last_image.clone();
        extended_object_detection::SimpleObjectArray array_objects;
                
        ros::Time begin_detection = ros::Time::now();
        
        // SIMPLE OBJECTS
        for( size_t i = 0 ; i < selected_to_detect_simple_objects.size(); i++){                                
            selected_to_detect_simple_objects[i]->Identify(last_image, last_depth, seq);            
            
            // drawing
            if(screenOutputFlag || publishImage)
                selected_to_detect_simple_objects[i]->draw(image2draw,Scalar(0,255,0));
            
            // collect data                                  
            for(size_t j = 0 ; j < selected_to_detect_simple_objects[i]->objects.size() ; j++ ){                
                ExtendedObjectInfo *ext_obj = &(selected_to_detect_simple_objects[i]->objects[j]);
                
                extended_object_detection::SimpleObject current_object = ros_msg_from_extended(ext_obj);  
                
                current_object.type_id = selected_to_detect_simple_objects[i]->ID;
                current_object.type_name = selected_to_detect_simple_objects[i]->name;                             
                array_objects.objects.push_back(current_object);
            }                                
        }        
#ifdef USE_IGRAPH
        // COMPLEX OBJECTS
        extended_object_detection::ComplexObjectArray array_co_msg;        
        for( size_t i = 0 ; i < selected_to_detect_complex_objects.size(); i++){   
            /*
            vector<ExtendedObjectInfo> DetectedComplexObjects;
            vector<vector<ExtendedObjectInfo> > DetectedObjects;
            
            bool full_answer;
            DetectedComplexObjects = selected_to_detect_complex_objects.at(i)->Identify(last_image, last_depth, seq, full_answer);                
            
            if( full_answer)
                DetectedObjects = selected_to_detect_complex_objects.at(i)->getFullAnswer();            
            
            if( screenOutputFlag || publishImage)
                selected_to_detect_complex_objects.at(i)->drawAll(image2draw, Scalar(255, 255, 0), 2);
                          
            for(size_t j = 0; j < DetectedComplexObjects.size(); j++){
                extended_object_detection::ComplexObject co_msg;
                                
                co_msg = ros_msg_from_complex(&DetectedComplexObjects[j], DetectedObjects[j]);                      
                                
                co_msg.type_id = selected_to_detect_complex_objects[i]->ID;
                co_msg.type_name = selected_to_detect_complex_objects[i]->name;
                    
                array_co_msg.complex_objects.push_back(co_msg);
            } 
            */
            vector<ExtendedObjectInfo> DetectedComplexObjects;
            
            DetectedComplexObjects = selected_to_detect_complex_objects.at(i)->Identify(last_image, last_depth, seq);     
            vector<ExtendedObjectInfo> DetectedObjects;
            
            for(size_t j = 0; j < DetectedComplexObjects.size(); j++){
                extended_object_detection::ComplexObject co_msg;
                                
                co_msg = ros_msg_from_complex(&DetectedComplexObjects[j], DetectedObjects);                      
                                
                co_msg.type_id = selected_to_detect_complex_objects[i]->ID;
                co_msg.type_name = selected_to_detect_complex_objects[i]->name;
                    
                array_co_msg.complex_objects.push_back(co_msg);
            } 
            
            if( screenOutputFlag || publishImage)
                selected_to_detect_complex_objects.at(i)->drawAll(image2draw, Scalar(255, 255, 0), 2);
            
            
        }
#endif
        seq++;
        ros::Time end_detection = ros::Time::now();
        double detection_time = (end_detection - begin_detection).toSec();
        if( detection_time > updateRateMs ){
            ROS_WARN("Missed desired detection rate of %f, actually loop takes %f.",updateRateMs, detection_time);
        }                  
        else{
            //ROS_INFO("Detection loop takes %f.", detection_time);
        }
        if( array_objects.objects.size() > 0){            
            array_objects.header.stamp = ros::Time::now();
            array_objects.header.frame_id = image_frame_id;            
            objects_pub_.publish(array_objects);      
            if( publishMarkers ){                
                marker_array_simple_pub_.publish(marker_array_simple(array_objects.objects));                
            }
            array_objects.objects.clear();
        }
#ifdef USE_IGRAPH        
        if( array_co_msg.complex_objects.size() > 0){
            array_co_msg.header.stamp = ros::Time::now();
            array_co_msg.header.frame_id = image_frame_id;
            scenes_pub_.publish(array_co_msg);                  
            if( publishMarkers ){
                marker_array_complex_pub_.publish(marker_array_complex(array_co_msg));
            }                
        }
#endif
        if( screenOutputFlag ){
            imshow(OUTPUT_WINDOW, image2draw);
#if (CV_MAJOR_VERSION > 3)
            waitKey(1);
#else
            cvWaitKey(1);
#endif
        }
        if( publishImage ){
            sensor_msgs::ImagePtr detected_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2draw).toImageMsg();
            detected_image_pub_.publish(detected_msg);
        }        
    }        
    mutex_image.unlock();
}

//-------------------------
// INIT
//-------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extended_object_detection_node");      
  ROS_INFO("Extended object detector started...\n");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p("~");
  
  string objectBasePath;  

  if( !nh_p.getParam("objectBasePath",objectBasePath) ){    
    ROS_ERROR("Object Base path didnot specified! Exit.");
    return -1;
  }
  ROS_INFO("ObjectBasePath: %s",objectBasePath.c_str());
    
  if( !nh_p.getParam("screenOutput",screenOutputFlag) ){
    screenOutputFlag = true;
    ROS_WARN("Screen output is turn on!");
  }
  
  if( !nh_p.getParam("publishImage",publishImage) ){
    publishImage = true;    
  }  
  nh_p.getParam("maxContourPoints", maxContourPoints);
  
  double videoProcessUpdateRate;
  if( !nh_p.getParam("videoProcessUpdateRate",videoProcessUpdateRate) ){
    videoProcessUpdateRate = 10;
    ROS_WARN("Default update rate for video is set!");
  }  
  ROS_INFO("Video update rate is %f",videoProcessUpdateRate);
  
  if( !nh_p.getParam("subscribeDepth",subscribeDepth) ){
      subscribeDepth = false;  
  }
  if (subscribeDepth ){
      if( !nh_p.getParam("depthScale", depth_scale))
          depth_scale = 0.001;
  }
  
  if( !nh_p.getParam("publishMarkers",publishMarkers) ){
      publishMarkers = false;  
  }
  
  nh_p.getParam("rotate_image_180", rotate_image_180);
  
  if( publishMarkers ){
      if( !nh_p.getParam("visualizationTypes",visualizationTypes) ){
          visualizationTypes.push_back("arrows");
          visualizationTypes.push_back("rects");
          visualizationTypes.push_back("axis");                    
          visualizationTypes.push_back("main_text");
          visualizationTypes.push_back("extracted_info");
          visualizationTypes.push_back("contour");
          visualizationTypes.push_back("track");
      }
  }
  
  nh_p.getParam("selectedOnStartSimple",selected_on_start_simple);
  
#ifdef USE_IGRAPH
  nh_p.getParam("selectedOnStartComplex",selected_on_start_complex);
#endif
  
  objectBase = new ObjectBase();
  if( !objectBase->loadFromXML(objectBasePath) ){
      ROS_ERROR("Error during loading object base in path '%s'! Please make yourself sure path to file and its content are correct.",objectBasePath.c_str());
      return -1;    
  }
  ROS_INFO("Object base counted %d simple objects",objectBase->simple_objects.size());
    
  // services stuff
  if( selected_on_start_simple.size() == 0 )
    for( size_t i = 0 ; i < objectBase->simple_objects.size() ; i++ ){
        selected_to_detect_simple_objects.push_back(objectBase->simple_objects[i]);
    }
  else{
    if(selected_on_start_simple[0] != -1){
        for( size_t i = 0 ; i < objectBase->simple_objects.size() ; i++ ){
            if( find(selected_on_start_simple.begin(), selected_on_start_simple.end(), objectBase->simple_objects[i]->ID) != selected_on_start_simple.end() )
                selected_to_detect_simple_objects.push_back(objectBase->simple_objects[i]);
        }        
    }
  }
  ros::ServiceServer setSimpleObjectsSrv = nh_p.advertiseService("set_simple_objects", setSimpleObjects);

#ifdef USE_IGRAPH
  if( selected_on_start_complex.size() == 0 )
    for( size_t i = 0 ; i < objectBase->complex_objects_graph.size() ; i++ ){
        selected_to_detect_complex_objects.push_back(objectBase->complex_objects_graph[i]);
    }
  else{
      if( selected_on_start_complex[0] != -1 ){
        for( size_t i = 0 ; i < objectBase->complex_objects_graph.size() ; i++ ){
            if( find(selected_on_start_complex.begin(), selected_on_start_complex.end(), objectBase->complex_objects_graph[i]->ID) != selected_on_start_complex.end() )
                selected_to_detect_complex_objects.push_back(objectBase->complex_objects_graph[i]);
        }
    }        
  }
  ros::ServiceServer setComplexObjectsSrv = nh_p.advertiseService("set_complex_objects", setComplexObjects);
#endif    
  ROS_INFO("Starting process...");
    
  objects_pub_ = nh_p.advertise<extended_object_detection::SimpleObjectArray>("simple_objects",1);
#ifdef USE_IGRAPH
  scenes_pub_ = nh_p.advertise<extended_object_detection::ComplexObjectArray>("complex_objects",1); 
#endif
  // mono
  image_transport::ImageTransport it(nh_);
  image_transport::ImageTransport it_p(nh_p);
  image_transport::Subscriber image_sub;
  
  if( publishImage ){
      detected_image_pub_ = it_p.advertise("detected_image",1);
  }
  if( publishMarkers ){
      marker_array_simple_pub_ = nh_p.advertise<visualization_msgs::MarkerArray>("simple_objects_markers",1);
#ifdef USE_IGRAPH
      marker_array_complex_pub_ = nh_p.advertise<visualization_msgs::MarkerArray>("complex_objects_markers",1);
#endif
  }
  
  updateRateMs =1/videoProcessUpdateRate;
    
  if( !subscribeDepth ){      
      image_sub = it.subscribe("camera/image_raw", 1, imageCallback);      
      ros::Subscriber camera_info_sub = nh_.subscribe("camera/info", 1, cameraInfoCallback);
  
      ros::Timer timer = nh_.createTimer(ros::Duration( updateRateMs ), video_process_cb);
      ros::spin();
  }  
  else{
      // rgbd
      message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub(nh_,"camera/image_raw",3);
      message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh_,"depth/image_raw",3);            
      typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;      
      Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_image_sub, depth_sub);
      
      ros::Subscriber depth_info_sub;           
      sync.registerCallback(boost::bind(&rgbdCallback, _1, _2));      
      depth_info_sub = nh_.subscribe("depth/info",1 , depthInfoCallback);
      
      ros::Subscriber camera_info_sub = nh_.subscribe("camera/info", 1, cameraInfoCallback);
  
      ros::Timer timer = nh_.createTimer(ros::Duration( updateRateMs ), video_process_cb);
        
      ROS_INFO("RGBD has set up.");
      ros::spin();
  }  
}
