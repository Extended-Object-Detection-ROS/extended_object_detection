cmake_minimum_required(VERSION 2.8.12)
project(extended_object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Optional parts of library
set(opencv_contrib OFF)
set(zbar_lib OFF)
set(TensorFlow OFF)
set(DLib OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  image_transport
  cv_bridge
  cmake_modules
)

find_package(OpenCV REQUIRED)  
find_package(TinyXML REQUIRED)
message("OpenCV version is ${OpenCV_VERSION}")
if(zbar_lib)
    find_package(PkgConfig REQUIRED)
    pkg_search_module(ZBAR REQUIRED zbar)
endif(zbar_lib)

if(DLib)
    add_subdirectory(/home/anton/Lib/dlib dlib_build)
endif(DLib)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ImagePoint.msg
  Rect.msg
  Track.msg
  ExtractedInfo.msg
  Contour.msg
  SimpleObject.msg
  SimpleObjectArray.msg
  ComplexObject.msg
  ComplexObjectArray.msg  
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetSimpleObjects.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS cv_bridge std_msgs sensor_msgs image_transport roscpp image_geometry cmake_modules message_runtime
   DEPENDS TinyXML
)

###########
## Build ##
###########
# main node
add_executable(
    extended_object_detection_node
    src/extended_object_detection_node/extended_object_detection_node.cpp
    # lib common
    src/lib/ObjectBase.cpp
    # types
    src/lib/types/ComplexObject.cpp
    src/lib/types/ExtendedObjectInfo.cpp
    src/lib/types/Attribute.cpp
    src/lib/types/SimpleObject.cpp
    src/lib/types/Tracker.cpp
    src/lib/types/Relationship.cpp    
    src/lib/types/Filtering.cpp    
    src/lib/types/Clusterization.cpp    
    # lib utils
    src/lib/utils/geometry_utils.cpp
    src/lib/utils/drawing_utils.cpp
    src/lib/utils/contour_utils.cpp
    # lib detectors
    src/lib/detectors/HsvColorDetector.cpp
    src/lib/detectors/HaarCascadeDetector.cpp
    src/lib/detectors/SizeDetector.cpp
    src/lib/detectors/HistColorDetector.cpp
    src/lib/detectors/DimentionDetector.cpp
    src/lib/detectors/HoughDetector.cpp    
    src/lib/detectors/BasicMotionDetector.cpp    
    src/lib/detectors/ArucoDetector.cpp
    src/lib/detectors/PoseDetector.cpp
    src/lib/detectors/TensorFlowDetector.cpp
    src/lib/detectors/FeatureDetector.cpp
    src/lib/detectors/DnnDetector.cpp
    src/lib/detectors/QrDetector.cpp
    src/lib/detectors/QrZbarDetector.cpp
    src/lib/detectors/LogicDetector.cpp
    src/lib/detectors/BlobDetector.cpp
    src/lib/detectors/DepthDetector.cpp
    src/lib/detectors/RoughDistanceDetector.cpp
    src/lib/detectors/DistanceDetector.cpp 
    src/lib/detectors/FaceDlibDetector.cpp 

    # lib relations        
    src/lib/relations/ImageRangeRelation.cpp
    src/lib/relations/LogicRelations.cpp
    src/lib/relations/ThreeDimRangeRelation.cpp
    src/lib/relations/SpaceRelations.cpp
)

if(TensorFlow)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_TF) 
endif(TensorFlow)

if(opencv_contrib)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_OPENCV_CONTRIB)         
endif(opencv_contrib)

if(zbar_lib)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_ZBAR)
endif(zbar_lib)

if(DLib)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_DLIB)    
endif(DLib)

#collect hsv color c++ utility
add_executable(
    hsv_color_params_collector_node
    src/utils/hsv_color_params_collector.cpp
    # types
    src/lib/types/ExtendedObjectInfo.cpp
    src/lib/types/Attribute.cpp
    src/lib/types/SimpleObject.cpp
    src/lib/types/Filtering.cpp
    src/lib/types/Clusterization.cpp    
    # utils    
    src/lib/utils/geometry_utils.cpp
    src/lib/utils/drawing_utils.cpp
    src/lib/utils/contour_utils.cpp
    # detectors
    src/lib/detectors/HsvColorDetector.cpp
    src/lib/detectors/SizeDetector.cpp
)
    
#collect histogram color c++ utility with point
add_executable(
    hist_color_params_collector_point_node
    src/utils/hist_color_params_collector_point.cpp
    # types
    src/lib/types/ExtendedObjectInfo.cpp
    src/lib/types/Attribute.cpp
    src/lib/types/SimpleObject.cpp
    src/lib/types/Filtering.cpp
    src/lib/types/Clusterization.cpp    
    # utils
    src/lib/utils/geometry_utils.cpp
    src/lib/utils/drawing_utils.cpp
    src/lib/utils/contour_utils.cpp
    # detectors
    src/lib/detectors/HistColorDetector.cpp
    src/lib/detectors/SizeDetector.cpp
)

#collect histogram color c++ utility with contour
add_executable(
    hist_color_params_collector_contour_node
    src/utils/hist_color_params_collector_contour.cpp
    # types
    src/lib/types/ExtendedObjectInfo.cpp
    src/lib/types/Attribute.cpp
    src/lib/types/SimpleObject.cpp
    src/lib/types/Filtering.cpp
    src/lib/types/Clusterization.cpp    
    # utils   
    src/lib/utils/geometry_utils.cpp
    src/lib/utils/drawing_utils.cpp
    src/lib/utils/contour_utils.cpp
    # detectors
    src/lib/detectors/HistColorDetector.cpp
    src/lib/detectors/SizeDetector.cpp
)
    
#collect hough circle c++ utility
add_executable(
    hough_circle_params_collector_node
    src/utils/hough_circle_params_collector.cpp
    # types
    src/lib/types/ExtendedObjectInfo.cpp
    src/lib/types/Attribute.cpp
    src/lib/types/SimpleObject.cpp
    src/lib/types/Filtering.cpp
    src/lib/types/Clusterization.cpp    
    # utils   
    src/lib/utils/geometry_utils.cpp
    src/lib/utils/drawing_utils.cpp
    src/lib/utils/contour_utils.cpp
    # detectors
    src/lib/detectors/HoughDetector.cpp
    src/lib/detectors/SizeDetector.cpp
)

#collect blob params c++ utility
add_executable(
    blob_params_collector_node
    src/utils/blob_params_collector.cpp
    # types
    src/lib/types/ExtendedObjectInfo.cpp
    src/lib/types/Attribute.cpp
    src/lib/types/SimpleObject.cpp
    src/lib/types/Filtering.cpp
    src/lib/types/Clusterization.cpp    
    # utils    
    src/lib/utils/geometry_utils.cpp
    src/lib/utils/drawing_utils.cpp
    src/lib/utils/contour_utils.cpp
    # detectors
    src/lib/detectors/BlobDetector.cpp
    #src/lib/detectors/SizeDetector.cpp
)
    
# dlib face descriptors saver
if(DLib)
    add_executable(
        face_dlib_descriptors_extractor
        src/utils/face_dlib_descriptors_extractor.cpp
        # types
        src/lib/types/ExtendedObjectInfo.cpp
        src/lib/types/Attribute.cpp
        src/lib/types/SimpleObject.cpp
        src/lib/types/Filtering.cpp
        src/lib/types/Clusterization.cpp    
        # utils    
        src/lib/utils/geometry_utils.cpp
        src/lib/utils/drawing_utils.cpp
        src/lib/utils/contour_utils.cpp
        # detectors
        src/lib/detectors/FaceDlibDetector.cpp
    )
    
    target_compile_definitions(face_dlib_descriptors_extractor PUBLIC USE_DLIB)
endif(DLib)
    
#offline video processor
add_executable(
    offline_video
    src/offline_video/offline_video.cpp
    # lib common
    src/lib/ObjectBase.cpp
    # types
    src/lib/types/ComplexObject.cpp
    src/lib/types/ExtendedObjectInfo.cpp
    src/lib/types/Attribute.cpp
    src/lib/types/SimpleObject.cpp
    src/lib/types/Tracker.cpp
    src/lib/types/Relationship.cpp    
    src/lib/types/Filtering.cpp    
    src/lib/types/Clusterization.cpp    
    # lib utils    
    src/lib/utils/geometry_utils.cpp
    src/lib/utils/drawing_utils.cpp
    src/lib/utils/contour_utils.cpp
    # lib detectors
    src/lib/detectors/HsvColorDetector.cpp
    src/lib/detectors/HaarCascadeDetector.cpp
    src/lib/detectors/SizeDetector.cpp
    src/lib/detectors/HistColorDetector.cpp
    src/lib/detectors/DimentionDetector.cpp
    src/lib/detectors/HoughDetector.cpp    
    src/lib/detectors/BasicMotionDetector.cpp    
    src/lib/detectors/ArucoDetector.cpp
    src/lib/detectors/PoseDetector.cpp
    src/lib/detectors/TensorFlowDetector.cpp
    src/lib/detectors/FeatureDetector.cpp
    src/lib/detectors/DnnDetector.cpp
    src/lib/detectors/QrDetector.cpp
    src/lib/detectors/QrZbarDetector.cpp
    src/lib/detectors/LogicDetector.cpp
    src/lib/detectors/BlobDetector.cpp
    src/lib/detectors/DepthDetector.cpp
    src/lib/detectors/RoughDistanceDetector.cpp
    src/lib/detectors/DistanceDetector.cpp    
    src/lib/detectors/FaceDlibDetector.cpp 

    # lib relations
    src/lib/relations/ImageRangeRelation.cpp
    src/lib/relations/LogicRelations.cpp
    src/lib/relations/ThreeDimRangeRelation.cpp
    src/lib/relations/SpaceRelations.cpp
)

if(TensorFlow)
    target_compile_definitions(offline_video PUBLIC USE_TF) 
endif(TensorFlow)


if(opencv_contrib)
    target_compile_definitions(offline_video PUBLIC USE_OPENCV_CONTRIB)         
endif(opencv_contrib)
    

add_dependencies(extended_object_detection_node
    extended_object_detection_generate_messages_cpp
)

target_include_directories(
    extended_object_detection_node
    PRIVATE
    include
    src/lib/include
    src/lib/detectors/include
    src/lib/relations/include
    src/lib/utils/include
    src/lib/types/include
    ${OpenCV_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

if(TensorFlow)
    target_include_directories(
        extended_object_detection_node
        PRIVATE
        ${Protobuf_INCLUDE_DIRS}
        /usr/local/include/eigen3
        /usr/local/include/tf/
        /usr/local/include/tf/bazel-genfiles/
        /usr/local/include/tf/tensorflow/
        /usr/local/include/tf/third-party/
    )
endif(TensorFlow)

if(zbar_lib)
    target_include_directories(
        extended_object_detection_node
        PRIVATE
        ${ZBAR_INCLUDE_DIRS}
    )
endif(zbar_lib)

target_include_directories(
    hsv_color_params_collector_node
    PRIVATE
    include
    src/lib/include    
    src/lib/types/include
    src/lib/detectors/include
    src/lib/utils/include
    src/lib/relations/include
    ${OpenCV_INCLUDE_DIRS}    
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(
    hist_color_params_collector_point_node
    PRIVATE
    include
    src/lib/include    
    src/lib/types/include
    src/lib/detectors/include
    src/lib/utils/include
    src/lib/relations/include
    ${OpenCV_INCLUDE_DIRS}    
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(
    hist_color_params_collector_contour_node
    PRIVATE
    include
    src/lib/include    
    src/lib/types/include
    src/lib/detectors/include
    src/lib/utils/include
    src/lib/relations/include
    ${OpenCV_INCLUDE_DIRS}    
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(
    hough_circle_params_collector_node
    PRIVATE
    include
    src/lib/include    
    src/lib/types/include
    src/lib/detectors/include
    src/lib/utils/include
    src/lib/relations/include
    ${OpenCV_INCLUDE_DIRS}    
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(
    blob_params_collector_node
    PRIVATE
    include
    src/lib/include    
    src/lib/types/include
    src/lib/detectors/include
    src/lib/utils/include
    src/lib/relations/include
    ${OpenCV_INCLUDE_DIRS}    
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(
    offline_video
    PRIVATE
    include
    src/lib/include
    src/lib/detectors/include
    src/lib/relations/include
    src/lib/utils/include
    src/lib/types/include
    ${OpenCV_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

if(TensorFlow)
    target_link_libraries(extended_object_detection_node  
        /usr/local/src/lib/libtensorflow_cc.so
        /usr/local/src/lib/libtensorflow_framework.so
    )
    
    target_link_libraries(offline_video  
        /usr/local/src/lib/libtensorflow_cc.so
        /usr/local/src/lib/libtensorflow_framework.so
    )
endif(TensorFlow)

if(zbar_lib)
    target_link_libraries(extended_object_detection_node  
        ${ZBAR_LIBRARIES}
    )
endif(zbar_lib)

if(DLib)
    target_link_libraries(extended_object_detection_node  
        dlib::dlib
    )
    
    target_link_libraries(offline_video  
        dlib::dlib
    )

    target_include_directories(
        face_dlib_descriptors_extractor
        PRIVATE
        include
        src/lib/include    
        src/lib/types/include
        src/lib/detectors/include
        src/lib/utils/include
        src/lib/relations/include
        ${OpenCV_INCLUDE_DIRS}    
        ${catkin_INCLUDE_DIRS}
    )
    
    target_link_libraries(face_dlib_descriptors_extractor  
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}    
        dlib::dlib
    )

endif(DLib)

target_link_libraries(extended_object_detection_node  
  ${OpenCV_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${catkin_LIBRARIES}    
)

target_link_libraries(hsv_color_params_collector_node  
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}    
)

target_link_libraries(hist_color_params_collector_point_node  
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}    
)

target_link_libraries(hist_color_params_collector_contour_node  
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}    
)

target_link_libraries(hough_circle_params_collector_node  
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}    
)

target_link_libraries(blob_params_collector_node  
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}    
)

target_link_libraries(offline_video  
  ${OpenCV_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${catkin_LIBRARIES}      
)
