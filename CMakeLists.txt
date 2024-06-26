cmake_minimum_required(VERSION 3.5)
project(extended_object_detection)


## Optional parts of library
set(opencv_contrib OFF)
set(zbar_lib OFF)
set(DLib OFF)
set(igraph OFF)
set(torch OFF)
set(tracking OFF)
set(OFFLINE OFF)
set(UTILS OFF)
set(OAK OFF)


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
  tf2
  tf2_ros    
#   depthai_ros_extended_msgs
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

if(torch)    
    set(Torch_DIR "/home/robofob/Lib/libtorch/share/cmake/Torch/")    
    find_package(Torch REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")    
    add_compile_options(-std=c++17)
else()
    add_compile_options(-std=c++11)
endif(torch)


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
  Keypoints.msg
  BaseObject.msg
  SimpleObjectArray.msg
  ComplexObject.msg
  ComplexObjectArray.msg  
  StatsStream.msg
  StatsArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetObjects.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs  
)

catkin_package(
   CATKIN_DEPENDS cv_bridge std_msgs sensor_msgs image_transport roscpp image_geometry cmake_modules message_runtime #depthai_ros_extended_msgs
   DEPENDS TinyXML   
)


###########
## Build ##
###########
set( EOD_SRC
    # lib common
    src/lib/ObjectBase.cpp
    
    # types
    src/lib/types/ComplexObjectGraph.cpp
    src/lib/types/ComplexObject.cpp
    src/lib/types/ExtendedObjectInfo.cpp
    src/lib/types/Attribute.cpp
    src/lib/types/SimpleObject.cpp
    src/lib/types/Tracker.cpp
    src/lib/types/Relationship.cpp    
    src/lib/types/Filtering.cpp    
    src/lib/types/Clusterization.cpp   
    src/lib/types/InfoImage.cpp
    
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
    src/lib/detectors/ExtractedInfoIdChecker.cpp
    src/lib/detectors/ExtractedInfoStringChecker.cpp
    src/lib/detectors/UnitTranslationExtracter.cpp
    src/lib/detectors/SquareObjectDistanceDetector.cpp
    src/lib/detectors/TorchYOLOv7Detector.cpp
    src/lib/detectors/KeypointPoseDetector.cpp
    src/lib/detectors/DummyHumanPoseDetector.cpp


    # lib relations        
    src/lib/relations/ImageRangeRelation.cpp
    src/lib/relations/LogicRelations.cpp
    src/lib/relations/ThreeDimRangeRelation.cpp
    src/lib/relations/SpaceRelations.cpp
    src/lib/relations/SizeRelations.cpp
    src/lib/relations/SameExtractedInfoRelation.cpp
    )

set ( EOD_INCLUDE
    src/lib/include
    src/lib/detectors/include
    src/lib/relations/include
    src/lib/utils/include
    src/lib/types/include
    )
    
# main node
add_executable(
    extended_object_detection_node
    src/extended_object_detection_node/eod_node.cpp
    ${EOD_SRC}    
    src/lib/detectors/ROSSubscriberOpenPoseRaw.cpp
)

if( OAK )
    target_compile_definitions(extended_object_detection_node PUBLIC USE_ROS)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_OAK)
endif( OAK )

if(opencv_contrib)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_OPENCV_CONTRIB)         
endif(opencv_contrib)

if(zbar_lib)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_ZBAR)
endif(zbar_lib)

if(DLib)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_DLIB)    
endif(DLib)

if(igraph)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_IGRAPH)    
endif(igraph)

if(torch)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_TORCH) 
endif(torch)

if(tracking)
    target_compile_definitions(extended_object_detection_node PUBLIC USE_TRACKING) 
endif(tracking)

#collect hsv color c++ utility
if(UTILS)
    add_executable(
        hsv_color_params_collector_node
        src/utils/hsv_color_params_collector.cpp
        # types
        src/lib/types/ExtendedObjectInfo.cpp
        src/lib/types/Attribute.cpp
        src/lib/types/SimpleObject.cpp
        src/lib/types/Filtering.cpp
        src/lib/types/Clusterization.cpp    
        src/lib/types/InfoImage.cpp
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
        src/lib/types/InfoImage.cpp
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
        src/lib/types/InfoImage.cpp
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
        src/lib/types/InfoImage.cpp
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
        src/lib/types/InfoImage.cpp
        # utils    
        src/lib/utils/geometry_utils.cpp
        src/lib/utils/drawing_utils.cpp
        src/lib/utils/contour_utils.cpp
        # detectors
        src/lib/detectors/BlobDetector.cpp    
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
            src/lib/types/InfoImage.cpp
            # utils    
            src/lib/utils/geometry_utils.cpp
            src/lib/utils/drawing_utils.cpp
            src/lib/utils/contour_utils.cpp
            # detectors
            src/lib/detectors/FaceDlibDetector.cpp
        )
        
        target_compile_definitions(face_dlib_descriptors_extractor PUBLIC USE_DLIB)
    endif(DLib)
endif(UTILS)
    
if(OFFLINE)
    #offline video processor
    add_executable(
        offline_video
        src/offline/offline_video.cpp
        ${EOD_SRC}
    )

    #offline image processor
    add_executable(
        offline_image
        src/offline/offline_image.cpp
        ${EOD_SRC}
    )

    if(TensorFlow)
        target_compile_definitions(offline_video PUBLIC USE_TF)
        target_compile_definitions(offline_image PUBLIC USE_TF) 
    endif(TensorFlow)


    if(opencv_contrib)
        target_compile_definitions(offline_video PUBLIC USE_OPENCV_CONTRIB)
        target_compile_definitions(offline_image PUBLIC USE_OPENCV_CONTRIB)         
    endif(opencv_contrib)
        
    if(igraph)
        target_compile_definitions(offline_video PUBLIC USE_IGRAPH)
        target_compile_definitions(offline_image PUBLIC USE_IGRAPH)         
    endif(igraph)
endif(OFFLINE)
    
add_dependencies(extended_object_detection_node
    extended_object_detection_generate_messages_cpp
)

target_include_directories(
    extended_object_detection_node
    PRIVATE
    include
    ${EOD_INCLUDE}
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

if(igraph)
    target_include_directories(
        extended_object_detection_node
        PRIVATE
        /usr/local/src/igraph-0.7.1/include/
    )
    if(OFFLINE)
        target_include_directories(
            offline_video
            PRIVATE
            /usr/local/src/igraph-0.7.1/include/
        )
        
        target_include_directories(
            offline_image
            PRIVATE
            /usr/local/src/igraph-0.7.1/include/
        )
    endif(OFFSET)
endif(igraph)

if(UTILS)
    target_include_directories(
        hsv_color_params_collector_node
        PRIVATE
        include
        ${EOD_INCLUDE}
        ${OpenCV_INCLUDE_DIRS}    <<<<<<< HEAD
        ${catkin_INCLUDE_DIRS}
    )

    target_include_directories(
        hist_color_params_collector_point_node
        PRIVATE
        include
        ${EOD_INCLUDE}
        ${OpenCV_INCLUDE_DIRS}    
        ${catkin_INCLUDE_DIRS}
    )

    target_include_directories(
        hist_color_params_collector_contour_node
        PRIVATE
        include
        ${EOD_INCLUDE}
        ${OpenCV_INCLUDE_DIRS}    
        ${catkin_INCLUDE_DIRS}
    )

    target_include_directories(
        hough_circle_params_collector_node
        PRIVATE
        include
        ${EOD_INCLUDE}
        ${OpenCV_INCLUDE_DIRS}    
        ${catkin_INCLUDE_DIRS}
    )

    target_include_directories(
        blob_params_collector_node
        PRIVATE
        include
        ${EOD_INCLUDE}
        ${OpenCV_INCLUDE_DIRS}    
        ${catkin_INCLUDE_DIRS}
    )
endif(UTILS)

if(OFFLINE)
    target_include_directories(
        offline_video
        PRIVATE
        include
        ${EOD_INCLUDE}
        ${OpenCV_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
    )

    target_include_directories(
        offline_image
        PRIVATE
        include
        ${EOD_INCLUDE}
        ${OpenCV_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
    )
endif(OFFLINE)

if(TensorFlow)
    target_link_libraries(extended_object_detection_node  
        /usr/local/src/lib/libtensorflow_cc.so
        /usr/local/src/lib/libtensorflow_framework.so
    )
    
    if(OFFLINE)
        target_link_libraries(offline_video  
            /usr/local/src/lib/libtensorflow_cc.so
            /usr/local/src/lib/libtensorflow_framework.so
        )
    endif(OFFLINE)
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
    
    if(OFFLINE)
    target_link_libraries(offline_video  
        dlib::dlib
    )
    endif(OFFLINE)

    if(UTILS)
        target_include_directories(
            face_dlib_descriptors_extractor
            PRIVATE
            include
            ${EOD_INCLUDE}
            ${OpenCV_INCLUDE_DIRS}    
            ${catkin_INCLUDE_DIRS}
        )
        
        target_link_libraries(face_dlib_descriptors_extractor  
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}    
            dlib::dlib
        )
    endif(UTILS)

endif(DLib)

if(igraph)    
    target_link_libraries(extended_object_detection_node  
        /usr/local/lib/libigraph.so
    )
    
    if(OFFLINE)
        target_link_libraries(offline_video  
            /usr/local/lib/libigraph.so
        )
        
        target_link_libraries(offline_image  
            /usr/local/lib/libigraph.so
        )
    endif(OFFLINE)
endif(igraph)

if(torch)    
    set_property(TARGET extended_object_detection_node PROPERTY CXX_STANDARD 17)
    if(OFFLINE)
        set_property(TARGET offline_video PROPERTY CXX_STANDARD 17)
        set_property(TARGET offline_image PROPERTY CXX_STANDARD 17)
    endif(OFFLINE)
    
    if(UTILS)
        set_property(TARGET hough_circle_params_collector_node PROPERTY CXX_STANDARD 17)
        set_property(TARGET hist_color_params_collector_contour_node PROPERTY CXX_STANDARD 17)
        set_property(TARGET hist_color_params_collector_point_node PROPERTY CXX_STANDARD 17)
        set_property(TARGET blob_params_collector_node PROPERTY CXX_STANDARD 17)
        set_property(TARGET hsv_color_params_collector_node PROPERTY CXX_STANDARD 17)
    endif(UTILS)
    
#     set_property(TARGET extended_object_detection_node PROPERTY CXX_STANDARD 17)
#     set_property(TARGET offline_video PROPERTY CXX_STANDARD 17)
#     set_property(TARGET offline_image PROPERTY CXX_STANDARD 17)

    target_link_libraries(extended_object_detection_node  
        ${TORCH_LIBRARIES}
    )
    
    if(OFFLINE)
        target_link_libraries(offline_video  
            ${TORCH_LIBRARIES}
        )
        
        target_link_libraries(offline_image  
            ${TORCH_LIBRARIES}
        )
    endif(OFFLINE)
endif(torch)


target_link_libraries(extended_object_detection_node  
  ${OpenCV_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${catkin_LIBRARIES}    
  
)

if(UTILS)
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
endif(UTILS)

if(OFFLINE)
    target_link_libraries(offline_video  
    ${OpenCV_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${catkin_LIBRARIES}      
    )

    target_link_libraries(offline_image  
    ${OpenCV_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${catkin_LIBRARIES}      
    )
endif(OFFLINE)
