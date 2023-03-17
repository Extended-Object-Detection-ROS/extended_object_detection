# Change Log

## [[2.0.0](https://github.com/Extended-Object-Detection-ROS/extended_object_detection/tree/r2.0.0)] - 2023-03-15
Fixed dozens of bugs.  
Added mask process in DNN modules.  
Added interface for libtorch (with YOLOv7).  
Changed idea, how to process camera information in lib core.  
Added automatic migration of detection results between different channels.  
Now main node can subscribe multiple cameras.  
For each subscribed camera where are statistic output.  
Now can subscribe depth-maps in two main formats.  
Unite output message formats.  
Added soft mode detection for complex objects.

## [[1.1.0](https://github.com/Extended-Object-Detection-ROS/extended_object_detection/tree/r1.1.0)] - 2021-11-21
Now core of solution is stand-alone module.
Changed to new way to store extracted info data, so output topics format are changed.
Added two attributes of extracted data checking.
Added size relations.
Fix minor bugs.

## [[1.0.1](https://github.com/Extended-Object-Detection-ROS/extended_object_detection/tree/r1.0.1)] - 2021-10-5
Fully changed logic of complex object detection.
Now this part is optional and need igraph library installed (see [wiki](https://github.com/Extended-Object-Detection-ROS/extended_object_detection/wiki/igraph_install)).
Also added offline_image node.

## [unreleased] - 2021-09-27
Added soft mode tracking.

## [unreleased] - 2020-12-15
Added face recognition with dlib.

## [unreleased] - 2020-12-10
Added RangeAttribute

## [[1.0.0](https://github.com/Extended-Object-Detection-ROS/extended_object_detection/tree/r1.0.0)] - 2020-11-22
First Release
