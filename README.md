# Extended Object Detection
![EOD logo](animated_logo.gif)  
[![GitHub issues](https://img.shields.io/github/issues/Extended-Object-Detection-ROS/extended_object_detection.svg)](https://github.com/Extended-Object-Detection-ROS/extended_object_detection/issues) [![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) ![version](https://img.shields.io/badge/version-2.0.0-blue)

## Short information

Extended Object Detection is an ROS package and has ample opportunities for describing objects for detection in the video image. 
 - Author: Moscowsky Anton \<moscowskyad AT gmail DOT com\>
 - Wiki on english: https://github.com/Extended-Object-Detection-ROS/wiki_english/wiki
 - Wiki on russian: https://github.com/Extended-Object-Detection-ROS/extended_object_detection/wiki (outdated)
 - Bug tracker: https://github.com/Extended-Object-Detection-ROS/extended_object_detection/issues
 - Youtube channel: https://youtube.com/channel/UCrZtFXAhxJIyk-T3d9-GLhw
 - ROS distro support: <a><img src="https://img.shields.io/badge/ROS-Noetic-blue" alt="ros_version_noetic" /></a> (full capabilities), <a><img src="https://img.shields.io/badge/ROS-Melodic-yellow" alt="ros_version_melodic" /></a> (partial capabilities), <a><img src="https://img.shields.io/badge/ROS-Kinetic-yellow" alt="ros_version_kinetic" /></a> (partial capabilities)

## Abstract

This package is a solution in the field of computer vision, which contains a set of various methods for detecting objects, united by one interface. To use the package, you do not need to dive into the API, all detection settings are available through a single configuration file. The package is developed within the "recognition by parts" paradigm, which allows you to specify various attributes of an object, such as image, color, shape, size, etc. These attributes are recognized separately and then aggregated into integral objects. In addition to recognizing simple objects, the package allows you to specify their combinations, detecting a complex object, which is a set of simple objects and relationships between them. The package provides a full-fledged ROS interface, allowing it to be used in robotics tasks.

## Cite
Repository
```bibtex
@misc{extended_object_detection,
 author = {Moscowsky, Anton},
 booktitle = {GitHub repository},
 publisher = {GitHub},
 title = {{Extended Object Detection}},
 url = {https://github.com/Extended-Object-Detection-ROS/extended_object_detection},
 year = {2020}
}
```
Article
```bibtex
@incollection{moscowsky_eod,
 author = {Moscowsky, A. D.},
 booktitle = {Smart Electromechanical Systems: Recognition, Identification and Modeling},
 doi = {10.1007/978-3-030-97004-8_3},
 keywords = {Computer vision,Object detection,ROS,Robotic},
 pages = {27--43},
 title = {{Extended Object Detection: Flexible Object Description System for Detection in Robotic Tasks}},
 url = {https://link.springer.com/10.1007/978-3-030-97004-8_3},
 volume = {419},
 year = {2022}
}

```

## Contests
 - [ROS Russia Open-Source Contest](https://habr.com/ru/post/541876/) - 1st Place
 - [OpenCV Blog Olympics Contest](https://learnopencv.com/blog-olympics/) - 1st Place ([post](https://learnopencv.com/multi-attribute-and-graph-based-object-detection/?ck_subscriber_id=1013959305))
