# Extended Object Detection
[![GitHub issues](https://img.shields.io/github/issues/Extended-Object-Detection-ROS/extended_object_detection.svg)](https://github.com/Extended-Object-Detection-ROS/extended_object_detection/issues) [![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) ![version](https://img.shields.io/badge/version-1.1.0-blue)

## Short information

Extended Object Detection is an ROS package and has ample opportunities for describing objects for detection in the video image. 
 - Author: Moscowsky Anton \<moscowskyad AT gmail DOT com\>
 - Wiki on english: https://github.com/Extended-Object-Detection-ROS/wiki_english/wiki
 - Wiki on russian: https://github.com/Extended-Object-Detection-ROS/extended_object_detection/wiki
 - Bug tracker: https://github.com/Extended-Object-Detection-ROS/extended_object_detection/issues
 - Youtube channel: https://youtube.com/channel/UCrZtFXAhxJIyk-T3d9-GLhw
 - ROS distro support: <a><img src="https://img.shields.io/badge/ROS-Noetic-blue" alt="ros_version_noetic" /></a> (full capabilities), <a><img src="https://img.shields.io/badge/ROS-Melodic-yellow" alt="ros_version_melodic" /></a> (partial capabilities), <a><img src="https://img.shields.io/badge/ROS-Kinetic-yellow" alt="ros_version_kinetic" /></a> (partial capabilities)

## Abstract

This package is a solution in the field of computer vision, which contains a set of various methods for detecting objects, united by one interface. To use the package, you do not need to dive into the API, all detection settings are available through a single configuration file. The package is developed within the "recognition by parts" paradigm, which allows you to specify various attributes of an object, such as image, color, shape, size, etc. These attributes are recognized separately and then aggregated into integral objects. In addition to recognizing simple objects, the package allows you to specify their combinations, detecting a complex object, which is a set of simple objects and relationships between them. The package provides a full-fledged ROS interface, allowing it to be used in robotics tasks.

## Cite
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

## Contests
 - [ROS Russia Open-Source Contest](https://habr.com/ru/post/541876/) - 1st Place
