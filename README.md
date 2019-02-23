# robot_and_sensor_hw

ROS implementation of generic force-torque sensor. The implementation implements pluginlib infrastructure and node for integration and use in ROS.

The package provides a simulation of a FTS where a joystic is used for data input. This is often usefull for tests.

Note: Current version of the package depends on iirob/iirob_filters package for filtering capabilities. To install this package clone the [git repository](https://github.com/iirob/iirob_filters) into src folder of your workspace 

## ROS Distro Support

|         | Melodic |
|:-------:|:-------:|
| Branch  |[`master`](https://github.com/muritane/robot_and_sensor_hw/tree/master) |
| Status  |[![Build Status](https://github.com/muritane/robot_and_sensor_hw.svg?branch=master)](https://travis-ci.org/muritane/robot_and_sensor_hw) |
| Version |[version](http://repositories.ros.org/status_page/ros_melodic_default.html?q=robot_and_sensor_hw) |
