# Visual Servoing (eye-in-hand)

## Overview
This package implements visual servoing (eye-in-hand) for Kinova Jaco2 7-dof robot arm.

https://github.com/robot0826/visual_servoing/assets/139858045/7ffd6c37-4c5d-4344-8bf1-b7f9334b516f

## Requirements
- Kinova Jaco2 7-dof robot arm
- Intel RealSense D435i
- april.tag.Tag36h11, id = 0 (size = 17.2 mm)

## Dependencies
- Ubuntu 20.04
- ROS noetic
- Python 3.8
- [kinova-ros](https://github.com/Kinovarobotics/kinova-ros.git) (modified, refer to the below)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros.git)
- [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros.git) (modified, refer to the below)
- rviz (optional, for visualization)

## Installation and Execution
Clone this repository in your ```/catkin_ws/src``` directory and use ```catkin build```.

Enter the commands below in order.
1. ```roslaunch kinova_bringup kinova_robot.launch```
2. ```roslaunch realsense2_camera rs_camera.launch```
3. ```roslaunch apriltag_ros continuous_detection.launch```
4. ```rosservice call /j2s7s300_driver/in/home_arm```
5. ```rosrun visual_servoing main.py```

## Modified Parts (for kinova-ros and apriltag_ros)
- For apriltag_ros/apriltag_ros/launch/continuous_detection.launch
  - ```<arg name="camera_name" default="/camera/color" />```
  - ```<arg name="image_topic" default="image_raw" />```
  - ```<remap from="image_rect" to="$(arg camera_name)/image_raw" />```
  - ```<remap from="camera_info" to="$(arg camera_name)/camera_info" />```
  - ```<node pkg="tf2_ros" type="static_transform_publisher" name="robot_camera_bridge" args="0 0.063 -0.105 -1.57079632679 -1.57079632679 0 j2s7s300_end_effector camera_link"/>```
  - ```<node pkg="tf2_ros" type="static_transform_publisher" name="tag_desired_ee_bridge" args="0 0 0.6 0 3.14159265359 0 tag_0 desired_end_effector"/>```
- For kinova-ros/kinova_bringup/launch/config/robot_parameters.yaml
  - ```jointSpeedLimitParameter1: 20``` (default = 10)
- For kinova-ros/kinova_description/urdf/j2s7s300.xacro
  - ```<xacro:property name="joint_1_velocity_limit" value="${72*J_PI/180}" />``` (default = 36*J_PI/180)
  - ```<xacro:property name="joint_2_velocity_limit" value="${72*J_PI/180}" />``` (default = 36*J_PI/180)
  - ```<xacro:property name="joint_3_velocity_limit" value="${72*J_PI/180}" />``` (default = 36*J_PI/180)
  - ```<xacro:property name="joint_4_velocity_limit" value="${72*J_PI/180}" />``` (default = 36*J_PI/180)
  - ```<xacro:property name="joint_5_velocity_limit" value="${96*J_PI/180}" />``` (default = 48*J_PI/180)
  - ```<xacro:property name="joint_6_velocity_limit" value="${96*J_PI/180}" />``` (default = 48*J_PI/180)
  - ```<xacro:property name="joint_7_velocity_limit" value="${96*J_PI/180}" />``` (default = 48*J_PI/180)
