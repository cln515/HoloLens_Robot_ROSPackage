
# HoloLens Robot Navigation

HoloLens-ROS bridge and calibration tool package for robot navigation (ROS side)
## Description
This is a package for robot navigation using two dimensional map. This package provides functions to calibrate the robot and HoloLens and to localize the robot on the map using SLAM function of the HoloLens 

Paper(arXiv.com)

## Requirement
Ceres solver(http://ceres-solver.org)

Follow the instructions on http://ceres-solver.org/installation.html. The minimum configuration is sufficient


## Install

	$catkin build hololens_robot_navigation

It is necessary to install a dedicated program in HoloLens. For installation of the HoloLens side program, see here.


## Usage

Start Program

	$roslaunch horolens_robot_navigation HoloLensLocalizer.launch hololens_ip:=<HoloLens ip> holo_linked_frame:=<Frame of the robot to which the HoloLens is attached> robot_odom_frame:=<Origin of robot's odometry> robot_foot_frame:=<Robot's ground plane frame> calib_file_path:=<calibration parameter file path>

Example

	$roslaunch horolens_robot_navigation HoloLensLocalizer.launch hololens_ip:=1.2.3.4 holo_linked_frame:=head robot_odom_frame:=odom robot_foot_frame:=base_link calib_file_path:=$(rospack find hololens_robot_navigation)/params/caslib.dat

How to Calibrate

- When the robot can rotate in the vertical direction
	- Press the space key and record the first position and pose
	- Rotate the frame to which HoloLens is attached in the horizontal direction
	- Press the space key and record the second position and pose
	- Rotate the frame to which HoloLens is attached in the vertical direction
	- Press the space key and record the third position and pose
- When the robot cannot rotate in the vertical direction
	- Press the space key and record the first position and pose
	- Rotate the robot horizontally
	- Press the space key and record the second position and pose
	- Advancing the robot
	- Press the space key and record the third position and pose

Push “c” key and calibration is performed.

Localization of Holo Lenz
-Start RViz
-Open /rviz/HoloLensNav.rviz
-With a floor map published on another node, use the "2D pose estimate" tool to specify the location of the HoloLens on RViz

## Licence
This project is licensed under the MIT license

## Author
Ryoichi Ishikawa, Computer Vision Laboratory, The University of Tokyo
