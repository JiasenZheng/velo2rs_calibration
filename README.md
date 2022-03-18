# Velo2rs Calibration ROS Package

This is a light-weighted and simple ROS package used to calibrate and find extrinsic parameters between a lidar and a RGB-D camera. The package provides an easy approach to find reference points from point clouds in Rviz for further calculation. The package is also compatable for any two sensors that measures `sensor_msgs::PointCloud2`.

## Calibration target

The calibration target can be any rigid body object with rectagular flat surfaces, such as a acrylic sheet, a card board, or a package box.

## Usage 

First, you need to identify the topic published by the sensors to be calibrated. In particular the package expects 3D point clouds (sensor_msgs::PointCloud2) from both your Lidar and RGB-D camera.

To start the calibration procedure, the reference point collection node must be run. The `find_points` launch file setup a calibration environment in Gazebo with data collection services.

**find_points.launch**
* find_rs_points: a node to find and collect reference points from the realsense camera
* find_velo_points: a node to find and collect reference points from the velodyne-16 Lidar
* /set_pos: a service to set a square-shape region on the target surface to locate reference points
    - arg: horizontal_coord: 
