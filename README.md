# About sjtu_drone #
This repository is forked from ['tahsinkose/sjtu-drone'] (https://github.com/tahsinkose/sjtu-drone), which was originally forked from ['tum_simulator'] (http://wiki.ros.org/tum_simulator), which is developed with ROS + Gazebo. It is used for testing visual SLAM algorithms aiding with different sensors, such as IMU, sonar range finder and laser range finder. Here by 'sjtu', it means Shanghai Jiao Tong University. Currently, this program is used for testing algorithms for [UAV contest in SJTU](https://github.com/tahsinkose/sjtu-drone.git)

# Dependencies #
This package is tested for following configurations
1. Ros Melodic version (Ubuntu 18.04)
2. Gazebo version 9.14.0
3. apriltag_ros package.  Install with (sudo apt install ros-${ROS_DISTRO}-apriltag-ros)


# Download and Compiling #
```
$ cd <catkin_ws>/src
$ git clone https://github.com/MScTUDelft18-20/drone_simulation.git
$ cd <catkin_ws>
$ catkin build
```

Here <catkin_ws> is the path of the catkin workspace. Please refer to the [tutorial](http://wiki.ros.org/ROS/Tutorials) about how to create a catkin workspace in ROS.

# Run
The simplest way is calling after you have built the workspace successfully.

```
$ cd <catkin_ws>
$ source devel/setup.bash
$ roslaunch sjtu_drone simple.launch
```
# Running with keyboard
In second terminal:
```
$ rosrun sjtu_drone drone_keyboard
```
# Running autonomous mode when drone is landed
In third terminal:
```
$ roslaunch apriltag_ros apriltag_controller.launch
```

# Adding drone to the custom world
In ROS codespace, the robots are generally added to the environment by `spawn_model` node of `gazebo_ros` package via feeding the corresponding URDF file. However, in this case there isn't any URDF file. In future I might add a simple URDf file just trivially produces a base link for the entire robot. However, current method is directly adding to the all `.world` files as follows:

```
<include>
    <uri>model://sjtu_drone</uri>
    <pose>${pose_x} ${pose_y} 1 0 0 0</pose>
</include>
```
Add x and y coordinates of drone pose instead of ${pose_x} ${pose_y} respectively.

# Read sensor data from ROS topics #
One can use [rqt_gui](http://wiki.ros.org/rqt_gui) to have an extensive amount of utilities for topic visualization and manipulation. Some of the useful topics reside below.
```
forward looking camera :  /drone/front_camera/image_raw
downward looking camera: /drone/down_camera/image_raw
```
# Demo of apriltag PID position controller #
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/sjSDr9rDs4s/0.jpg)](https://www.youtube.com/watch?v=sjSDr9rDs4s)

# Objective #
The objective is to land the drone precisely on an april tag placed anywhere in the simulation world.

# Working of the controller #

Two Proprtional controllers are used to complete the objective; one for each camera message. 
The problem is divided into 2 main sections:

1. Detecting the apriltag with the front camera:

	A simple script is written in which the drone takes off and rises to a desired height. It then performs a full 360 degree yaw to scan the environment for a tag autonomously. As soon as the tag is detected, the position controller is activated. Position of the tag is updated and is fed as error to the controller. This error is minimized as the drone moves towards the marker.

2. Detecting the apriltag with the down camera:

	As the drone moves towards the tag, it is very likely that it loses sight of the tag. For this reason, the drone keeps moving towards the last updated direction. As the drone approaches the tag, it becomes visible in the down camera. The P controller for the down camera gets activated. The position of the tag is again fed as error to the controller and the x and y positional error is minimized such that the drone is right above the tag. Alongside the position correction, drone yaw is also corrected to align the drone's orientation to the apriltag. Once the desired error threshold is reached, land mode is activated.
