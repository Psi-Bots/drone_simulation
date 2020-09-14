# About sjtu_drone #
This repository is forked from ['tahsinkose/sjtu-drone'] (https://github.com/tahsinkose/sjtu-drone), which was originally forked from ['tum_simulator'] (http://wiki.ros.org/tum_simulator), which is developed with ROS + Gazebo. It is used for testing visual SLAM algorithms aiding with different sensors, such as IMU, sonar range finder and laser range finder. Here by 'sjtu', it means Shanghai Jiao Tong University. Currently, this program is used for testing algorithms for [UAV contest in SJTU](http://mediasoc.sjtu.edu.cn/wordpress)

# Requirements #
This package is tested for following configurations
1. Ros Melodic version (Ubuntu 18.04)
2. Gazebo version 9.14.0

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
[![Alt text](https://www.youtube.com/watch?v=sjSDr9rDs4s/0.jpg)](https://www.youtube.com/watch?v=sjSDr9rDs4s)
