#ifndef ARDRONE_ROS_H
#define ARDRONE_ROS_H
#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "tf/tf.h"

/**
 * @brief A simple class to send the commands to the drone through 
 * the corresponding topics
 */

class DroneObjectROS
{
protected:
    DroneObjectROS() {}

public:
    DroneObjectROS(ros::NodeHandle &node)
    {
        initROSVars(node);
    }

    bool isFlying;
    bool isPosctrl;
    bool isVelMode;

    float error_front_x = 0;
    float error_front_y = 0;

    float error_down_x = 0;
    float error_down_y = 0;

    // variables to store the velocity commands to send to the drone
    float velocity_front_x = 0;
    float velocity_front_y = 0;

    float velocity_down_x = 0;
    float velocity_down_y = 0;

    // variables to store the yaw velocities to send to the drone
    float velocity_down_yaw = 0;

    // proportional gain values for velocity in x-axis
    float kp_front_x = 0.5;
    float kp_down_x = 0.5;

    // proportional gain values for velocity in y-axis
    float kp_front_y = 0.5;
    float kp_down_y = 0.5;

    // proportional gain values for yaw of the drone
    float kp_down_yaw = 0.1;
    bool tag_detected = false;

    ros::Subscriber front_sub;
    ros::Subscriber down_sub;
    ros::Publisher pubTakeOff;
    ros::Publisher pubLand;
    ros::Publisher pubReset;
    ros::Publisher pubCmd;
    ros::Publisher pubPosCtrl;
    ros::Publisher pubVelMode;
    std::vector<apriltag_ros::AprilTagDetection> tag;
    geometry_msgs::Twist twist_msg;
    geometry_msgs::Pose arucopose_down, arucopose_front;

    void initROSVars(ros::NodeHandle &node);
    void frontCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    void downCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

    bool takeOff();
    bool land();
    bool hover();
    bool posCtrl(bool on);
    bool velMode(bool on);
    bool Autoflight();

    // commands for controling ARDrone
    // pitch_lr = left-right tilt		(-1) to right		(+1)
    // roll_fb = front-back tilt 		(-1) to backwards	(+1)
    // v_du = velocity downwards	(-1) to upwards		(+1)
    // w_lr = angular velocity left (-1) to right		(+1)

    bool move(float v_lr, float v_fb, float v_du, float w_lr);
    bool moveTo(float x, float y, float z);
    bool pitch(float x = 1.0, float y = 1.0, float speed = 0.2);
    bool roll(float x = 1.0, float y = 1.0, float speed = 0.2);
    bool rise(float speed = 0.1);
    bool yaw(float speed = 0.1);
};

#endif // ARDRONE_ROS_H
