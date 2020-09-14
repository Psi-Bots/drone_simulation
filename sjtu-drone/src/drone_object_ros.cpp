#include "drone_object_ros.h"

void DroneObjectROS::initROSVars(ros::NodeHandle &node)
{
    // Setting Flight Modes
    isFlying = false;
    isPosctrl = false;
    isVelMode = false;

    // Publishers
    pubTakeOff = node.advertise<std_msgs::Empty>("/drone/takeoff", 1024);
    pubLand = node.advertise<std_msgs::Empty>("/drone/land", 1024);
    pubReset = node.advertise<std_msgs::Empty>("/drone/reset", 1024);
    pubPosCtrl = node.advertise<std_msgs::Bool>("/drone/posctrl", 1024);
    pubCmd = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1024);
    pubVelMode = node.advertise<std_msgs::Bool>("/drone/vel_mode", 1024);
    
    // Subscribers
    front_sub = node.subscribe("/front_tag_detection", 10, &DroneObjectROS::frontCallback, this);
    down_sub = node.subscribe("/down_tag_detection", 10, &DroneObjectROS::downCallback, this);
}

/**
 * @brief A callback function for the front facing camera topic. 
 * Function is activated as soon as the marker is detected.
 * Publishes desired twist message to the topic /cmd_vel using the P controller.
 */
void DroneObjectROS::frontCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    ROS_INFO("Front Tag Callback");
    if (msg->detections.size() > 0)
    {
        tag_detected = true;

        ROS_INFO("Front Tag Detected");
        arucopose_front.position = msg->detections[0].pose.pose.pose.position;
        arucopose_front.orientation = msg->detections[0].pose.pose.pose.orientation;

        error_front_x = arucopose_front.position.x;
        error_front_y = arucopose_front.position.z;

        // # pid loop for x and y velocities
        velocity_front_x = kp_front_x * error_front_x;
        velocity_front_y = kp_front_y * error_front_y;

        twist_msg.linear.x = velocity_front_y;
        twist_msg.linear.y = -velocity_front_x;
        twist_msg.linear.z = 0;
        twist_msg.angular.x = 0;
        twist_msg.angular.y = 0;
        twist_msg.angular.z = 0;

        pubCmd.publish(twist_msg);
    }
}

/**
 * @brief A callback function for the downward facing camera topic. 
 * 
 * Function is activated as soon as the marker is detected by the downward facing camera.
 * P controller reduces position error to position the drone exactly above the tag.
 * Then yaw correction is performed to match the orientation of the tag. 
 * Lands when the desired error threshold is reached.
 */
void DroneObjectROS::downCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (msg->detections.size() > 0)
    {
        ROS_INFO("dOWN Tag Detected");
        arucopose_down.position = msg->detections[0].pose.pose.pose.position;
        arucopose_down.orientation = msg->detections[0].pose.pose.pose.orientation;

        tf::Quaternion q(arucopose_down.orientation.x, arucopose_down.orientation.y, arucopose_down.orientation.z, arucopose_down.orientation.w);
        tf::Matrix3x3 rpy(q);
        double error_down_roll, error_down_pitch, error_down_yaw;
        rpy.getRPY(error_down_roll, error_down_pitch, error_down_yaw);

        error_down_x = arucopose_down.position.x;
        error_down_y = arucopose_down.position.y;

        velocity_down_x = kp_down_x * error_down_x;
        velocity_down_y = kp_down_y * error_down_y;
        velocity_down_yaw = kp_down_yaw * error_down_yaw;

        twist_msg.linear.x = -velocity_down_y;
        twist_msg.linear.y = -velocity_down_x;
        twist_msg.linear.z = 0;
        twist_msg.angular.x = 0;
        twist_msg.angular.y = 0;
        twist_msg.angular.z = velocity_down_yaw;

        pubCmd.publish(twist_msg);

        if ((abs(error_down_x) < 0.01) && (abs(error_down_y) < 0.01) && (fmod(error_down_yaw, M_PI) < 0.01))
        {
            double secs_up = ros::Time::now().toSec();
            while ((ros::Time::now().toSec() - secs_up) < 2.0)
            {
                twist_msg.linear.x = 0;
                twist_msg.linear.y = 0;
                twist_msg.linear.z = -1;
                twist_msg.angular.x = 0;
                twist_msg.angular.y = 0;
                twist_msg.angular.z = 0;
                pubCmd.publish(twist_msg);
            }
            DroneObjectROS::land();
        }
    }
}

/**
 * @brief Command the drone to takeoff. 
 */
bool DroneObjectROS::takeOff()
{
    if (isFlying)
        return false;

    pubTakeOff.publish(std_msgs::Empty());
    ROS_INFO("Taking Off...");

    isFlying = true;
    return true;
}

/**
 * @brief Command the drone to land 
 */
bool DroneObjectROS::land()
{
    if (!isFlying)
        return false;

    pubLand.publish(std_msgs::Empty());
    ROS_INFO("Landing...");

    isFlying = false;
    return true;
}

/**
 * @brief Command the drone to hover 
 */
bool DroneObjectROS::hover()
{
    if (!isFlying)
        return false;

    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    pubCmd.publish(twist_msg);
    ROS_INFO("Hovering...");
    return true;
}

/**
 * @brief Put the drone into position control mode
 */
bool DroneObjectROS::posCtrl(bool on)
{
    if (!isFlying)
        return false;

    isPosctrl = on;
    std_msgs::Bool bool_msg;
    bool_msg.data = on ? 1 : 0;

    pubPosCtrl.publish(bool_msg);

    if (on)
        ROS_INFO("Switching position control on...");
    else
        ROS_INFO("Switching position control off...");

    return true;
}

/**
 * @brief Put the drone into velocity control mode
 */
bool DroneObjectROS::velMode(bool on)
{
    if (!isFlying)
        return false;

    isVelMode = on;
    std_msgs::Bool bool_msg;
    bool_msg.data = on ? 1 : 0;

    pubVelMode.publish(bool_msg);

    if (on)
        ROS_INFO("Switching velocity mode on...");
    else
        ROS_INFO("Switching velocity mode off...");

    return true;
}

/**
 * @brief Command the drone to rise as per desired orientation 
 */
bool DroneObjectROS::move(float lr, float fb, float ud, float w)
{
    if (!isFlying)
        return false;

    twist_msg.linear.x = 1.0;
    twist_msg.linear.y = 1.0;
    twist_msg.linear.z = ud;
    twist_msg.angular.x = lr;
    twist_msg.angular.y = fb;
    twist_msg.angular.z = w;
    pubCmd.publish(twist_msg);
    ROS_INFO("Moving...");
    return true;
}

/**
 * @brief Command the drone to move to a desired position
 */
bool DroneObjectROS::moveTo(float x, float y, float z)
{
    if (!isFlying)
        return false;

    twist_msg.linear.x = x;
    twist_msg.linear.y = y;
    twist_msg.linear.z = z;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = 0;

    pubCmd.publish(twist_msg);
    ROS_INFO("Moving...");
}

/**
 * @brief Command the drone to pitch at desired speed 
 */
bool DroneObjectROS::pitch(float x, float y, float speed)
{
    if (!isFlying)
        return false;

    twist_msg.linear.x = x;
    twist_msg.linear.y = y;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = speed;
    twist_msg.angular.z = 0.0;
    pubCmd.publish(twist_msg);
    ROS_INFO("Pitching...");
    return true;
}

/**
 * @brief Command the drone to roll at desired speed 
 */
bool DroneObjectROS::roll(float x, float y, float speed)
{
    if (!isFlying)
        return false;

    twist_msg.linear.x = x;
    twist_msg.linear.y = y;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = speed;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    pubCmd.publish(twist_msg);
    ROS_INFO("Rolling...");
    return true;
}

/**
 * @brief Command the drone to rise at desired speed 
 */
bool DroneObjectROS::rise(float speed)
{
    if (!isFlying)
        return false;

    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = speed;
    twist_msg.angular.x = 0.0; //flag for preventing hovering
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    pubCmd.publish(twist_msg);
    ROS_INFO("Rising...");
    return true;
}

/**
 * @brief Command the drone to yaw at desired speed 
 */
bool DroneObjectROS::yaw(float speed)
{
    if (!isFlying)
        return true;

    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0.0; //flag for preventing hovering
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = speed;
    pubCmd.publish(twist_msg);
    ROS_INFO("Turning head...");
    return true;
}

/**
 * @brief Autonomous Flight.
 * 
 * Commands the drone to takeoff and then rise higher for 2 seconds.
 * Next, commands the drone to perform a full 360 degrees yaw motion.
 * If the tag is detected in the front camera, frontCallback function is executed. 
 */
bool DroneObjectROS::Autoflight()
{
    if (isFlying)
        return false;
    pubTakeOff.publish(std_msgs::Empty());
    ROS_INFO("Taking Off...");
    isFlying = true;

    double secs_up = ros::Time::now().toSec();
    while ((ros::Time::now().toSec() - secs_up) < 2.0)
    {
        DroneObjectROS::rise(0.4f);
    }
    double secs_yaw = ros::Time::now().toSec();

    while ((ros::Time::now().toSec() - secs_yaw) < 9.0)
    {
        if (!tag_detected)
        {
            DroneObjectROS::yaw(0.8f);
        }
    }
    ROS_INFO("Autnomous Flight");
    return true;
}