#include <DialogKeyboard.h>
#include <QtWidgets>
#include "ui_DialogKeyboard.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

DialogKeyboard::DialogKeyboard(QWidget *parent) : QDialog(parent),
                                                  ui(new Ui::DialogKeyboard), n("")
{
    ui->setupUi(this);
    // ros::NodeHandle n;
    sub = n.subscribe("/aruco_single/pose", 10, &DialogKeyboard::poseCallback, this);
    // ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, &DialogKeyboard::poseCallback, this);
}

DialogKeyboard::~DialogKeyboard()
{
    delete ui;
}

void DialogKeyboard::poseCallback(const geometry_msgs::PoseStamped msg)
{
    // dronepose = msg;
    dronepose.pose.position.x = 1;
    dronepose.pose.position.y = 1;
    dronepose.pose.position.z = 20;
    ROS_INFO("I Heard");

    std::cout << " I heard "  << std::endl;
}

void DialogKeyboard::keyPressEvent(QKeyEvent *event)
{
    if (!drone)
        return;
    char key = event->key();
    std::cout << "key:" << key << std::endl;
    switch (key)
    {
    case 'Z':
        //take off
        std::cout << "take off !" << std::endl;
        drone->takeOff();
        break;
    case 'X':
        //land
        drone->land();
        break;
    case 'H':
        drone->hover();
        break;
    case 'I':
        //going up
        drone->rise(0.4f);
        break;
    case 'K':
        //going down
        drone->rise(-0.4f);
        break;
    case 'J':
        //turn left
        drone->yaw(0.4f);
        break;
    case 'L':
        //turn right
        drone->yaw(-0.4f);
        break;
    case 'A':
        //tilt left
        if (!drone->isVelMode)
            drone->roll(0.0f, -1.0f, -0.1f);
        else
            drone->pitch(1.0f, 0.0f, 0.7f);

        break;
    case 'D':
        //tilt right
        if (!drone->isVelMode)
            drone->roll(0.0f, 1.0f, 0.1f);
        else
            drone->pitch(-1.0f, -0.0f, -0.7f);
        break;
    case 'W':
        //title front
        if (!drone->isVelMode)
            drone->pitch(1.0f, 0.0f, 0.1f);
        else
            drone->roll(0.0f, 1.0f, 0.7f);
        break;
    case 'S':
        if (!drone->isVelMode)
            drone->pitch(-1.0f, -0.0f, -0.1f);
        else
            drone->roll(-0.0f, -1.0f, 0.7f);
        break;

    case 'M':
        drone->velMode(!drone->isVelMode);
        break;

    case 'T':
        testPositionControl();
        break;

    default:
        //break;
        if (!drone->isPosctrl)
            drone->hover();
    }
    event->accept();
}

void DialogKeyboard::keyReleaseEvent(QKeyEvent *event)
{
    if (!drone)
        return;
    char key = event->key();
    if (!event->isAutoRepeat())
    {
        std::cout << "key:" << key << " has been released !" << std::endl;
        if (!drone->isPosctrl)
            drone->hover();
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

void DialogKeyboard::testPositionControl()
{
    if (drone->isPosctrl)
    {
        drone->posCtrl(false);
        std::cout << "position control off!" << std::endl;
    }
    else
    {
        drone->posCtrl(true);
        std::cout << "(0.5,-1.5,6)" << std::endl;
        drone->moveTo(dronepose.pose.position.x, dronepose.pose.position.y, dronepose.pose.position.z);
        sleep(5);
        // drone->moveTo(0.5, 1.5, 2);
        // sleep(5);
        // drone->moveTo(-3, 1.5, 2);
        // sleep(5);
        // drone->moveTo(-3, -1.5, 2);
        // sleep(5);
        // drone->moveTo(0.5, -1.5, 2);
        // sleep(5);
    }
}