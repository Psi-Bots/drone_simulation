#ifndef DIALOGKEYBOARD_H
#define DIALOGKEYBOARD_H

#include <QDialog>
#include "drone_object_ros.h"
#include "geometry_msgs/PoseStamped.h"

namespace Ui {
class DialogKeyboard;
}

class DialogKeyboard : public QDialog
{
    Q_OBJECT  
public:
    DroneObjectROS* drone;
public:
    explicit DialogKeyboard(QWidget *parent = 0);
    ~DialogKeyboard();
    void setDrone(DroneObjectROS& drone_){
        this->drone = &drone_;
    }
    
    void keyPressEvent(QKeyEvent *event);
    virtual void keyReleaseEvent(QKeyEvent *event);
    void testPositionControl();
    ros::NodeHandle n;
    ros::Subscriber sub;
    geometry_msgs::PoseStamped dronepose;
    void poseCallback(const geometry_msgs::PoseStamped msg);

private:
    Ui::DialogKeyboard *ui;
};

#endif // DIALOGKEYBOARD_H
