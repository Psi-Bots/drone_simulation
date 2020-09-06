import rospy
import sys
import time
from sensor_msgs.msg import Imu
import cv2
from cv2 import aruco

class Drone:
    
    def __init__(self):
        
        self.i = 0
        
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.position_pub = rospy.Publisher("/position", Imu, queue_size=5)
        
    def imu_callback(self,data):
        print
        
def main(args):
    
    rospy.init_node(Drone, anonymous=True)
    drone = Drone()
    assert drone
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        
if __name__ == '__main__':
    main(sys.argv)   
        