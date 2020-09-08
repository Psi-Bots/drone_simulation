import numpy as np
import cv2, PIL
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import rospy
import sys
import time
import os
from sensor_msgs.msg import Image
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError
# %matplotlib nbagg


class ArucoDetection:
    
    def __init__(self):
        
        self.index = 0
        self.corners = np.float32([[0, 360], [640, 0], [0, 0], [640, 360]])
        self.bridge = CvBridge()
        self.image_path = "/home/vishwas/drone_ws/src/sjtu-drone/Images/"
        self.front_cam_sub = rospy.Subscriber("/drone/front_camera/image_raw", Image, self.front_cam_callback)
        self.tag_pub = rospy.Publisher("/position", Image, queue_size=5)
        
    def front_cam_callback(self,img):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError, e:
            print e
        
        blur = cv2.medianBlur(cv_image,3,0)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        sobel_image = self.sobel_gradient(img)

        warpedImage, src, dest = self.perspective_transform(blur, self.corners)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        parameters.minDistanceToBorder = 0
        parameters.adaptiveThreshWinSizeMax = 1000  

        corners, ids, rejectedImgPoints = aruco.detectMarkers(blur,aruco_dict,parameters=parameters)
        print('corners', corners)
        print('id', ids)
        print('rej', rejectedImgPoints)
        out = aruco.drawDetectedMarkers(blur, corners, ids)
        # cv2.imwrite('/home/vishwas/drone_ws/src/sjtu-drone/Images/', out)
        # cv2.imwrite(os.path.join(self.image_path, "frame{:06d}.jpg".format(self.index)), out)
        # self.index += 1
        cv2.imshow("out",out)
        cv2.waitKey(2)
        
    def perspective_transform(self, img, corners, xoffset=0):
        
        height, width = img.shape[0:2]
        output_size = height / 2

        new_top_left = np.array([corners[0, 0], 0])
        new_top_right = np.array([corners[3, 0], 0])
        offset = [xoffset, 0]
        img_size = (img.shape[1], img.shape[0])
        src = np.float32([corners[0], corners[1], corners[2], corners[3]])
        dst = np.float32(
            [
                new_top_left + offset,
                corners[3] - offset,
                new_top_right - offset,
                corners[0] + offset,
            ]
        )
        M = cv2.getPerspectiveTransform(src, dst)

        warped = cv2.warpPerspective(img, M, (width, height), flags=cv2.INTER_AREA)
        warped = np.rot90(warped)
        
        cv2.imshow("warped", warped)
        cv2.waitKey(2)
        
        return warped, src, dst
    
    def sobel_gradient(self, img):
        """ Function to calculate Sobel gradient and resize as per 
            required dimensions
            Input:
                img - Image on which gradient has to be calculated
            Output:
                grad - the final image with calculated gradient and also resized"""

        scale = 1
        delta = 0
        ddepth = cv2.CV_16S

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # cv2.imwrite('/home/vishwas/rover/ros2_ws/src/rover_ros2/rover_packages/Images/recorded_images/camera_image.jpeg', cv_image)
        # cv2.imwrite(os.path.join(self.image_path, "frame{:06d}.jpg".format(self.index)), cv_image)#*255)
        # self.index += 1

        blur1 = cv2.medianBlur(cv_image, 3)
        blur = cv2.GaussianBlur(blur1, (3, 3), 0)

        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        grad_x = cv2.Sobel(
            gray,
            ddepth,
            1,
            0,
            ksize=3,
            scale=scale,
            delta=delta,
            borderType=cv2.BORDER_DEFAULT,
        )

        grad_y = cv2.Sobel(
            gray,
            ddepth,
            0,
            1,
            ksize=3,
            scale=scale,
            delta=delta,
            borderType=cv2.BORDER_DEFAULT,
        )

        abs_grad_x = cv2.convertScaleAbs(grad_x)
        abs_grad_y = cv2.convertScaleAbs(grad_y)

        grad = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)

        # grad[grad < 110] = 0
        # grad[grad != 0] = 255

        cv2.imshow("cv_img", grad)
        cv2.waitKey(2)

        return grad
        
        
def main(args):
    arucodetect = ArucoDetection()
    rospy.init_node('arucodetect', anonymous=True)
    assert arucodetect
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()
        
if __name__ == '__main__':
    main(sys.argv)   

