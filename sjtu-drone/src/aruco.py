import cv2
from cv2 import aruco

img = cv2.imread('/home/vishwas/Pictures/aruco4.png')
# img = cv2.imread('/home/vishwas/drone_ws/src/sjtu-drone/Images/frame000585.jpg')

blur = cv2.medianBlur(img,3,0)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# gray.fill(255)
# aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 53
print(parameters)
# Detect the markers.
corners, ids, rejectedImgPoints = aruco.detectMarkers(blur,aruco_dict,parameters=parameters)
print(corners)
out = aruco.drawDetectedMarkers(blur, corners, ids)

cv2.imshow("out",out)
cv2.waitKey(0)
cv2.destroyAllWindows()