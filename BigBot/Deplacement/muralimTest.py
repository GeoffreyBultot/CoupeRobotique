import numpy as np
import cv2
import cv2.aruco as aruco
import time
cap = cv2.VideoCapture(0)
while(True):
    ret,frame = cap.read()
    frame = cv2.flip(frame,0)
    frame = cv2.flip(frame,1)
#    frame = cv2.imread('/home/pi/Documents/ARUCO_Reader/images/aruco.PNG')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=arucoParameters)
    frame = aruco.drawDetectedMarkers(frame, corners,ids)
    cv2.imshow('Display', frame)
 #   time.sleep(5)
    print(ids)


    if np.all(ids != None):
        display = aruco.drawDetectedMarkers(frame, corners)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

