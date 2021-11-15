import paho.mqtt.client as mqtt
import _thread
import numpy as np
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import math  
import json
import time 

C_t_refresh_Seconds = 1

#Avec photos dans chambre
DIM=(1920, 1080)
K=np.array([[920.5081694823547, 0.0, 843.4705819715025], [0.0, 924.0258507534027, 524.297176724678], [0.0, 0.0, 1.0]])
D=np.array([[0.004350222775772691], [-0.3280398581618519], [1.0520878689147095], [-1.0998229415104885]])

#labeau
DIM=(3280, 2464)
K=np.array([[1476.819868473153, 0.0, 1588.1346039798418], [0.0, 1470.3333122880208, 1214.1943439962924], [0.0, 0.0, 1.0]])
D=np.array([[0.09181633028099863], [-0.456540060140012], [0.6964594451322582], [-0.35862275918235986]])


#distortion_coeff = np.array([  0.97042383,   2.60428972,   0.35928837,   0.12724034, -41.59389042])
camera_matrix = np.array([[639.87721705,   0.        , 330.12073612],
						[  0.        , 643.69687408, 208.61588364],
						[  0.        ,   0.        ,   1.        ]])
distortion_coeff = np.array([ 5.66942769e-02, -6.05774927e-01, -7.42066667e-03, -3.09571466e-04, 1.92386974e+00])


#camera_matrix = np.array([[309.65140551, 0, 299.7942552], [0, 309.63299386, 236.80161718], [ 0, 0, 1]])
#distortion_coeff = np.array([-0.32061628, 0.13711123, 0.0058947, 0.00258218, -0.03117783])

#rpiCam
camera_matrix = np.array([[639.87721705,   0.        , 330.12073612],
						[  0.        , 643.69687408, 208.61588364],
						[  0.        ,   0.        ,   1.        ]])
distortion_coeff = np.array([ 5.66942769e-02, -6.05774927e-01, -7.42066667e-03, -3.09571466e-04, 1.92386974e+00])


def calculateDistance(x1,y1,x2,y2):  
	dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
	return dist  

# define a video capture object
marker_size = 2
pos_42 = 0
x_42 = 0
y_42 = 0
if __name__ == '__main__':
	timeStart = 0
	while(True):
		Dist = []
		pathent = "./images/foo.jpg"
		#pathent = "./images/markers.jpg"
		frame = cv2.imread(pathent)#vid.read(0)
		#frame=cv2.rotate(frame, cv2.ROTATE_180)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)#DICT_4X4_250
		parameters =  aruco.DetectorParameters_create()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		#corners, ids, rejected = aruco.detectMarkers(frame , aruco_dict)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		frame = aruco.drawDetectedMarkers(frame.copy(), corners)
		timeNow = time.time()
		if ids is not None:
			if timeNow - timeStart > C_t_refresh_Seconds:
				timeStart = timeNow
				markerSizeInCM = 2
				rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, camera_matrix, distortion_coeff)
				print("rvecs")
				print(rvec)
				for j in range(0,len(ids)):
					aruco.drawAxis(frame, camera_matrix, distortion_coeff, rvec[j], tvec[j], 1)
				#frame = aruco.drawAxis(frame, camera_matrix, distortion_coeff, rvec, tvec, 1)  # Draw axis
				#frame = cv2.resize(frame, ( 960,720))
				cv2.imshow('blop',frame)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break
# Destroy all the windows
cv2.destroyAllWindows()