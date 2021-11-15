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

C_t_refresh_Seconds = 0.00
client = mqtt.Client()

#Labo avec 30 photos 2592 x 1944 le 15/11
DIM=(2592, 1944)
K=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
D=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]])

def data_Thread(threadID,name,counter):
	while True:
		print('[DEBUG	] Connecting to the TTN Broker...')
		client.connect("192.168.0.13", 1883, 60)
		#client.connect("172.30.40.67", 1883, 60)
		client.loop_forever()

def intUndis():
	global map1
	global map2
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(img, balance=0, dim2=None, dim3=None):
	gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
	dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
	assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
	if not dim2:
		dim2 = dim1
	if not dim3:
		dim3 = dim1
	scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
	scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
	new_K = None #cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
	undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)#, borderMode=cv2.BORDER_CONSTANT)
	return(undistorted_img)
	cv2.imshow("undistorted", cv2.resize(undistorted_img, (960, 540)))
			
vid = cv2.VideoCapture(0) # define a video capture object


def change_res(width, height):
    vid.set(3, width)
    vid.set(4, height)

if __name__ == '__main__':
	#_thread.start_new_thread( data_Thread, (1,"Thread-1", 2) )
	#print("[DEBUG	] Thread Started")
	timeStart = 0
	change_res(1280, 960)
	#change_res(2592, 1944) #https://www.codingforentrepreneurs.com/blog/open-cv-python-change-video-resolution-or-scale
	#change_res(1920, 1080) #https://www.codingforentrepreneurs.com/blog/open-cv-python-change-video-resolution-or-scale
	#intUndis()
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250 )#aruco.DICT_ARUCO_ORIGINAL)
	parameters =  aruco.DetectorParameters_create()
	while(True):
		Dist = []
		t1 = time.time()
		#pathent = "./images/calib0.png"
		pathent = "./images/Labeau/test7.png"
		#frame = cv2.imread(pathent)#vid.read(0)
		ret, frame = vid.read()

		#frame=cv2.rotate(frame, cv2.ROTATE_180)
		gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
		frame = aruco.drawDetectedMarkers(frame, corners)
		if ids is not None:
			for i in range(0, len(ids)):
				line_thickness = 1
				if(ids[i][0] == 42):
					corner_42 = corners[i][0]
					M = cv2.moments(corner_42)
					x_sum = int(M["m10"] / M["m00"])
					y_sum = int(M["m01"] / M["m00"])
					frame = cv2.circle(frame, (int(x_sum),int(y_sum)), radius=5, color=(0, 0, 255), thickness=2)
					corners_abcd = corner_42.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corners_abcd
					#topleft premier puis topright puis bottomright puis bottomleft
					mlist = []
					plist = []
					offsetX = 100#9.5*calculateDistance(corners_abcd[1][0],corners_abcd[1][1],corners_abcd[2][0],corners_abcd[2][1])
					offsetY = 50#14.5*calculateDistance(corners_abcd[0][0],corners_abcd[0][1],corners_abcd[1][0],corners_abcd[1][1])
					
					'''
					for i in range(0,4):
						mlist.append((corners_abcd[(i+1)%4][1]-corners_abcd[i][1])/(corners_abcd[(i+1)%4][0]-corners_abcd[i][0])) #x
						plist.append(corners_abcd[(i+1)%4][1]-mlist[i]*corners_abcd[(i+1)%4][0])
					try:
						if not ((np.NaN in mlist) or (np.NaN in plist)) :
							cv2.line(frame,(0,int(mlist[0]+plist[0]-offsetY)),(500,int(mlist[0]*500+plist[0]-offsetY)),(255,0,0),thickness=2)
							cv2.line(frame,(0,int(mlist[2]+plist[2]+offsetY)),(500,int(mlist[2]*500+plist[2]+offsetY)),(0,255,0),thickness=2)
							cv2.line(frame,(0,int(mlist[3]*(0+offsetX)+plist[3])),(500,int(mlist[3]*(500+offsetX) +plist[3])),(0,0,255),thickness=2)
							cv2.line(frame,(0,int(mlist[1]*(0-offsetX)+plist[1])),(500,int(mlist[1]*(500-offsetX) +plist[1])),(255,255,0),thickness=2)
					except:
						print("nan case")
					'''
					# Computing rotation matrix
					rotation_matrix = np.zeros(shape=(3,3))
					cv2.Rodrigues(rvec[i], rotation_matrix)
					#Apply rotation matrix to point
					original_point = tvec[i] #np.matrix([[1],[0],[0]])
					rotated_point = rotation_matrix*original_point
					
		timeNow = time.time()
		if ids is not None:
			if True : #timeNow - timeStart > C_t_refresh_Seconds:
				userdata = []
				for i in range(0,len(ids)):
					marker = {}
					marker["id"]= int(ids[i][0])
					marker["x"]= int(corners[i][0][0][0])
					marker["y"]= int(corners[i][0][0][1])
					markerSizeInCM = 7
					rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, K, D)
					aruco.drawAxis(frame, K, D, rvec[i], tvec[i], 20)
					userdata.append(marker)
				print(ids)
				print(tvec)
				print(rvec)
				
				timeStart = timeNow
				payload_json = json.dumps(userdata)
				
				
				#client.publish("data", payload=payload_json, qos=0, retain=False)
		#ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1:], None, None)
		#dst = cv2.undistort(frame,camera_matrix, distortion_coeff,None,camera_matrix)
		#frame = cv2.resize(frame, (960, 540))
		#frame = cv2.imread("1")
		cv2.imshow("lala",cv2.resize(frame,(720,480)))
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print("fps " + str(1/(time.time()-t1)))
		
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()