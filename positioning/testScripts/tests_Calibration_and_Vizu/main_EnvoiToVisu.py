import math
import numpy as np
import cv2
import paho.mqtt.client as mqtt
import _thread
import json
from picamera.array import PiRGBArray
from picamera import PiCamera
import threading
from cv2 import aruco
from pylab import *
import time
import math 
from dictionary import *
from TableDrawer import *

DIM=(1280, 960)
K=np.array([[630.7754887941976, 0.0, 620.9371992476929], [0.0, 629.4950869640792, 465.74381948235816], [0.0, 0.0, 1.0]])
D=np.array([[-0.02525869724179829], [0.01980933465020202], [-0.02957970842677755], [0.014154643445441723]])


C_IP_MQTT = "172.30.40.65"

def on_connect(client, userdata, flags, rc):
	print("Connected with result code ")

#If on RPI
camera = PiCamera()
#camera.rotation = 180
#camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
#camera.contrast = 0
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

client = mqtt.Client()
client.on_connect = on_connect

def data_Thread(theadID):
	while True:
		print('[DEBUG	] Connecting to the TTN Broker...')
		#client.connect("192.168.0.13", 1883, 60)
		print(client.connect(C_IP_MQTT, 1883, 60))
		client.loop_forever()

def mqtt_pubData(ids,tvecs,rvecs):
	userdata = []

	for k in dict_sizes:
		j=0
		for i in range(0,len(ids)):
			if(ids[i][0] == k):
				j = j + 1
				marker = {}
				marker["id"]= int(ids[i][0])
				marker["x"]= int(tvecs[i][0])
				marker["y"]= int(tvecs[i][1]) #?? because why the fuck not
				marker["z"] = int(tvecs[i][2])
				marker["rz"] = int(rvecs[i][2])
				userdata.append(marker)
				payload_json = json.dumps(marker)
				client.publish(f"data/{ids[i][0]}_{j}", payload=payload_json, qos=0, retain=False)

rotation_z = None
rotation_y = None
rotation_x = None

tvec42=None
rvec42=None

def ComputeRotationOffteur(rvec42):
	global rotation_x
	global rotation_y
	global rotation_z
	#print('rvec42=',rvec42)
	thetaX = math.radians(rvec42[0])
	thetaY = math.radians(rvec42[1])
	thetaZ = math.radians(rvec42[2])
	cosT = math.cos(thetaX)
	sinT = math.sin(thetaX)
	rotation_x = np.array([[1,0,0],
						[0, cosT, -sinT],
						[0, sinT, cosT]])
	cosT = math.cos(thetaY)
	sinT = math.sin(thetaY)
	rotation_y = np.array([[cosT,0,sinT],
							[0, 1, 0],
							[-sinT, 0, cosT]])
	cosT = math.cos(thetaZ)
	sinT = math.sin(thetaZ)
	rotation_z = np.array([[cosT,-sinT,0],
							[sinT, cosT, 0],
							[0, 0, 1]])

def rotationMatrixToEulerAngles(R) :
	sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
	singular = sy < 1e-6
	if  not singular :
		x = math.atan2(R[2,1] , R[2,2])
		y = math.atan2(-R[2,0], sy)
		z = math.atan2(R[1,0], R[0,0])
	else :
		x = math.atan2(-R[1,2], R[1,1])
		y = math.atan2(-R[2,0], sy)
		z = 0
	return np.array([math.degrees(x), math.degrees(y), math.degrees(z)])

def computeMcalibMatrix(rvec42,tvec42):
	M_calib    = np.matrix(cv2.Rodrigues(rvec42)[0])
	newrow = [[0,0,0,1]]
	tvecCol = np.reshape(tvec42, (3, 1))
	M_calib=np.c_[M_calib,tvecCol]
	M_calib = np.vstack((M_calib, newrow))
	return M_calib

def changeReference(tvec,matrix):
	tvec = np.reshape(tvec, (3, 1))
	tvec = np.vstack((tvec,[1]))
	return (inv(matrix) * tvec)


map1 = None
map2 = None

def initUndis():
	global map1
	global map2
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(img, balance=0, dim2=None, dim3=None):
	#img = cv2.resize(img,DIM)
	dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
	assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
	undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	return(undistorted_img)

if __name__ == '__main__':	
	_thread.start_new_thread( data_Thread, (1 ,) )
	print("[DEBUG	] Thread MQTT Started")
	new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=0)
	#new_K = K
	t1 = time.time()
	initUndis()
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100 )
	parameters =  aruco.DetectorParameters_create()
	M_Calib = None
	for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		frame=frame_pi.array
		image_base = frame
		frame = undistort(frame)
		#cv2.fisheye.undistortImage(frame,K,D,frame,new_K,DIM)
		gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		#frame = aruco.drawDetectedMarkers(frame, corners)
		if ids is not None:
			markers_tvec = []
			markers_rvec = []
			markers_ids = []
			temp_tvecs = []
			temp_rvecs = []
			for i in range(0,len(ids)):
				
				if(ids[i][0] in dict_sizes):
					markerSizeInCM = dict_sizes[ids[i][0]]
					rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], markerSizeInCM, new_K, D)
					#frame = cv2.aruco.drawAxis(frame,new_K, D, rvec, tvec, 10)
					tvec = (tvec[0][0])
					rvec =  (rvec[0][0])
					
					if(ids[i][0]==42):
						M_Calib = computeMcalibMatrix(rvec,tvec)
						rotation,_ = cv2.Rodrigues(rvec)
						rvec = rotationMatrixToEulerAngles(rotation)
						ComputeRotationOffteur(rvec)
						rvec42 = rvec
						print(rvec)
						tvec42 = tvec
					temp_tvecs.append(tvec)
					temp_rvecs.append(rvec)
					
			for i in range (0,len(temp_tvecs)):
				tvec = temp_tvecs[i]
				rvec = temp_rvecs[i]
				if( ids[i][0] != 42 and rotation_x is not None):
					rotation,_ = cv2.Rodrigues(rvec)
					rvec = rotationMatrixToEulerAngles(rotation)
					'''
					rvec =  np.matmul(rotation_x, rvec)
					rvec =  np.matmul(rotation_y, rvec)
					rvec =  np.matmul(rotation_z, rvec)
					rvec = changeReference(rvec,M_Calib)
					'''
					
				if(rotation_x is not None):
					'''
					tvec =  np.matmul(rotation_x, tvec)
					tvec =  np.matmul(rotation_y, tvec)
					tvec =  np.matmul(rotation_z, tvec)
					'''
					tvec = changeReference(tvec,M_Calib)
				markers_tvec.append( [tvec[0],tvec[1],tvec[2]])
				markers_rvec.append( [rvec[0],rvec[1],rvec[2]])
				markers_ids.append(ids[i])
			#frame = cv2.resize(frame,(800,720))
			#cv2.imshow("captured",frame)
			'''
			if cv2.waitKey(1) & 0xFF == ord('q'):
				cv2.destroyAllWindows()
			'''
			
			mqtt_pubData(markers_ids,markers_tvec,markers_rvec)
		rawCapture.truncate(0)
		