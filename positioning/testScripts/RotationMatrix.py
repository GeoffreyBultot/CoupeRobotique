import math
import numpy as np
import cv2
import paho.mqtt.client as mqtt
import _thread
import json
from picamera.array import PiRGBArray
from picamera import PiCamera
from cv2 import aruco
import matplotlib.pyplot as plt
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import time
import math 
import os


#N_tag = size
dict_sizes = {
  1: 7,
  2: 7,
  6: 7,
  7: 7,
  42: 10, #tag central
  17: 5, #face cachée - all
  47: 5, #face tresor rouge
  13: 5, #face tresor bleu
  36: 5, #face tresor vert
}

# 50 images au labo avec une résolution de 1280x960

DIM=(1280, 960)
K=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
D=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]])

''' 	
DIM=(2592, 1952)
K=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
D=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]])
'''


#C_IP_MQTT = "172.30.40.24"
C_IP_MQTT = "172.30.40.21"

angle = -50
theta = math.radians(angle)
cosT = math.cos(theta)
sinT = math.sin(theta)
#X
rotation_matrix = np.array([[1,0,0],
						[0, cosT, -sinT],
						[0, sinT, cosT]])


camera = PiCamera()
#camera.rotation = 180
#camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
camera.contrast = 0
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

client = mqtt.Client()



def initUndis():
	global map1
	global map2
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(img, balance=0, dim2=None, dim3=None):
	#img = cv2.resize(img,DIM)
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

def calculateDistance(dx,dy):  
	dist = math.sqrt((dx)**2 + (dy)**2)  
	return dist

def rotationMatrixToEulerAngles(R) :
	#assert(isRotationMatrix(R))
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



def data_Thread(theadID):
	while True:
		print('[DEBUG	] Connecting to the TTN Broker...')
		#client.connect("192.168.0.13", 1883, 60)
		print(client.connect(C_IP_MQTT, 1883, 60))
		client.loop_forever()



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

def mqtt_pubData(ids,tvecs,rvecs):
	userdata = []
	for i in range(0,len(ids)):
		marker = {}
		marker["id"]= int(ids[i][0])
		marker["x"]= int(tvecs[i][0])
		marker["y"]= int(tvecs[i][1])
		marker["rz"] = abs(int(rvecs[i][2]))
		userdata.append(marker)
		payload_json = json.dumps(marker)
		client.publish("data/"+str(ids[i][0]), payload=payload_json, qos=0, retain=False)

if __name__ == '__main__':
	_thread.start_new_thread( data_Thread, (1 ,) )
	print("[DEBUG	] Thread MQTT Started")
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100 )
	parameters =  aruco.DetectorParameters_create()
	M_Calib = None
	initUndis()
	'''
	fig = plt.figure(figsize=(4,4))
	a1 = fig.add_subplot(111)#, projection='3d')
	'''
	'''
	marker = {}
	#marker["id"]= int(ids[i][0])
	marker["x"]= 0
	marker["y"]= 100
	marker["rz"] = 90
	payload_json = json.dumps(marker)
	client.publish("data/togo", payload=payload_json, qos=0, retain=False)
	'''
	for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		t1 = time.time()
		frame = frame_pi.array
		gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
		#ret,gray = cv2.threshold(gray,200,255,cv2.THRESH_BINARY) #TODO update the treshold
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		frame = aruco.drawDetectedMarkers(frame, corners)
		if ids is not None:
			markers_tvec = []
			markers_rvec = []
			#plt.draw()
			#a1.cla()  
			for i in range(0,len(ids)):

				if(ids[i][0] in dict_sizes):
					markerSizeInCM = dict_sizes[ids[i][0]]
				else:
					markerSizeInCM = 5
				
				rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], markerSizeInCM, K, D)
				
				#a1.scatter(tvec[0][0][0],tvec[0][0][1],label = str(ids[i][0]*10))
				tvec = np.matmul(rotation_matrix, tvec[0][0])
				rvec =  np.matmul(rotation_matrix, rvec[0][0])
				
				rotation,_ = cv2.Rodrigues(rvec)
				rvec = rotationMatrixToEulerAngles(rotation)
				#print(ids[i][0])
				#print(rvec)
				#print(tvec)
				#Ici on a tvec et rvec p/r à la caméra mais sur un plan normal à la table
				markers_tvec.append( [tvec[0],tvec[1],tvec[2]])
				markers_rvec.append( [rvec[0],rvec[1],rvec[2]])
				#a1.scatter(tvec[0],tvec[1],label = str(ids[i][0]))
				if(ids[i][0] == 42):
					pass
					#print(tvec)
			mqtt_pubData(ids,markers_tvec,markers_rvec)
				

			'''
			a1.legend()
			a1.set_xlabel("x")
			a1.set_ylabel("y")
			a1.set_xlim(0,100)
			a1.set_ylim(0,100)
			plt.ion()
			plt.show()
			plt.pause(0.001)
			'''
			'''
			if(len(ids) == 2 and len(markers_tvec) == 2 ):
				os.system('clear')
				dx = markers_tvec[0][0]-markers_tvec[1][0]
				print("dx = "+str(dx))
				dy = markers_tvec[0][1]-markers_tvec[1][1]
				print("dy = "+str(dy))
				dz = markers_tvec[0][1]-markers_tvec[1][1]
				print("dz = "+str(dz))
				print("distance ="+str(calculateDistance(dx,dy)))
				print(ids[0][0],ids[1][0])
				print(markers_rvec[0][2] , markers_rvec[1][2])
				drz = markers_rvec[0][2]-markers_rvec[1][2]
				print("diff angle = " + str(drz))
			else:
				print(len(ids))
			'''

		#cv2.imshow('Measure distance',cv2.resize(frame,(1280, 960)))
		#cv2.imshow('Measure distance',frame)
		rawCapture.truncate(0)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print("fps : "+str(1/(time.time()-t1)))
		t1 = time.time()
cv2.destroyAllWindows()