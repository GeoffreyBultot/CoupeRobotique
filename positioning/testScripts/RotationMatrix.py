from glob import glob
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
import socket

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
#new camera
DIM=(1280, 960)
K=np.array([[621.1570691293736, 0.0, 600.2355303092088], [0.0, 619.3390978742257, 472.024070607675], [0.0, 0.0, 1.0]])
D=np.array([[-0.010783339031079746], [0.026397062411191077], [-0.050406394553889795], [0.02369514183103941]])

""" DIM=(1280, 960)
K=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
D=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]]) """

'''
DIM=(1280, 960) #NEW CALIB 20/12 by Antho the GOD
K=np.array([[636.2973331204615, 0.0, 728.8166434876724], [0.0, 632.4666494131404, 460.8562085116319], [0.0, 0.0, 1.0]])
D=np.array([[-0.06005220364781312], [0.04832812120892501], [-0.04895075355487911], [0.017239151765319385]])
'''
""" DIM=(2592, 1952)
K=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
D=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]]) """



#C_IP_MQTT = "172.30.40.24"
#C_IP_MQTT = "172.30.40.105"
#C_IP_MQTT = commands.getoutput('hostname -I')
C_IP_MQTT = "raspilocalization.lan"#socket.gethostbyname(socket.gethostname())

rotation_y = None
rotation_x = None

tvec42=None
rvec42=None

def ComputeRotationOffteur(rvec42):
	global rotation_x
	global rotation_y
	thetaX = rvec42[0]
	thetaY = rvec42[1]
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


angle = -130
theta = math.radians(angle)
cosT = math.cos(theta)
sinT = math.sin(theta)
#X
rotation_x = np.array([[1,0,0],
						[0, cosT, -sinT],
						[0, sinT, cosT]])
rotation_y = np.array([[cosT,0,sinT],
						[0, 1, 0],
						[-sinT, 0, cosT]])
rotation_z = np.array([[cosT,-sinT,0],
						[sinT, cosT, 0],
						[0, 0, 1]])

camera = PiCamera()
#camera.rotation = 180
camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
camera.contrast = 0
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

client = mqtt.Client()




def isMineTag(tagid,tvec):
	if(tagid in dict_sizes):
		if(tagid == 42):
			return True
		elif(tvec42 is not None):
			if(tvec[0] < tvec42[0]): #trop a gauche #TODO faire pour le cote droit
				return False
		else:
			return True
	else:
		return False

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
		marker["y"]= -int(tvecs[i][1]) #?? because why the fuck not
		marker["z"] = int(tvecs[i][2])
		marker["rx"] = int(rvecs[i][0])
		marker["ry"] = int(rvecs[i][1])
		marker["rz"] = int(rvecs[i][2])
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
			markers_ids = []
			#plt.draw()
			#a1.cla()  
			temp_tvecs = []
			temp_rvecs = []
			for i in range(0,len(ids)):

				if(ids[i][0] in dict_sizes):
					markerSizeInCM = dict_sizes[ids[i][0]]
					rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], markerSizeInCM, K, D)
					tvec = (tvec[0][0])
					rvec =  (rvec[0][0])
					if(ids[i][0]==42):
						#M_Calib = computeMcalibMatrix(rvec,tvec)
						rvec42 = tvec
						tvec42 = rvec
					temp_tvecs.append(tvec)
					temp_rvecs.append(rvec)
				#print(ids[i][0])
				#print(rvec)
				#print(tvec)
			#try:
			for i in range (0,len(temp_tvecs)):
				tvec = temp_tvecs[i]
				rvec = temp_rvecs[i]
				if( isMineTag(ids[i][0],tvec)):
					#print("tag"+str(ids[i][0])+"ok")
					#print(M_Calib)
					#tvec = changeReference(tvec,M_Calib)
					tvec =  np.matmul(rotation_x, tvec)
					markers_tvec.append( [tvec[0],tvec[1],tvec[2]])
					rvec =  np.matmul(rotation_x, rvec)
					rotation,_ = cv2.Rodrigues(rvec)
					rvec = rotationMatrixToEulerAngles(rotation)
					markers_rvec.append( [rvec[0],rvec[1],rvec[2]])
					markers_ids.append(ids[i])
			#print(markers_ids)
			#print(markers_rvec)
			#print(markers_tvec)
			if(len(markers_ids) > 0):
				print("pub")
				mqtt_pubData(markers_ids,markers_tvec,markers_rvec)
			#except:
			#	pass
		#cv2.imshow('Measure distance',cv2.resize(frame,(1280, 960)))
		cv2.imshow('Measure distance', undistort(frame))
		cv2.imshow('NotDistored', frame)
		rawCapture.truncate(0)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print("fps : "+str(1/(time.time()-t1)))
		t1 = time.time()
cv2.destroyAllWindows()