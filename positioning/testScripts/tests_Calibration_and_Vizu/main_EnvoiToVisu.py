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
from mqtt_utils import *

DIM=(1280, 960)

K=np.array([[630.7754887941976, 0.0, 620.9371992476929], [0.0, 629.4950869640792, 465.74381948235816], [0.0, 0.0, 1.0]])
D=np.array([[-0.02525869724179829], [0.01980933465020202], [-0.02957970842677755], [0.014154643445441723]])

#calib parfaite avec newK=K et undistort = remap 
K=np.array([[631.2572612018607, 0.0, 618.467038857025], [0.0, 629.6274581887568, 467.35515194079954], [0.0, 0.0, 1.0]])
D=np.array([[-0.0215299622191804], [-0.014370973740814787], [0.021604009136345702], [-0.009689611617510977]])

#K=np.array([[630.5825314045783, 0.0, 620.8057164012537], [0.0, 629.3053625836126, 465.50088979427255], [0.0, 0.0, 1.0]])
#D=np.array([[-0.023615555373413755], [0.013474224062150016], [-0.018809393732810716], [0.007742965964820537]])




#If on RPI
camera = PiCamera()
#camera.rotation = 180
#camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
#camera.contrast = 0
camera.resolution = DIM
#camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

rotation_z = None
rotation_y = None
rotation_x = None

tvec42=None
rvec42=None

def ComputeRotationOffteur(rvec42):
	global rotation_x
	global rotation_y
	global rotation_z
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

def determineMidpoint(point1,point2):
	x1 = point1[0]
	y1 = point1[1]
	x2 = point2[0]
	y2 = point2[1]
	diff_vect = [point1[0] - point2[0],point1[1] - point2[1]]
	midpoint_X = point1[0]-diff_vect[0]/2
	midpoint_Y = point1[1]-diff_vect[1]/2
	
	angle = math.degrees(math.atan(diff_vect[1]/diff_vect[0]))
	y1 = point1[1] - point2[1]
	x2 = point1[0] - point2[0]
	y2 = point1[1] - point2[1]
	angle = math.degrees(math.atan2(y1, x1) - math.atan2(y2, x2))%360

	return midpoint_X, midpoint_Y, angle

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

lastsTvecs42 = None
lastsRvecs42 = None
map1 = None
map2 = None

def moving_average(x, w):
	return np.convolve(x, np.ones(w), 'valid') / w


def initUndis():
	global map1
	global map2
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

newFrame = None
def ThreadCapture(theadID):
	global newFrame
	for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		newFrame=frame_pi.array
		rawCapture.truncate(0)


if __name__ == '__main__':	
	initClient()
	_thread.start_new_thread(ThreadCapture, (2 ,) )
	#_thread.start_new_thread(ThreadConvertFrame, (2 ,) )
	#new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=1)
	new_K = K
	t1 = time.time()
	initUndis()
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100 )
	parameters =  aruco.DetectorParameters_create()
	M_Calib = None
	t1 = time.time()
	while(True):
		if(newFrame is not None):
			frame = cv2.remap(newFrame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
			newFrame = None
			#cv2.fisheye.undistortImage(frame,K,D,frame,new_K,DIM)
			gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
			frame = aruco.drawDetectedMarkers(frame, corners)
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
							if(M_Calib is None):
								print("compute mcalib")
								M_Calib = computeMcalibMatrix(rvec,tvec)
								rotation,_ = cv2.Rodrigues(rvec)
								rvec = rotationMatrixToEulerAngles(rotation)
							#ComputeRotationOffteur(rvec)
								rvec42 = rvec
								tvec42 = tvec
						temp_tvecs.append(tvec)
						temp_rvecs.append(rvec)
						
				for i in range (0,len(temp_tvecs)):
					tvec = temp_tvecs[i]
					rvec = temp_rvecs[i]
					if(ids[i][0] in dict_sizes):
						if( ids[i][0] != 42 and M_Calib is not None):
							rotation,_ = cv2.Rodrigues(rvec)
							rvec = rotationMatrixToEulerAngles(rotation)
							
						if(M_Calib is not None):
							tvec = changeReference(tvec,M_Calib)
							
						markers_tvec.append( [tvec[0],tvec[1],tvec[2]])
						rvec[2] = int(rvec[2])%360
						markers_rvec.append( [rvec[0],rvec[1], rvec[2]])
						markers_ids.append(ids[i])
				#frame = cv2.resize(frame,(800,720))
				#cv2.imshow("captured",frame)
				
				mqtt_pubData(markers_ids,markers_tvec,markers_rvec)
				#print(1/(time.time()-t1))
				t1 = time.time()
		
		


