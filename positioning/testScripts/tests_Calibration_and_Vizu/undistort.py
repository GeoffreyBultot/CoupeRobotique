
import math
import numpy as np
import cv2
import paho.mqtt.client as mqtt
import _thread
import json
from cv2 import aruco
from pylab import *
import time
import math 
from dictionary import *



DIM=(1280, 960)
K=np.array([[621.1570691293736, 0.0, 600.2355303092088], [0.0, 619.3390978742257, 472.024070607675], [0.0, 0.0, 1.0]])
D=np.array([[-0.010783339031079746], [0.026397062411191077], [-0.050406394553889795], [0.02369514183103941]])	

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
	if not dim2:
		dim2 = dim1
	if not dim3:
		dim3 = dim1
	scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
	scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
	new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
	print(new_K.tolist())
	undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)#, borderMode=cv2.BORDER_CONSTANT)
	return(undistorted_img)


if __name__ == '__main__':
	initUndis()
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100 )
	parameters =  aruco.DetectorParameters_create()
	M_Calib = None
	frame = cv2.imread('image3.jpg') 
	frame = cv2.flip(frame, 1)
	frame = cv2.rotate(frame, cv2.ROTATE_180)
	gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	frame = aruco.drawDetectedMarkers(frame, corners)
	new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=1)
	#cv2.fisheye.undistortImage(frame,K,D,frame,new_K,DIM) #undistort(frame)
	frame = undistort(frame,balance = 0)
	
	cv2.imshow('Distored',cv2.resize(frame,(1080,720)))
	if cv2.waitKey(0) & 0xFF == ord('q'):
		cv2.destroyAllWindows()
		