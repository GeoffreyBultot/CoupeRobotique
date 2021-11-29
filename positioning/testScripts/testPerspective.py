import math
import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from cv2 import aruco
import matplotlib.pyplot as plt
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import time 
import os

# 50 images au labo avec une résolution de 1280x960
DIM=(1280, 960)
K=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
D=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]])


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

#--- initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
#camera.rotation = 180
#camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
camera.contrast = 0
camera.image_denoise = True 
#camera.resolution = (1920, 1080)
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)



if __name__ == '__main__':
	timeStart = 0 
	
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100 )
	parameters =  aruco.DetectorParameters_create()
	initUndis()
	for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		t1 = time.time()
		frame = frame_pi.array
		frame = undistort(frame)
		gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
		#ret,gray = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
		res = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		#frame = aruco.drawDetectedMarkers(frame, corners)
		
		if res[1] != None: # if aruco marker detected
			im_src = frame
			im_dst = frame
	
			pts_dst = np.array([[res[0][0][0][0][0], res[0][0][0][0][1]], [res[0][0][0][1][0], res[0][0][0][1][1]], [res[0][0][0][2][0], res[0][0][0][2][1]], [res[0][0][0][3][0], res[0][0][0][3][1]]])
			pts_src = pts_dst
			h, status = cv2.findHomography(pts_src, pts_dst)
			print(h) 
			'''
			[[ 1.00000000e+00 -9.26502817e-15  3.24903775e-12]
			[ 9.33213289e-16  1.00000000e+00  2.21525301e-12]
			[ 2.11548372e-18 -1.60884136e-17  1.00000000e+00]]
			'''
			imgWithAruco = cv2.warpPerspective(im_src, h, (im_dst.shape[1], im_dst.shape[0]))
			markerLength = 10
			rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(res[0], markerLength, K, D)

			imgWithAruco = cv2.aruco.drawAxis(imgWithAruco, K, D, rvec, tvec, 10)
			#cameraPose = cameraPoseFromHomography(h)
			cv2.imshow('No homography',frame)
			cv2.imshow('Measure distance',imgWithAruco)
		rawCapture.truncate(0)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print("fps : "+str(1/(time.time()-t1)))

# Destroy all the windows
cv2.destroyAllWindows()