import math
import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video import FPS
from cv2 import aruco
import matplotlib.pyplot as plt
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import time 
import os

#Labo avec 30 photos 2592 x 1944 le 15/11
DIM=(2592, 1952)
#Camera matrix
K=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
#Distorsion matrix
D=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]])



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
#camera.iso = 1600 # max ISO to force exposure time to minimum to get less motion blur
#camera.contrast = 100
#camera.image_denoise = True 
#camera.resolution = (1920, 1080)
camera.resolution = DIM

camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

#--- start imutils fps counter
fps = FPS().start()

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
	#print(tvec)
	return (inv(matrix) * tvec)

if __name__ == '__main__':
	timeStart = 0 
	
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100 )
	parameters =  aruco.DetectorParameters_create()
	M_Calib = None
	initUndis()
	for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		t1 = time.time()
		frame = frame_pi.array
		frame = undistort(frame)
		gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
		#ret,gray = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
		
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		frame = aruco.drawDetectedMarkers(frame, corners)
		
		if ids is not None:
			fig = plt.figure(figsize=(4,4))
			a1 = fig.add_subplot(111)#, projection='3d')
			markerSizeInCM = 6.7#9.6
			rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, K, D)
			
			for i in range(0,len(ids)):
				if(ids[i][0] == 6):
					M_Calib = computeMcalibMatrix(rvec[i],tvec[i])
					vect = tvec[i]
					vect = (np.reshape(changeReference(vect[0],M_Calib),(1, 4)))[0][0]
					a1.scatter(vect[0,0],vect[0,1],label = str(ids[i][0]))
				else:
					if M_Calib is not None:
						print("id = "+str(ids[i][0]))
						vect = tvec[i]
						#vect = changeReference(tvec[0],M_Calib)
						vect = (np.reshape(changeReference(vect[0],M_Calib),(1, 4)))[0][0]
						print(vect)
						print(vect[0,1])
						a1.scatter(vect[0,0],vect[0,1],label = str(ids[i][0]))
				
				#aruco.drawAxis(frame,K,D, rvec[i], tvec[i], 10)
				#a1.scatter(tvec[i][0][0],tvec[i][0][1],label = str(ids[i][0]))
				#str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0][0][0], tvec[0][0][1], tvec[0][0][2])
				#font = cv2.FONT_HERSHEY_PLAIN
				#print(tvec)
				#cv2.putText(frame, str_position, (0, (i+1)*100), font, 4, (0, 255, 0), 2, cv2.LINE_AA)
			if(len(ids) == 2):
				os.system('clear')
				dx = tvec[0][0][0]-tvec[1][0][0]
				print("dx = "+str(dx))
				dy = tvec[0][0][1]-tvec[1][0][1]
				print("dy = "+str(dy))
				print("dz = "+str(tvec[0][0][2]-tvec[1][0][2]))
				print("distance ="+str(calculateDistance(dx,dy)))
				
				print("rx "+str(ids[i][0])+"= "+str(tvec[0][0][0]-tvec[1][0][0]))
				print("ry"+str(tvec[0][0][1]-tvec[1][0][1]))
				print("rz"+str(tvec[0][0][2]-tvec[1][0][2]))
			else:
				print(len(ids))
		a1.legend()
		a1.set_xlabel("x")
		a1.set_ylabel("y")
		a1.set_xlim(-100,100)
		a1.set_ylim(-100,100)
		plt.show()
		#cv2.imshow('Measure distance',cv2.resize(frame,(960,600)))	
		#cv2.imshow('Measure distance',cv2.resize(frame,(1920,1000)))
		rawCapture.truncate(0)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print("fps : "+str(1/(time.time()-t1)))

# Destroy all the windows
cv2.destroyAllWindows()