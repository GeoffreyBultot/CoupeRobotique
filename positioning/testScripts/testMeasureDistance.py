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

'''
#Labo avec 30 photos 2592 x 1944 le 15/11
DIM=(2592, 1952)
#Camera matrix
K=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
#Distorsion matrix
D=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]])
'''

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
#camera.contrast = 100
camera.image_denoise = True 
#camera.resolution = (1920, 1080)
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

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
		#frame = undistort(frame)
		#blur = cv2.blur(frame,(2,2),0)
		#blur = cv2.GaussianBlur(frame,(3,3),cv2.BORDER_DEFAULT)
		
		gray = frame # cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
		
		#ret,gray = cv2.threshold(gray,100,255,cv2.THRESH_BINARY) #TODO update the treshold
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		frame = aruco.drawDetectedMarkers(frame, corners)
		#frame = aruco.drawDetectedMarkers(frame, rejectedImgPoints)
		
		if ids is not None:
			#fig = plt.figure(figsize=(4,4))
			#a1 = fig.add_subplot(111)#, projection='3d')
			#markerSizeInCM = 7#9.6
			#rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, K, D)
			markers_tvec = []
			markers_rvec = []
			#markers_tvec = np.array([])
			#markers_rvec = np.array([])
			for i in range(0,len(ids)):
				corner = corners[i]
				if(ids[i][0] == 42):
					markerSizeInCM = 10
				else:
					markerSizeInCM = 6.7
				
				rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corner, markerSizeInCM, K, D)
				
				#markers_tvec.append( [tvec[0][0][0],tvec[0][0][1],tvec[0][0][2]])
				#markers_rvec.append( [rvec[0][0][0],tvec[0][0][1],tvec[0][0][2]])
				
				#np.insert(markers_rvec,rvec,axis = 0)
				#np.append([markers_tvec,tvec],axis = 0)
				#a1.scatter(vect[0,0],vect[0,1],label = str(ids[i][0]))
				
				if(ids[i][0] == 42):
					M_Calib = computeMcalibMatrix(rvec,tvec)
					nVec = changeReference(tvec,M_Calib)
					#tvec = (np.reshape(changeReference(tvec,M_Calib),(1, 4)))
					print(nVec)
					markers_tvec.append( [nVec[0],nVec[1],nVec[2]])
					markers_rvec.append( [rvec[0][0][0],rvec[0][0][1],rvec[0][0][2]])
				else:
					if M_Calib is not None:
						print("id = "+str(ids[i][0]))
						#vect = tvec[i]
						nVec = changeReference(tvec,M_Calib)
						markers_tvec.append( [nVec[0],nVec[1],nVec[2]])
						markers_rvec.append( [rvec[0][0][0],rvec[0][0][1],rvec[0][0][2]])
				
						#vect = (np.reshape(changeReference(vect[0],M_Calib),(1, 4)))[0][0]
						#print(vect)
						#print(vect[0,1])
						#a1.scatter(vect[0,0],vect[0,1],label = str(ids[i][0]))

				#aruco.drawAxis(frame,K,D, rvec, tvec, 10)
				#cv2.putText(frame, str_position, (0, (i+1)*100), font, 4, (0, 255, 0), 2, cv2.LINE_AA)
			
			if(len(ids) == 2):
				os.system('clear')
				#d = ((x2 - x1)2 + (y2 - y1)2 + (z2 - z1)2)1/2    
				print(markers_tvec[0][0])
				print(markers_tvec[1][0])
				dx = markers_tvec[0][0]-markers_tvec[1][0]
				print("dx = "+str(dx))
				dy = markers_tvec[0][1]-markers_tvec[1][1]
				print("dy = "+str(dy))
				dz = markers_tvec[0][1]-markers_tvec[1][1]
				print("dz = "+str(dz))
				print("distance ="+str(calculateDistance(dx,dy)))
				'''
				print("rx "+str(ids[i][0])+"= "+str(tvec[0][0][0]-tvec[1][0][0]))
				print("ry"+str(tvec[0][0][1]-tvec[1][0][1]))
				print("rz"+str(tvec[0][0][2]-tvec[1][0][2]))
				'''
			else:
				print(len(ids))
			'''
			a1.legend()
			a1.set_xlabel("x")
			a1.set_ylabel("y")
			a1.set_xlim(-100,100)
			a1.set_ylim(-100,100)
			#plt.show()
			'''
		#cv2.imshow('Measure distance',cv2.resize(frame,(960,600)))	
		#cv2.imshow('Measure distance',cv2.resize(frame,(1920,1000)))
		cv2.imshow('Measure distance',frame)
		#cv2.imshow('blured',blur)
		rawCapture.truncate(0)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print("fps : "+str(1/(time.time()-t1)))

# Destroy all the windows
cv2.destroyAllWindows()