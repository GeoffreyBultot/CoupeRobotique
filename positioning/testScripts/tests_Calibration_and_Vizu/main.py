from email.errors import NonASCIILocalPartDefect
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
from TableDrawer import *

DIM=(1280, 960)
# 50 images au labo avec une résolution de 1280x960 by anthony
#new camera
'''
K=np.array([[621.1570691293736, 0.0, 600.2355303092088], [0.0, 619.3390978742257, 472.024070607675], [0.0, 0.0, 1.0]])
D=np.array([[-0.010783339031079746], [0.026397062411191077], [-0.050406394553889795], [0.02369514183103941]])	
'''
'''
#Ancienne cam qui est mtn sur le robeau
DIM=(1280, 960)
K=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
D=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]]) 
'''

'''
#avec new K from undistort.py
K=np.array([[487.18303262 , 0.0, 541.06467081], [0.0, 487.76569225, 478.33392294], [0.0, 0.0, 1.0]])
D=np.array([[-0.010783339031079746], [0.026397062411191077], [-0.050406394553889795], [0.02369514183103941]])	
'''

'''
#Calib avec le calib "normal"
K=np.array([[1750.8001202391754, 0.0, 644.6374121494749], [0.0, 884.6214678015123, 468.57101078421596], [0.0, 0.0, 1.0]])
D=np.array([[-0.014275121467061143, -0.05830354358680527, 0.1838065048004832, 0.007221670457971407, 0.8453482197585449]])
'''
'''
#calib avec le calib "normal" mais en remettant les images à l'endroit
K=np.array([[851.3267938588147, 0.0, 641.1176655442072], [0.0, 969.9500167329727, 482.1267808171842], [0.0, 0.0, 1.0]])
D=np.array([[-0.24300009784277374, -0.6020231515355897, 0.03615321946513229, 0.0019046706743943956, 1.2595584257574064]])
'''

'''
#calib 16/01 GBU : Python 3.9 avec toutes les images
K=np.array([[623.9007739705611, 0.0, 717.5555620735222], [0.0, 623.538643068371, 451.809666565839], [0.0, 0.0, 1.0]])
D=np.array([[-0.02414595817904007], [0.06546391683919295], [0.04261541000056712], [-0.16448577824189425]])
'''
#calibration utilisée
#NB : 487 meilleurs résultats with undistort + newK = K
DIM=(1280, 960)
K=np.array([[487.18303262 , 0.0, 541.06467081], [0.0, 487.76569225, 478.33392294], [0.0, 0.0, 1.0]])

K=np.array([[621.1570691293736, 0.0, 600.2355303092088], [0.0, 619.3390978742257, 472.024070607675], [0.0, 0.0, 1.0]])
D=np.array([[-0.010783339031079746], [0.026397062411191077], [-0.050406394553889795], [0.02369514183103941]])	

K = np.array([[4.7145831298828125e+02, 0., 6.1314993079996202e+02], [0., 4.8821551513671875e+02, 4.6546089687167841e+02], [0., 0., 1.]])
D=np.array([[-4.1817826199830443e-01], [2.3352366215907835e-01],[2.0186395911750031e-03], [-3.5460784335298407e-04], [-7.6370444111713787e-02]])

rotation_z = None
rotation_y = None
rotation_x = None

tvec42=None
rvec42=None

def ComputeRotationOffteur(rvec42):
	global rotation_x
	global rotation_y
	global rotation_z
	print('rvec42=',rvec42)
	thetaX = math.radians(rvec42[0])
	thetaY = math.radians(rvec42[1])
	thetaZ = math.radians(40)#rvec42[2])
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
new_K = None
def initUndis():
	global map1
	global map2
	global new_K
	new_K =cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3),balance=0)
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, DIM, cv2.CV_16SC2)

def undistort(img, balance=0, dim2=None, dim3=None):
	#img = cv2.resize(img,DIM)
	dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
	assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"	
	undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	return(undistorted_img)

if __name__ == '__main__':
	for imgage in range(1,9):
		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100 )
		parameters =  aruco.DetectorParameters_create()
		M_Calib = None
		str = f'image{imgage}.jpg'
		frame = cv2.imread(str) 
		frame = cv2.flip(frame, 1)
		frame = cv2.rotate(frame, cv2.ROTATE_180)
		image_base = frame
		#initUndis()
		#new_K = K
		#frame = cv2.undistort(frame,K,D)
		frame = undistort(frame)
		#cv2.fisheye.undistortImage(frame,K,D,frame,new_K,DIM) #undistort(frame)
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
					frame = cv2.aruco.drawAxis(frame,new_K, D, rvec, tvec, 10)
					tvec = (tvec[0][0])
					rvec =  (rvec[0][0])
					
					if(ids[i][0]==42):
						
						M_Calib = computeMcalibMatrix(rvec,tvec)
						rotation,_ = cv2.Rodrigues(rvec)
						rvec = rotationMatrixToEulerAngles(rotation)
						ComputeRotationOffteur(rvec)
						rvec42 = rvec
						tvec42 = tvec
					temp_tvecs.append(tvec)
					temp_rvecs.append(rvec)
					
			for i in range (0,len(temp_tvecs)):
				tvec = temp_tvecs[i]
				rvec = temp_rvecs[i]
				if( ids[i][0] != 42):
					rotation,_ = cv2.Rodrigues(rvec)
					rvec = rotationMatrixToEulerAngles(rotation)
					rvec =  np.matmul(rotation_x, rvec)
					rvec =  np.matmul(rotation_y, rvec)
					rvec =  np.matmul(rotation_z, rvec)
					#rvec = changeReference(rvec,M_Calib)
				'''
				tvec =  np.matmul(rotation_x, tvec)
				tvec =  np.matmul(rotation_y, tvec)
				tvec =  np.matmul(rotation_z, tvec)
				'''
				tvec = changeReference(tvec,M_Calib)
				markers_tvec.append( [tvec[0],tvec[1],tvec[2]])
				markers_rvec.append( [rvec[0],rvec[1],rvec[2]])
				markers_ids.append(ids[i])
					#print(ids[i][0],tvec)
			#cv2.imshow('NotDistored', cv2.resize(frame,(1080,720)))
			frame = cv2.resize(frame,(800,720))
			plotInTable = drawInTableJeu(markers_ids,markers_tvec,markers_rvec)
			#cv2.imshow('ImageDistored',plotInTable)
			plotInTable = cv2.resize(plotInTable,(800,720))
			numpy_horizontal_concat = np.concatenate((frame,plotInTable), axis=1)
			#cv2.imshow('ImageDistored',frame)
			cv2.imshow('NotDistored',numpy_horizontal_concat)
			cv2.imwrite(f'./Points_IN_TABLE/numpy_horizontal_concat{imgage}.jpg',numpy_horizontal_concat)
			#cv2.imshow("drawInTable",drawInTableJeu(markers_ids,markers_tvec))
			if cv2.waitKey(0) & 0xFF == ord('q'):
				cv2.destroyAllWindows()
		