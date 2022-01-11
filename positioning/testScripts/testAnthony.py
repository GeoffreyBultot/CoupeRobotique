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

dict_name ={
	1: 'face_gauche',
	2: 'face_avant',
	6: 'face_droite',
	7: 'face_arrière',
	42: 'tag_central', #tag central
	17: 'caillou', #face cachée - all
	47: 'rouge', #face tresor rouge
	13: 'bleu', #face tresor bleu
	36: 'vert', #face tresor vert

}

# 50 images au labo avec une résolution de 1280x960

""" DIM=(1280, 960)
K=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
D=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]]) """


""" DIM=(1280, 960) #NEW CALIB 20/12 by Antho the GOD
K=np.array([[636.2973331204615, 0.0, 728.8166434876724], [0.0, 632.4666494131404, 460.8562085116319], [0.0, 0.0, 1.0]])
D=np.array([[-0.06005220364781312], [0.04832812120892501], [-0.04895075355487911], [0.017239151765319385]]) """

""" DIM=(2592, 1952)
K=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
D=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]]) """



DIM=(1280, 960) #21-12
K=np.array([[612.6769659225217, 0.0, 573.732399040683], [0.0, 614.0511112656127, 464.15063277573313], [0.0, 0.0, 1.0]])
D=np.array([[-0.027945714136134205], [-0.012776430253932694], [0.00586270163443667], [-0.0015790193010345587]])



#C_IP_MQTT = "172.30.40.24"
C_IP_MQTT = "172.30.40.105"


angle = -53
theta = math.radians(angle)
cosT = math.cos(theta)
sinT = math.sin(theta)
#X
rotation_matrix = np.array([[1,0,0],
						[0, cosT, -sinT],
						[0, sinT, cosT]])

camera = PiCamera()
camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
camera.contrast = 0
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

client = mqtt.Client()

tvec_42 = [0,125,100]
rvec_42 = None
matrix_calib = None


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



def mqtt_pubData(ids,tvecs,rvecs):
	userdata = []
	for i in range(0,len(ids)):
		marker = {}
		marker["id"]= int(ids[i][0])
		marker["x"]= int(tvecs[i][0])
		marker["y"]= int(tvecs[i][1])
		marker["z"]= int(tvecs[i][2]) 
		#marker["z"] = int(math.sqrt(abs(int(tvecs[i][2]) * int(tvecs[i][1]))))
		marker["rx"] = int(rvecs[i][0]) % 360
		marker["ry"] = int(rvecs[i][1]) % 360
		marker["rz"] = int(rvecs[i][2]) % 360
		userdata.append(marker)
		payload_json = json.dumps(marker)
		try:
			name = str(dict_name[ids[i][0]])
		except:
			name = "error"
		client.publish("data/"+name, payload=payload_json, qos=0, retain=False)


def computeMcalibMatrix(rvec42,tvec42):
	rodr = cv2.Rodrigues(rvec42)[0]
	#print("Rodrigues = " + str(rodr))
	M_calib    = np.matrix(rodr)
	newrow = [[0,0,0,1]]
	tvecCol = np.reshape(tvec42, (3, 1))
	#print("Reshaped = " + str(tvecCol))
	M_calib=np.c_[M_calib,tvecCol]
	M_calib = np.vstack((M_calib, newrow))
	#print(M_calib)
	return M_calib

def changeReference(tvec,matrix):
	tvec = np.reshape(tvec, (3, 1))
	tvec = np.vstack((tvec,[1]))
	return (inv(matrix) * tvec)

if __name__ == '__main__':
	_thread.start_new_thread( data_Thread, (1 ,) )
	print("[DEBUG	] Thread MQTT Started")
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100 )
	parameters =  aruco.DetectorParameters_create()

	for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		t1 = time.time()
		frame = frame_pi.array
		gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		frame = aruco.drawDetectedMarkers(frame, corners)
		if ids is not None:
			markers_tvec = []
			markers_rvec = []

			for i in range(0,len(ids)):

				if(ids[i][0] in dict_sizes):
					markerSizeInCM = dict_sizes[ids[i][0]]
				else:
					markerSizeInCM = 5
				
				rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], markerSizeInCM, K, D)
				
				#a1.scatter(tvec[0][0][0],tvec[0][0][1],label = str(ids[i][0]*10))
				tvec = np.matmul(rotation_matrix, tvec[0][0])
				rvec =  np.matmul(rotation_matrix, rvec[0][0])

				if(ids[i][0] == 42):
					tvec_42 = tvec
					rvec_42 = rvec
					matrix_calib = computeMcalibMatrix(rvec_42,tvec_42)
					#print(tvec_42)
				if(rvec_42 is not None and matrix_calib is not None ):
					tvec = changeReference(tvec,matrix_calib)

				#tvec = tvec - tvec_42
				rotation,_ = cv2.Rodrigues(rvec)
				rvec = rotationMatrixToEulerAngles(rotation)

				#Ici on a tvec et rvec p/r à la caméra mais sur un plan normal à la table
				markers_tvec.append( [tvec[0],tvec[1],tvec[2]])
				markers_rvec.append( [rvec[0],rvec[1],rvec[2]])
				
			mqtt_pubData(ids,markers_tvec,markers_rvec)

		#cv2.imshow('Measure distance',cv2.resize(frame,(1280, 960)))
		#cv2.imshow('Measure distance',frame)
		rawCapture.truncate(0)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		#print("fps : "+str(1/(time.time()-t1)))
		t1 = time.time()
cv2.destroyAllWindows()