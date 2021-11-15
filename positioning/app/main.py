import paho.mqtt.client as mqtt
import _thread
import numpy as np
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import math  
import json
import time 

C_t_refresh_Seconds = 0.00
client = mqtt.Client()

#Labo avec 30 photos 2592 x 1944 le 15/11
DIM=(2592, 1944)
#Camera matrix
K=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
#Distorsion matrix
D=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]])


def data_Thread(theadID):
	while True:
		print('[DEBUG	] Connecting to the TTN Broker...')
		client.connect("192.168.0.13", 1883, 60)
		#client.connect("172.30.40.67", 1883, 60)
		client.loop_forever()

vid = cv2.VideoCapture(0) # define a video capture object

def change_res(width, height):
	vid.set(3, width)
	vid.set(4, height)

def mqtt_pubData(ids,corners):

	userdata = []
	for i in range(0,len(ids)):
		marker = {}
		marker["id"]= int(ids[i][0])
		marker["x"]= int(corners[i][0][0][0])
		marker["y"]= int(corners[i][0][0][1])
		userdata.append(marker)
	payload_json = json.dumps(userdata)
	#client.publish("data", payload=payload_json, qos=0, retain=False)

if __name__ == '__main__':
	#start the MQTT thread
	#_thread.start_new_thread( data_Thread, (1,"Thread-1", 2) )
	#print("[DEBUG	] Thread MQTT Started")
	timeStart = 0
	#TODO resolution to define
	#change_res(1280, 960)  #https://www.codingforentrepreneurs.com/blog/open-cv-python-change-video-resolution-or-scale
	change_res(2592, 1944)  #https://www.codingforentrepreneurs.com/blog/open-cv-python-change-video-resolution-or-scale
	#change_res(1920, 1080) #https://www.codingforentrepreneurs.com/blog/open-cv-python-change-video-resolution-or-scale

	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250 )
	parameters =  aruco.DetectorParameters_create()

	while(True):
		t1 = time.time()
		#pathent = "./images/Labeau/test7.png"
		#frame = cv2.imread(pathent)
		ret, frame = vid.read()
		
		#frame=cv2.rotate(frame, cv2.ROTATE_180)
		
		gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
		#frame = aruco.drawDetectedMarkers(frame, corners)

		if ids is not None:
			markerSizeInCM = 7
			rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, K, D)
			#aruco.drawAxis(frame, K, D, rvec[i], tvec[i], 20)
			for i in range(0, len(ids)):
				line_thickness = 1
				if(ids[i][0] == 42):
					#dessine le milieu de chaque tag
					corner_42 = corners[i][0]
					M = cv2.moments(corner_42)
					x_sum = int(M["m10"] / M["m00"])
					y_sum = int(M["m01"] / M["m00"])
					frame = cv2.circle(frame, (int(x_sum),int(y_sum)), radius=5, color=(0, 0, 255), thickness=2)
					#corners_abcd = corner_42.reshape((4, 2))
					
					#TODO 
					# Computing rotation matrix
					rotation_matrix = np.zeros(shape=(3,3))
					cv2.Rodrigues(rvec[i], rotation_matrix)
					#Apply rotation matrix to point
					original_point = tvec[i] #np.matrix([[1],[0],[0]])
					rotated_point = rotation_matrix*original_point
			mqtt_pubData(ids,corners)			
			print(ids)
			print(tvec)
			print(rvec)
			
			
		#frame = cv2.resize(frame, (960, 540))
		#frame = cv2.imread("1")
		cv2.imshow("Img",cv2.resize(frame,(720,480)))
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print("fps " + str(1/(time.time()-t1)))
		
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()