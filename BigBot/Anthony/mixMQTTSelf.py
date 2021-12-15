#!/usr/bin/python
import Robot
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import math
import serial
import json
from enum import IntEnum
from motorLogic import *
import sys
import _thread
import paho.mqtt.client as mqtt
import time


def rotationMatrixToEulerAngles(R) :

	sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

	singular = sy < 1e-6

	if not singular :
		x = math.atan2(R[2,1] , R[2,2])
		y = math.atan2(-R[2,0], sy)
		z = math.atan2(R[1,0], R[0,0])
	else :
		x = math.atan2(-R[1,2], R[1,1])
		y = math.atan2(-R[2,0], sy)
		z = 0

	return np.array([math.degrees(x), math.degrees(y), math.degrees(z)])



camera_matrix = np.array([[1.28799158e+03 , 0.00000000e+00, 6.53026522e+02],
 [0.00000000e+00, 1.28175352e+03, 4.68851197e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coeff = np.array([[ 0.20209073, -1.26402114,0.0062418, -0.00483837,1.88582135]])

angle_camera = 90
theta_camera = math.radians(angle_camera)

rotation_matrix = np.array([[1,           0 ,                0], #rotation axe X
[0           ,math.cos(theta_camera),               -math.sin(theta_camera)],
[0,         math.sin(theta_camera),                math.cos(theta_camera)]]) 



#--- initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.iso = 1600 # max ISO to force exposure time to minimum to get less motion blur
camera.contrast = 100
camera.resolution = (1296,976)
rawCapture = PiRGBArray(camera, size=camera.resolution)

C_IP_MQTT = "172.30.40.20"

client = mqtt.Client() 
targetX = 0
targetY = 0

newMessage = False




def on_message(client, userdata, message):
	global newMessage
	global JeanMichelDuma
	global targetX,targetY
	if(newMessage == False):
		msg = message.payload.decode("utf-8")
		#print("message received " ,str(msg))
		#print("message topic=",message.topic)
		msg = json.loads(msg)
		if(msg["id" ] == 47):
			targetX = msg["x"]
			targetY = msg["y"]
			#print("Tag 42 X Y  = " + str(targetX) + " " + str(targetY))
		elif(msg["id" ] == 17):
			JeanMichelDuma.positionX = msg["x"]
			JeanMichelDuma.positionY = msg["y"]
			JeanMichelDuma.orientationZ = msg["rz"]

			#print("ROBOT X Y  = " + str(robotX) + " " + str(robotY))
		newMessage = True
	else:
		print("RONPICH")
		time.sleep(0.1)




def on_connect(client, userdata, flags, rc):
	if rc==0:
		print("connected OK Returned code=",rc)
		client.subscribe("data/#")
		newMessage = False
	else:
		print("Bad connection Returned code=",rc)
		
def data_Thread(theadID):
	while True:
		print('[DEBUG	] Connecting to the TTN Broker...')
		#client.connect("192.168.0.13", 1883, 60)
		while( client.connect(C_IP_MQTT, 1883, 60) != 0):
			pass#client.subscribe("data/42")
		print("connected")
		client.loop_forever()




  


if __name__ == '__main__':
	JeanMichelDuma = Robot()
	markerSizeInCM = 5
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	parameters =  aruco.DetectorParameters_create()
	print(rotation_matrix)
	client.on_message = on_message
	client.on_connect = on_connect
	_thread.start_new_thread( data_Thread, (1 , ) )

	arrived_close = False

	while(not arrived_close):
		distance = JeanMichelDuma.goToUsingLocation(targetX,targetY)
		if(distance < 20):
			arrived_close = True
			print("Arrived")
		



	for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		frame = frame_pi.array
		frame = cv2.rotate(frame,cv2.ROTATE_180)
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		if ids is not None:
			#frame = aruco.drawDetectedMarkers(frame, corners)
			ret = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, camera_matrix, distortion_coeff)	
			(rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
			rvec_xyz =  np.matmul(rotation_matrix, rvec)
			rotation,_ = cv2.Rodrigues(rvec_xyz)
			euleurAngle = rotationMatrixToEulerAngles(rotation)
			#print("Rotation : \n" + str(euleurAngle[2]))
			rz = abs(euleurAngle[2])
			coord_xyz = np.matmul(rotation_matrix, tvec)
			angle = rz % 60
			JeanMichelDuma.goToSelfCamera(coord_xyz[1],coord_xyz[0])

		rawCapture.truncate(0)

	cv2.destroyAllWindows()