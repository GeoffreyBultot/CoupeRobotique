#!/usr/bin/python
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
FORWARD = 1
BACKWARD = 0

ON = 1

PORT = "/dev/ttyUSB0"


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
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
diff = 5

ser = serial.Serial (PORT, 
        baudrate = 115200,)

def serialWriteReg(reg):
	try: 
		reg = reg.to_bytes(1,'big')
		ser.write(reg)
	except:
		print("ERROR USB")
		if(ser.isOpen()):
			ser.write(0)
			ser.close()
		else:
			ser.open() 


  


if __name__ == '__main__':
	markerSizeInCM = 5
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	parameters =  aruco.DetectorParameters_create()
	print(rotation_matrix)

	#ser.close()

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
			anglexy = math.degrees(math.atan(coord_xyz[1] / coord_xyz[0] ))
			if(anglexy < 0):
				anglexy = -(90+anglexy)
			elif(anglexy > 0):
				anglexy = 90-anglexy
			x = abs(coord_xyz[1])
			y = coord_xyz[0]
			offset_max_x = 20
			offset_max_y = 2
			offset_max_angle = 5

			
			if(anglexy > offset_max_angle or anglexy < -offset_max_angle  ):
				if(anglexy > 0):
					reg = rotationRight()
					print("rotationright")
				else:
					reg = rotationLeft()
					print("rotationleft")
			elif(x < -offset_max_x):
				reg = goForward()
				print("forward")
			else:
				reg = stopMotors()
				print("stop")
			print(f"X : {round(x, 2)}\nY: {round(y, 2)}\nrz: {round(angle, 2)}\nangleXY: {round(anglexy, 2)}\n")
			serialWriteReg(reg)			
		else:
			serialWriteReg(0)
			

		rawCapture.truncate(0)

	cv2.destroyAllWindows()
