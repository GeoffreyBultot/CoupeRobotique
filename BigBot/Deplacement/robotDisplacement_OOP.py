#!/usr/bin/python
from Robot import *
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import math
import serial
import json
from enum import IntEnum
import sys



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

""" DIM=(1280, 960)
K=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
D=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]]) """


DIM=(1280, 960)
camera_matrix=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
distortion_coeff=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]])



angle_camera = 35
theta_camera = math.radians(angle_camera)

rotation_matrix = np.array([[1,           0 ,                0], #rotation axe X
[0           ,math.cos(theta_camera),               -math.sin(theta_camera)],
[0,         math.sin(theta_camera),                math.cos(theta_camera)]]) 



camera = PiCamera()
camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
camera.contrast = 0
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

def calculateDistance(list):
    dist = 0
    for elt in list :
        dist += elt**2
    dist = math.sqrt(dist)
    return dist




  


if __name__ == '__main__':
    JeanMichelDuma = Robot()
    JeanMichelDuma.DEBUG = 0
    markerSizeInCM = 5
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    parameters =  aruco.DetectorParameters_create()
    print(rotation_matrix)
    offset_x = -.5
    offset_y = 1
    distance = []
    ret = []

    #ser.close()

    for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame_pi.array
        #frame = cv2.flip(frame,0)
        #frame = cv2.flip(frame,1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            for i in range(len(ids)):
                ret = aruco.estimatePoseSingleMarkers(corners[i], markerSizeInCM, camera_matrix, distortion_coeff)
                (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                distance.append(calculateDistance(tvec))
            min_dist = min(distance)
            index = distance.index(min_dist)
            ret = aruco.estimatePoseSingleMarkers(corners[index], markerSizeInCM, camera_matrix, distortion_coeff) #TODO évité de calculer 2x
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            rvec_xyz =  np.matmul(rotation_matrix, rvec)
            rotation,_ = cv2.Rodrigues(rvec_xyz)
            euleurAngle = rotationMatrixToEulerAngles(rotation)
            #print("Rotation : \n" + str(euleurAngle[2]))
            rz = abs(euleurAngle[2])
            coord_xyz = np.matmul(rotation_matrix, tvec)
            #print(coord_xyz)
            rz = rz % 360
            print(rz)
            distance = []
            if(coord_xyz[1] > 0.5):
                print("Steaup")
                JeanMichelDuma.block()
                exit()
            elif(-coord_xyz[1] > offset_x or abs(coord_xyz[0]) > offset_y ):
                JeanMichelDuma.goToSelfCamera(-coord_xyz[1],coord_xyz[0],rz,offset_x,offset_y)
                #print("Distance to target = " + str(distance))
            
            else:
                print("Not detected")
                JeanMichelDuma.block()
                

        rawCapture.truncate(0)

    cv2.destroyAllWindows()