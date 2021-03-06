#!/usr/bin/python
from Robot import *
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import math
import _thread
import paho.mqtt.client as mqtt
import json


TOPIC_BIG_BOT = "BigBot/2"

def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("connected OK Returned code=",rc)
        client.subscribe(TOPIC_BIG_BOT)
    else:
        print("Bad connection Returned code=",rc)


def on_message(client, userdata, message):
    global data_id
    global data_id_topic
    global data_rvec
    global data_tvec
    global JeanMichelDuma
    msg = message.payload.decode("utf-8")
    msg = json.loads(msg)
    if(message.topic == (TOPIC_BIG_BOT)):
        JeanMichelDuma.positionX = msg["x"]
        JeanMichelDuma.positionY = msg["y"]
        JeanMichelDuma.orientationZ = msg["rz"]
        #print("X Y RZ + " + str(msg["x"]) +  str(msg["y"]) +  str(msg["rz"]))

C_IP_MQTT = "172.30.40.68"
client = mqtt.Client()
client.on_connect = on_connect

def data_Thread(theadID):
    while True:
        print('[DEBUG	] Connecting to the TTN Broker...')
        #client.connect("192.168.0.13", 1883, 60)
        print(client.connect(C_IP_MQTT, 1883, 60))
        client.loop_forever()





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




DIM=(1280, 960)
camera_matrix=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
distortion_coeff=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]])



angle_camera = 30
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

def changeXYZ(xyz):
    temp = [-xyz[1],xyz[0], xyz[2] ]
    return temp

""" 
x
^
|
|
0----->y
 """

if __name__ == '__main__':
    JeanMichelDuma = Robot()
    JeanMichelDuma.DEBUG = 0
    _thread.start_new_thread( data_Thread, (1 ,) )
    print("[DEBUG	] Thread MQTT Started")
    client.on_connect = on_connect
    client.on_message = on_message
    markerSizeInCM = 5
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    parameters =  aruco.DetectorParameters_create()
    print(rotation_matrix)
    distance = []
    ret_array = []
    while(not JeanMichelDuma.setOrientation(0,5)):
        pass

    print("ORIENTED")
    JeanMichelDuma.stopMotors()


    for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame_pi.array
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            for i in range(len(ids)): #trouve le tag le plus proche
                ret_array.append(aruco.estimatePoseSingleMarkers(corners[i], markerSizeInCM, camera_matrix, distortion_coeff))
                ret = ret_array[i]
                (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                distance.append(calculateDistance(tvec))
            min_dist = min(distance)
            index = distance.index(min_dist)
            #print("Target is TAG " + str(ids[index]))
            ret = ret_array[index]
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            rvec_xyz =  np.matmul(rotation_matrix, rvec)
            rotation,_ = cv2.Rodrigues(rvec_xyz)
            euleurAngle = rotationMatrixToEulerAngles(rotation)
            #print("Rotation : \n" + str(euleurAngle[2]))
            rz = euleurAngle[2]
            coord_xyz = np.matmul(rotation_matrix, tvec)
            coord_xyz = changeXYZ(coord_xyz)
            #print(coord_xyz)
            #print(rz)
            distance = [] #clear le tableau
            ret_array = []
            #if(JeanMichelDuma.approachTargetUsingRotation(coord_xyz,rz)):
            if(JeanMichelDuma.goToDistributeur(coord_xyz)):
                while(not JeanMichelDuma.setOrientation(0)):
                    pass
                print("steaup")
                JeanMichelDuma.stopMotors()
                camera.stop_recording()
                exit()
        else:
            #print("Not detected")
            JeanMichelDuma.stopMotors()
                

        rawCapture.truncate(0)

    cv2.destroyAllWindows()
