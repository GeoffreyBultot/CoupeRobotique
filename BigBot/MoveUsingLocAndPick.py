#!/usr/bin/python
from Deplacement.Robot import *
from armMov import *
from Deplacement.Zones_Strategy import dict_zones
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import math
import asyncio
import _thread
import paho.mqtt.client as mqtt
import json


def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("connected OK Returned code=",rc)
        client.subscribe(TOPIC_BIG_BOT)
    else:
        print("Bad connection Returned code=",rc)


def on_message(client, userdata, message):
    global JeanMichelDuma
    msg = message.payload.decode("utf-8")
    msg = json.loads(msg)
    if(message.topic == TOPIC_BIG_BOT):
        JeanMichelDuma.positionX = msg["x"]
        JeanMichelDuma.positionY = msg["y"]
        JeanMichelDuma.orientationZ = msg["rz"]
        #cprint(f"Robot : X =  {JeanMichelDuma.positionX}, Y = {JeanMichelDuma.positionY}, RZ = {JeanMichelDuma.orientationZ}")


def data_Thread(theadID):
    while True:
        print('[DEBUG	] Connecting to the TTN Broker...')
        #client.connect("192.168.0.13", 1883, 60)
        print(client.connect(C_IP_MQTT, 1883, 60))
        client.loop_forever()

C_IP_MQTT = "172.30.40.68"
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message


#region ENT
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
camera_matrix = np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
distortion_coeff = np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]])

angle_camera = 20
theta_camera = math.radians(angle_camera)

rotation_matrix = np.array([[1,           0 ,                0], #rotation axe X
[0           ,math.cos(theta_camera),               -math.sin(theta_camera)],
[0,         math.sin(theta_camera),                math.cos(theta_camera)]])

def initUndis():
	global map1
	global map2
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(camera_matrix, distortion_coeff, np.eye(3), camera_matrix, DIM, cv2.CV_16SC2)

def calculateDistance(list):
    dist = 0
    for elt in list :
        dist += elt**2
    dist = math.sqrt(dist)
    return dist

def changeXYZ(xyz):
    temp = [-xyz[1],xyz[0], xyz[2] ]
    return temp

def numberOfItemInStockage():
    n = 0
    for i in range(0, len(stockageArray)):
        if stockageArray[i] == True:
            n += 1
    return n

def isStockageFull():
    for i in range(0, len(stockageArray)):
        if stockageArray[i] == False:
            return False
    
    return True


async def grabItem(posElement):
    global isArmMoving

    isArmMoving = True
    if posElement == "GND":
        await grabElementGround()

    elif posElement == "DSTB0":
        await setArmPosDistrib(0)
        #SHOULD DRIVE FORWARD HERE
        await suckAndSetArmUpDistrib()
        #SHOULD DRIVE BACK HERE

    elif posElement == "DSTB1":
        await setArmPosDistrib(1)
        #SHOULD DRIVE FORWARD HERE
        await suckAndSetArmUpDistrib()
        #SHOULD DRIVE BACK HERE

    elif posElement == "DSTB2":
        await setArmPosDistrib(2)
        #SHOULD DRIVE FORWARD HERE
        await suckAndSetArmUpDistrib()
        #SHOULD DRIVE BACK HERE


async def storeItem():
    global isArmMoving
    
    await setupAfterGrab()
    for i in range(3, 0, -1):
        if not stockageArray[i]:
            await setSlotId(i)
            stockageArray[i] = True
            print("stockageArray :", stockageArray)
            isArmMoving = False
            return True
#endregion

async def loopDrivingUntilFound():
    global isArmMoving
    
    distance = []
    ret_array = []
    with PiCamera() as camera:
        camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
        camera.contrast = 0
        camera.resolution = DIM
        camera.framerate = 30
        rawCapture = PiRGBArray(camera, size=camera.resolution)
        for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            frame = frame_pi.array
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)    
            idx_42 = np.where(ids == [42])

            if(idx_42[0].size > 0):
                ids = np.delete(ids, idx_42[0], idx_42[1])
                corners = np.delete(corners, idx_42[0], idx_42[1])

            #print("ids !", ids)

            if ids is not None and ids.size > 0:
                for i in range(len(ids)): #trouve le tag le plus proche
                    ret_array.append(aruco.estimatePoseSingleMarkers(corners[i], markerSizeInCM, camera_matrix, distortion_coeff))
                    ret = ret_array[i]
                    (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                    distance.append(calculateDistance(tvec))
                #print("Distzncer brr :", distance)
                min_dist = min(distance)
                index = distance.index(min_dist)
                ret = ret_array[index]
                (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                rvec_xyz =  np.matmul(rotation_matrix, rvec)
                rotation,_ = cv2.Rodrigues(rvec_xyz)
                euleurAngle = rotationMatrixToEulerAngles(rotation)
                rz = euleurAngle[2]
                coord_xyz = np.matmul(rotation_matrix, tvec)
                coord_xyz = changeXYZ(coord_xyz)
                distance = [] #clear le tableau
                ret_array = []
                if(JeanMichelDuma.goToSelfCamera(coord_xyz,rz)):
                    print("steaup")
                    return True  
            else:
                print("Not detected")
                if not isArmMoving:
                    JeanMichelDuma.speed = JeanMichelDuma.dict_speed['Medium']
                    JeanMichelDuma.rotationLeft()

            await asyncio.sleep(0.05)
            rawCapture.truncate(0)

async def goToStartPosition():
    targetX = dict_zones['Start'][0] / 10
    targetY = dict_zones['Start'][1] / 10
    print(f"fTarget = {targetX} , {targetY}")
    targetAngle = 30
    while(True):
        await asyncio.sleep(0.15)
        if(JeanMichelDuma.goToUsingLocation(targetX,targetY,targetAngle)):
            client.unsubscribe(TOPIC_BIG_BOT)
            print("ARRIVED")
            time.sleep(5)
            return

async def main():
    initUndis() #initialize le undistort
    _thread.start_new_thread( data_Thread, (1 ,) )
    print("[DEBUG	] Thread MQTT Started")
    try:  
        loop = asyncio.get_event_loop()
        if not arm.isInside:
            await hideInside()
        await goToStartPosition()
        print("AYYYYYYYYYAAAAAAAAAAAA")
        await loopDrivingUntilFound()
        await grabItem("GND")

        while isCodeRunning and not isStockageFull():
            tasks = [storeItem(), loopDrivingUntilFound()]

            a, b = loop.run_until_complete(asyncio.wait(tasks))
            loop.close()
            
            print(f"a : {a}, b: {b}")
            
            await grabItem("GND")
            if(numberOfItemInStockage == 3):
                arm.disableTorqueAll()
                JeanMichelDuma.stopMotors()
                servo.stopPwm()
                exit()

        
    except KeyboardInterrupt:
        arm.disableTorqueAll()
        JeanMichelDuma.stopMotors()
        servo.stopPwm()

""" 
x
^
|
|
0----->y
 """

team = "Y" #"P"
isCodeRunning = True
isArmMoving = False


TOPIC_BIG_BOT = "BigBot/2"
JeanMichelDuma = Robot()
JeanMichelDuma.DEBUG = 0
markerSizeInCM = 5
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
parameters =  aruco.DetectorParameters_create()
print(rotation_matrix)

stockageArray = [False, False, False, False]

galleryArray =  [[False, False, False, False, False],
                [False, False, False, False, False]]

galleryColor = ['B','B', 'G', 'R', 'R']
if team == "P":
    galleryColor.reverse()

ventouse.setDefault()

arm.enableTorqueAll()
arm.setMaxTorqueAll(100)
arm.setTorqueLimitAll(100)
arm.MAX_OVERALL_SPEED = 20

servo = ServoStock(13, 400, GPIO.BCM)

servo.setDefault()
servo.stopPwm()

if __name__ == '__main__':
    asyncio.run(main())
