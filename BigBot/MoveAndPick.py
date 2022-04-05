#!/usr/bin/python
from Deplacement.Robot import *
from Deplacement.utils import stepsFromCm
from Deplacement.Zones_Strategy import dict_zones
from armMov import *
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import math
import threading
import _thread
import paho.mqtt.client as mqtt
import json


from queue import Queue

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


def data_Thread(theadID):
    C_IP_MQTT = "172.30.40.68"
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
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

def initUndis():
	global map1
	global map2
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(camera_matrix, distortion_coeff, np.eye(3), camera_matrix, DIM, cv2.CV_16SC2)

angle_camera = 20
theta_camera = math.radians(angle_camera)

rotation_matrix = np.array([[1,           0 ,                0], #rotation axe X
[0           ,math.cos(theta_camera),               -math.sin(theta_camera)],
[0,         math.sin(theta_camera),                math.cos(theta_camera)]]) 

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


def grabItem(posElement):
    global isArmMoving
    N_forward = 1200
    N_back = 500
    JeanMichelDuma.speed = JeanMichelDuma.dict_speed['Slow']
    print(numberOfItemInStockage())

    isArmMoving = True
    if posElement == "GND":
        grabElementGround()

    elif posElement == "DSTB0":
        setArmPosDistrib(0)
        time.sleep(0.9)
        JeanMichelDuma.goForward()
        time.sleep(0.9)
        JeanMichelDuma.stopMotors()
        suckAndSetArmUpDistrib(0)
        JeanMichelDuma.goBackward(N_back)

    elif posElement == "DSTB1":
        setArmPosDistrib(1)
        time.sleep(0.9)
        JeanMichelDuma.goForward()
        time.sleep(0.9)
        JeanMichelDuma.stopMotors()
        suckAndSetArmUpDistrib(1)
        JeanMichelDuma.goBackward(N_back)

    elif posElement == "DSTB2":
        setArmPosDistrib(2)
        time.sleep(0.9)
        JeanMichelDuma.goForward()
        time.sleep(0.9)
        JeanMichelDuma.stopMotors()
        suckAndSetArmUpDistrib(2)
        JeanMichelDuma.goBackward(N_back)


def storeItem():
    global isArmMoving
    
    setupAfterGrab()
    for i in range(3, 0, -1):
        if not stockageArray[i]:
            setSlotId(i)
            stockageArray[i] = True
            print("stockageArray :", stockageArray)
            isArmMoving = False
            return True

def goToGallery(i):
    precision = 4
    distanceBetweenGallery = abs(GallerieRougeX - GallerieVertX)
    steps = stepsFromCm(distanceBetweenGallery)
    if i == 0:
        JeanMichelDuma.approachTargetUsingRotation(GallerieRougeX,GallerieRougeY) #TODO A TESTER
        while(not JeanMichelDuma.goToNewVersion(GallerieRougeX,GallerieRougeY, precision)):
            pass
    elif i == 1:
        JeanMichelDuma.goLeft(steps)
        time.sleep(3)
        while(not JeanMichelDuma.goToNewVersion(GallerieVertX,GallerieVertY, precision)):
            pass
    elif i == 2:
        JeanMichelDuma.goLeft(steps)
        time.sleep(3)
        while(not JeanMichelDuma.goToNewVersion(GallerieBleuX,GallerieBleuY, precision)):
            pass
    elif i== 3:
        pass

        
def setGalleryBot(i):
    grabElementSlot(i)
    setArmBotGallery() #setArmBotGallery()


def loopDrivingUntilFound(pos_el):
    global isArmMoving

    #camera = PiCamera()
    
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
            #frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            idx_42 = np.where(ids == [42])

            if(idx_42[0].size > 0):
                ids = np.delete(ids, idx_42[0], idx_42[1])
                corners = np.delete(corners, idx_42[0], idx_42[1])

            if ids is not None and ids.size > 0:
                for i in range(len(ids)): #trouve le tag le plus proche
                    ret_array.append(aruco.estimatePoseSingleMarkers(corners[i], markerSizeInCM, camera_matrix, distortion_coeff))
                    ret = ret_array[i]
                    (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                    distance.append(calculateDistance(tvec))
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
                #print(coord_xyz)
                #print("rz :", rz)
                distance = [] #clear le tableau
                ret_array = []
                if pos_el == "GND":
                    if(JeanMichelDuma.goToSelfCamera(coord_xyz,rz)):
                        print("steaup")
                        return "GND"
                elif pos_el == "DSTB":
                    if(JeanMichelDuma.setOrientation(0,4)):
                        if(JeanMichelDuma.goToDistributeur(coord_xyz)):
                            print("steaup")
                            return "DSTB"
            else:
                print("Not detected")
                if not isArmMoving:
                    JeanMichelDuma.speed = JeanMichelDuma.dict_speed['Medium']
                    JeanMichelDuma.rotationLeft()

            rawCapture.truncate(0)

        cv2.destroyAllWindows()


def reverseStorageSafe():
    setOutsideFromInside()
    servo.setReverse()
    time.sleep(0.5)
    servo.stopPwm()
    time.sleep(0.5)
    print("Hiding Inside")
    hideInside()


def campementBrr():
    while(not JeanMichelDuma.goToNewVersion(CampementX,CampementY,2)):
        pass


def flexBrr():
    for i in range(0, 25):
        flex()


def goToStartPosition():
    JeanMichelDuma.goForward(STEPS_TO_START)
    JeanMichelDuma.rotationLeft(JeanMichelDuma.stepsForAngle(103))
    time.sleep(4.8)
    """while(not JeanMichelDuma.setOrientation(40,5)): #s'oriente pour les tag
        pass"""


def getFirst3Items():
    res_drive = loopDrivingUntilFound("GND")
    print("res_drive1 :", res_drive)
    grabItem("GND")

    while True:#numberOfItemInStockage() < 3:
        arm_thread = threading.Thread(target=lambda q: q.put(storeItem()), args=(que,))
        arm_thread.start()
        
        if numberOfItemInStockage() >= 2:
            arm_thread.join()
            res_arm = que.get()
            break

        drive_thread = threading.Thread(target=lambda q, arg1: q.put( loopDrivingUntilFound(arg1)), args=(que,"GND"))
        drive_thread.start()

        arm_thread.join()
        res_arm = que.get()
        print("res_arm :", res_arm)

        if drive_thread.is_alive():
            arm.setServosOurAngle([90,90,90])
            drive_thread.join()

        res_drive = que.get() 
        print("res_drive :", res_drive)
        
        grabItem("GND")


def placeItemsInGallery():
    while(not JeanMichelDuma.setOrientation(0,6)):
        print("Positioning for Gallery")
        print("Orientation = ", JeanMichelDuma.orientationZ)
        pass
        
    JeanMichelDuma.stopMotors()

    storage_thread = threading.Thread(target=lambda q: q.put(reverseStorageSafe()), args=(que,))
    storage_thread.start()
    
    for i in range(0, 4):
        drive_thread = threading.Thread(target=lambda q, arg1: q.put(goToGallery(arg1)), args=(que,i))
        drive_thread.start()

        storage_thread.join()

        if i!= 3:
            arm_thread = threading.Thread(target=lambda q, arg1: q.put(setGalleryBot(arg1)), args=(que,i))
            arm_thread.start()

        arm_thread.join()
        drive_thread.join()

        while(not JeanMichelDuma.setOrientation(0,2)):
            pass

        dist = JeanMichelDuma.positionY - 28 #TODO CHANGER
        steps = abs(stepsFromCm(dist))
        JeanMichelDuma.goForward(steps)
        JeanMichelDuma.waitForSteps(steps)
        
        JeanMichelDuma.block()
        time.sleep(0.3)
        JeanMichelDuma.stopMotors()

        ventouse.drop()
        setupAfterGrab()

        stockageArray[i] = False
    
    hideInside()
    servo.setDefault()


def goGalleryBluePseudoFinal():
    while(not JeanMichelDuma.setOrientation(270,5)):
        pass
    distance = abs(JeanMichelDuma.positionX - GallerieBleuX)
    steps = stepsFromCm(distance)*2 #pcq on va sur le cote et CPT
    JeanMichelDuma.goForward(steps)
    while(JeanMichelDuma.positionX > GallerieBleuX + 5 ):
        pass
    while(not JeanMichelDuma.setOrientation(0,4)):
        pass
    while(not JeanMichelDuma.goToNewVersion(GallerieBleuX,GallerieBleuY,2)):
        pass


def main():
    initUndis()
    MQQT_thread = threading.Thread(target=data_Thread, args=(42,))
    MQQT_thread.start()
    print("[DEBUG	] Thread MQTT Started")
    try:
        goToStartPosition()
        getFirst3Items()
        placeItemsInGallery()

        #-----#On est à la gallerie bleue, on va chercher dans le stockage--------
        #New version
        while(not JeanMichelDuma.setOrientation(90,3)):
            pass
        distance = abs(JeanMichelDuma.positionX - DistribMatX)
        steps = stepsFromCm(distance)
        JeanMichelDuma.goForward(steps)
        startTime = time.time()
        endTime = startTime
        while(JeanMichelDuma.positionX < DistribMatX - 5 and (endTime - startTime) < 6):
            endTime = time.time()
        while(not JeanMichelDuma.setOrientation(0,4)):
            pass
        """ JeanMichelDuma.goRight(60000) 
        time.sleep(5)

        while(not JeanMichelDuma.goToNewVersion(DistribMatX,DistribMatY,2)):
            pass
        
        while(not JeanMichelDuma.setOrientation(0,4)):
            pass
        
        JeanMichelDuma.stopMotors() """

        #-----On est au Distrib-------------
        res_drive = loopDrivingUntilFound("DSTB") 
        grabItem("DSTB0")
        storeItem()

        storage_thread = threading.Thread(target=lambda q: q.put(reverseStorageSafe()), args=(que,))
        storage_thread.start()
        #-----On repart vers la gallerie bleue-----
        drive_thread = threading.Thread(target=lambda q: q.put(goGalleryBluePseudoFinal()), args=(que,))
        drive_thread.start()

        storage_thread.join()
        drive_thread.join()
        
        
        """ JeanMichelDuma.stopMotors()

        JeanMichelDuma.goLeft(60000)
        time.sleep(5)

        while(not JeanMichelDuma.goToNewVersion(GallerieBleuX,GallerieBleuY,2)):
            pass

        while(not JeanMichelDuma.setOrientation(0,2)):
            pass

        JeanMichelDuma.stopMotors() """

        grabElementSlot(0)
        setArmTopGallery()
        dist = JeanMichelDuma.positionY - 26 #TODO CHANGER
        steps = abs(stepsFromCm(dist))
        JeanMichelDuma.goForward(steps)
        time.sleep(steps/1000)
        
        JeanMichelDuma.block()
        time.sleep(0.1)
        JeanMichelDuma.stopMotors()
        ventouse.drop()
        
        print("steaup final")

        arm.disableTorqueAll()
        JeanMichelDuma.stopMotors()
        servo.stopPwm()
        servo.setDefault()
        time.sleep(1)
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

TOPIC_BIG_BOT = "BigBot/2"

DIST_START = 77
STEPS_TO_START = stepsFromCm(DIST_START)

StartX = dict_zones['Start'][0]
StartY = dict_zones['Start'][1]
GallerieRougeX = dict_zones['Galerie_Rouge'][0]
GallerieRougeY = dict_zones['Galerie_Rouge'][1]
GallerieVertX = dict_zones['Galerie_Vert'][0]
GallerieVertY = dict_zones['Galerie_Vert'][1]
GallerieBleuX = dict_zones['Galerie_Bleu'][0]
GallerieBleuY = dict_zones['Galerie_Bleu'][1]

DistribMatX = dict_zones['DispenserMat'][0]
DistribMatY = dict_zones['DispenserMat'][1]

CampementX = dict_zones['Campement'][0]
CampementY = dict_zones['Campement'][1]


team = "Y" #"P"
isCodeRunning = True
isArmMoving = False

que = Queue()

JeanMichelDuma = Robot()
JeanMichelDuma.DEBUG = 0
markerSizeInCM = 5
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
parameters =  aruco.DetectorParameters_create()

stockageArray = [False, False, False, False]

galleryArray =  [[False, False, False, False, False],
                [False, False, False, False, False]]

galleryColor = ['B','B', 'G', 'R', 'R']
if team == "P":
    galleryColor.reverse()

arm.disableTorqueAll()
arm.enableTorqueAll()
arm.setMaxTorqueAll(100)
arm.setTorqueLimitAll(100)
arm.setDelayTimeAll(0)

arm.setServosOurAngle([90,90,90])

servo = ServoStock(13, 400, GPIO.BCM)

ventouse.setDefault()
'''
servo.setDefault()
time.sleep(1)
servo.setReverse()
time.sleep(1)'''
servo.setDefault()
time.sleep(1)
servo.stopPwm()
time.sleep(1)

main()
