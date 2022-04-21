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
import argparse
import RPi.GPIO as GPIO

import ydlidar

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
    C_IP_MQTT = "192.168.0.17"
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    while True:
        print('[DEBUG	] Connecting to the TTN Broker...')
        #client.connect("192.168.0.13", 1883, 60)
        print(client.connect(C_IP_MQTT, 1883, 60))
        client.loop_forever()


def lidarThread():
    distanceMax = 300 #mm
    robotSize = 260 #mm
    size_gliss = 5

    last_positive = [False]*size_gliss

    ydlidar.os_init()
    laser = ydlidar.CYdLidar()

    port = "/dev/tty_LIDAR"

    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan = ydlidar.LaserScan()
        try:
            while ret and ydlidar.os_isOk() :
                r = laser.doProcessSimple(scan)
                if r:
                    frontFound = False
                    rightFound = False
                    leftFound = False
                    backFound = False
                    for p in scan.points:
                        last_positive.pop(0)
                        curr_angle = int(math.degrees(p.angle)+180)
                        dist = (p.range)*254 #*0.0254*10*1000 #Il renvoie le range en 1/10 de inches, *0.0254 => to dm, *10 to m, *1000 to mm

                        if robotSize/2 < dist < distanceMax:
                            last_positive.append(True)
                        else:
                            last_positive.append(False)

                        if False not in last_positive:
                            '''
                            ***FRONT***
                            ****90°****
                            0°*****180° RIGHT
                            ***270°****
                            ***BACK****
                            '''
                            #print(f"Caught object with angle {curr_angle}° and range : {round(dist/10, 2)}cm") 
                            if 45 <= curr_angle <= 135:
                                print("Object on the front")
                                frontFound = True

                            elif 135 < curr_angle < 225:
                                print("Object on the right")
                                rightFound = True

                            elif 225 <= curr_angle < 315:
                                print("Object on the back")
                                backFound = True

                            elif (315 <= curr_angle <= 360) or (0 <= curr_angle < 45):
                                print("Object on the left")
                                leftFound = True

                    direction = JeanMichelDuma.getCurrentDirection()

                    if frontFound and not JeanMichelDuma.is_object_in_dir['front']:
                        JeanMichelDuma.is_object_in_dir['front'] = True
                        if direction == "forward":
                            print("STEAUP FRONT ENCULER")
                            JeanMichelDuma.stopMotors()
                    elif frontFound and JeanMichelDuma.is_object_in_dir['front']:
                        pass
                    else:
                        JeanMichelDuma.is_object_in_dir['front'] = False


                    if rightFound and not JeanMichelDuma.is_object_in_dir['right']:
                        JeanMichelDuma.is_object_in_dir['right'] = True
                        if direction == "right":
                            print("STEAUP RIGHT ENCULER")
                            JeanMichelDuma.stopMotors()
                    elif frontFound and JeanMichelDuma.is_object_in_dir['right']:
                        pass
                    else:
                        JeanMichelDuma.is_object_in_dir['right'] = False


                    if backFound and not JeanMichelDuma.is_object_in_dir['back']:
                        JeanMichelDuma.is_object_in_dir['back'] = True
                        if direction == "backward":
                            print("STEAUP BACK ENCULER")
                            JeanMichelDuma.stopMotors()
                    elif frontFound and JeanMichelDuma.is_object_in_dir['back']:
                        pass
                    else:
                        JeanMichelDuma.is_object_in_dir['back'] = False


                    if leftFound and not JeanMichelDuma.is_object_in_dir['left']:
                        JeanMichelDuma.is_object_in_dir['left'] = True
                        if direction == "left":
                            print("STEAUP LEFT ENCULER")
                            JeanMichelDuma.stopMotors()
                    elif frontFound and JeanMichelDuma.is_object_in_dir['left']:
                        pass
                    else:
                        JeanMichelDuma.is_object_in_dir['left'] = False
                    
                else:
                    print("Failed to get Lidar Data.")

                time.sleep(0.2)

            laser.turnOff()    
        except KeyboardInterrupt:
            laser.turnOff()
            laser.disconnecting()
        
    laser.disconnecting()

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
    distanceBetweenGallery = abs(dict_zones['Galerie_Rouge'][0] - dict_zones['Galerie_Vert'][0])
    steps = stepsFromCm(distanceBetweenGallery)
    if i == 0:
        JeanMichelDuma.approachTargetUsingRotation(dict_zones['Galerie_Rouge'][0],dict_zones['Galerie_Rouge'][1]) #TODO A TESTER
        while(not JeanMichelDuma.goToNewVersion(dict_zones['Galerie_Rouge'][0],dict_zones['Galerie_Rouge'][1], precision)):
            pass
    elif i == 1:
        JeanMichelDuma.goLeft(steps)
        time.sleep(3)
        while(not JeanMichelDuma.goToNewVersion(dict_zones['Galerie_Vert'][0],dict_zones['Galerie_Vert'][1], precision)):
            pass
    elif i == 2:
        JeanMichelDuma.goLeft(steps)
        time.sleep(3)
        while(not JeanMichelDuma.goToNewVersion(dict_zones['Galerie_Bleu'][0],dict_zones['Galerie_Bleu'][1], precision)):
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
    while(not JeanMichelDuma.goToNewVersion(dict_zones['Campement'][0],dict_zones['Campement'][1],2)):
        pass
    exit()


def flexBrr():
    for i in range(0, 25):
        flex()


def goToStartPosition(side_to_start):
    JeanMichelDuma.goForward(STEPS_TO_START)
    if(side_to_start == "mauve"):
        JeanMichelDuma.rotationRight(JeanMichelDuma.stepsForAngle(103))
    else:
        JeanMichelDuma.rotationLeft(JeanMichelDuma.stepsForAngle(103))
    time.sleep(4.8)
    """while(not JeanMichelDuma.setOrientation(40,5)): #s'oriente pour les tag
        pass"""


def getFirst3Items():
    res_drive = loopDrivingUntilFound("GND")
    print("res_drive1 :", res_drive)
    grabItem("GND")

    while True:#numberOfItemInStockage() < 3:
        arm_thread = threading.Thread(target=storeItem, args=())
        arm_thread.start()

        if numberOfItemInStockage() >= 2:
            arm_thread.join()
            break

        drive_thread = threading.Thread(target=loopDrivingUntilFound, args=("GND",))
        drive_thread.start()

        arm_thread.join()

        if drive_thread.is_alive():
            arm.setServosOurAngle([90,90,90])
            drive_thread.join()

        grabItem("GND")


def placeItemsInGallery():
    while(not JeanMichelDuma.setOrientation(0,6)):
        print("Positioning for Gallery")
        print("Orientation = ", JeanMichelDuma.orientationZ)
        pass

    JeanMichelDuma.stopMotors()

    storage_thread = threading.Thread(target=reverseStorageSafe, args=())
    storage_thread.start()

    for i in range(0, 4):
        drive_thread = threading.Thread(target=goToGallery, args=(i,))
        drive_thread.start()

        storage_thread.join()

        if i!= 3:
            arm_thread = threading.Thread(target=setGalleryBot, args=(i,))
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


def getDistribItem():
    while(not JeanMichelDuma.setOrientation(90,3)):
        pass
    distance = abs(JeanMichelDuma.positionX - dict_zones['DispenserMat'][0])
    steps = stepsFromCm(distance)
    JeanMichelDuma.goForward(steps)
    startTime = time.time()
    endTime = startTime
    while(JeanMichelDuma.positionX < dict_zones['DispenserMat'][0] - 5 and (endTime - startTime) < 6):
        endTime = time.time()
    while(not JeanMichelDuma.setOrientation(0,4)):
        pass

    #-----On est au Distrib-------------
    loopDrivingUntilFound("DSTB")
    grabItem("DSTB0")
    storeItem()


def placeItemsInGalleryTop():
    #ON A TOUT
    storage_thread = threading.Thread(target=reverseStorageSafe, args=())
    storage_thread.start()
    #-----On repart vers la gallerie bleue-----
    drive_thread = threading.Thread(target=goGalleryBluePseudoFinal, args=())
    drive_thread.start()

    storage_thread.join()
    drive_thread.join()

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


def goGalleryBluePseudoFinal():
    while(not JeanMichelDuma.setOrientation(270,5)):
        pass
    distance = abs(JeanMichelDuma.positionX - dict_zones['Galerie_Bleu'][0])
    steps = stepsFromCm(distance)*2 #pcq on va sur le cote et CPT
    JeanMichelDuma.goForward(steps)
    while(JeanMichelDuma.positionX > dict_zones['Galerie_Bleu'][0] + 5 ):
        pass
    while(not JeanMichelDuma.setOrientation(0,4)):
        pass
    while(not JeanMichelDuma.goToNewVersion(dict_zones['Galerie_Bleu'][0],dict_zones['Galerie_Bleu'][1],2)):
        pass


def startup(side_to_start):
    initUndis()
    global dict_zones
    if(side_to_start == "mauve"):
        dict = {}
        for ele in dict_zones:
            pose = dict_zones[ele]
            dict[ele]=(300-pose[0],pose[1])
        dict_zones = dict
        #print(dict_zones)

    MQQT_thread = threading.Thread(target=data_Thread, args=(42,))
    MQQT_thread.start()
    print("[DEBUG	] Thread MQTT Started")

    lidar_thread = threading.Thread(target=lidarThread, args=())
    lidar_thread.start()
    print("[DEBUG	] Thread LIDAR Started")

    print("[WARNING] wait for TIRER LA CORDE")
    #   while(pin == 1)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(22, GPIO.IN)
    i = GPIO.input(22)
    print(i)
    while(i == 1):
        i = GPIO.input(22)

    try:
        goToStartPosition(side_to_start)
        if time.time() > last_time_before_camp:
            print("go to camp bitch")
            campementBrr()

        getFirst3Items()
        if time.time() > last_time_before_camp:
            print("go to camp bitch")
            campementBrr()

        placeItemsInGallery()
        if time.time() > last_time_before_camp:
            print("go to camp bitch")
            campementBrr()

        getDistribItem()
        if time.time() > last_time_before_camp:
            print("go to camp bitch")
            campementBrr()

        placeItemsInGalleryTop()
        if time.time() > last_time_before_camp:
            print("go to camp bitch")
            campementBrr()


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

team = "Y" #"P"
isCodeRunning = True
isArmMoving = False

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

start = time.time()
end = start + 100
last_time_before_camp = end - 20

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', type=str, required=True, help='side to start [jaune - mauve]:hibou:')
    args = parser.parse_args()
    for i in range(0,10):
        print('[Warning] On commence cote ', args.s)
        #time.sleep(0.1)
    startup(args.s)
    
