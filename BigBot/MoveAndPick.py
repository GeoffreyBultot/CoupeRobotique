#!/usr/bin/python
from Deplacement.Robot import *
from armMov import *
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import math
import threading
import _thread

from queue import Queue

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


def isStockageFull():
    for i in range(0, len(stockageArray)):
        if stockageArray[i] == False:
            return False
    
    return True


def grabItem(posElement):
    global isArmMoving

    isArmMoving = True
    if posElement == "GND":
        grabElementGround()

    elif posElement == "DSTB0":
        setArmPosDistrib(0)
        #SHOULD DRIVE FORWARD HERE
        suckAndSetArmUpDistrib()
        #SHOULD DRIVE BACK HERE

    elif posElement == "DSTB1":
        setArmPosDistrib(1)
        #SHOULD DRIVE FORWARD HERE
        suckAndSetArmUpDistrib()
        #SHOULD DRIVE BACK HERE

    elif posElement == "DSTB2":
        setArmPosDistrib(2)
        #SHOULD DRIVE FORWARD HERE
        suckAndSetArmUpDistrib()
        #SHOULD DRIVE BACK HERE


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


def loopDrivingUntilFound():
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
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
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
                #print(coord_xyz)
                #print(rz)
                distance = [] #clear le tableau
                ret_array = []
                if(JeanMichelDuma.approachTargetUsingRotation(coord_xyz,rz)):
                    if(JeanMichelDuma.goToSelfCamera(coord_xyz,rz)):
                        print("steaup")
                        #camera.close()
                        return True  
            else:
                print("Not detected")
                if not isArmMoving:
                    JeanMichelDuma.speed = JeanMichelDuma.dict_speed['Medium']
                    JeanMichelDuma.rotationLeft()

            rawCapture.truncate(0)

        cv2.destroyAllWindows()
        
        #camera.close()
        

def main():
    initUndis()
    try:
        arm.setServosOurAngle([90,92,92])
        loopDrivingUntilFound()
        grabItem("GND")

        while isCodeRunning and not isStockageFull():
            arm_thread = threading.Thread(target=lambda q, arg1: q.put(storeItem(arg1)), args=(que,))
            arm_thread.start()

            #driveThread = threading.Thread(target=loopDrivingUntilFound, args=())
            drive_thread = threading.Thread(target=lambda q, arg1: q.put( loopDrivingUntilFound(arg1)), args=(que,))
            drive_thread.start()

            arm_thread.join()

            if drive_thread.is_alive():
                hideInside()
                drive_thread.join()

            res_drive = que.get() 
            print("res_drive :", res_drive)
            
            grabItem("GND")

        
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

que = Queue()

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

main()
