#!/usr/bin/python

import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import math


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6



# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

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



camera_matrix = np.array([[1.28799158e+03 , 0.00000000e+00, 6.53026522e+02],
 [0.00000000e+00, 1.28175352e+03, 4.68851197e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coeff = np.array([[ 0.20209073, -1.26402114,0.0062418, -0.00483837,1.88582135]])

angle = -60
theta = math.radians(angle)
""" rotation_matrix = np.array([[math.cos(theta), 0 , math.sin(theta)],
                        [0,               1,               0],
                        [-math.sin(theta), 0, math.cos(theta)]]) rotation Y"""
rotation_matrix = np.array([[1,           0 ,                0], #rotation axe X
[0           ,math.cos(theta),               -math.sin(theta)],
[0,         math.sin(theta),                math.cos(theta)]]) 



#--- initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
#camera.rotation = 180
camera.iso = 1600 # max ISO to force exposure time to minimum to get less motion blur
camera.contrast = 100
camera.resolution = (1296,976)
#camera.vflip = True
#camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)



if __name__ == '__main__':
    markerSizeInCM = 5
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters =  aruco.DetectorParameters_create()
    print(rotation_matrix)



    for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame_pi.array
        #frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners)
            ret = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, camera_matrix, distortion_coeff)	
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            rvec_xyz =  np.matmul(rotation_matrix, rvec)
            rotation,_ = cv2.Rodrigues(rvec_xyz)
            euleurAngle = rotationMatrixToEulerAngles(rotation)
            #print("RVEC : " + str(rvec))
            #print("Rotation : \n" + str(euleurAngle))     
            coord_xyz = np.matmul(rotation_matrix, tvec)
            print(": XYZ " + str(coord_xyz))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #cv2.imshow("LIVE FEED" ,frame)	
        rawCapture.truncate(0)

    cv2.destroyAllWindows()




