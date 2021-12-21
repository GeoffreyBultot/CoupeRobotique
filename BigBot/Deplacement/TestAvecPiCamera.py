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



""" DIM=(1280, 960) #21-12
camera_matrix=np.array([[612.6769659225217, 0.0, 573.732399040683], [0.0, 614.0511112656127, 464.15063277573313], [0.0, 0.0, 1.0]])
distortion_coeff = np.array([[-0.027945714136134205], [-0.012776430253932694], [0.00586270163443667], [-0.0015790193010345587]]) """

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




if __name__ == '__main__':
    markerSizeInCM = 5
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    parameters =  aruco.DetectorParameters_create()
    print(rotation_matrix)



    for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame_pi.array
        #frame = cv2.flip(frame,0)
        #frame = cv2.flip(frame,1)
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
            print("Angles " + str(euleurAngle))
            cv2.imshow("ENT",frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #cv2.imshow("LIVE FEED" ,frame)	
        rawCapture.truncate(0)

    cv2.destroyAllWindows()




