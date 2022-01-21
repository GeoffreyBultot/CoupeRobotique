import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import math


def rotationMatrixToEulerAngles(R) :
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

map1 = None
map2 = None

def initUndis():
	global map1
	global map2
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

DIM=(1280, 960)
K=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
D=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]])

angle_camera = 20
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

def changeXYZ(xyz):
    temp = [-xyz[1],xyz[0], xyz[2] ]
    return temp



if __name__ == '__main__':
    initUndis()
    markerSizeInCM = 5
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    parameters =  aruco.DetectorParameters_create()
    print(rotation_matrix)

    for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame_pi.array
        frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners)
            ret = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, K, D)	
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            coord_xyz = np.matmul(rotation_matrix, tvec)
            coord_xyz = changeXYZ(coord_xyz)
            print(coord_xyz)
            rvec_xyz = rvec
            rotation,_ = cv2.Rodrigues(rvec_xyz)
            rvec_xyz = np.matmul(rotation_matrix, rvec) 
            euleurAngle = rotationMatrixToEulerAngles(rotation)
            dist = math.sqrt(coord_xyz[0]**2 + coord_xyz[1]**2)
            print("Dist = ", dist)
            #print("Euler = " ,euleurAngle[2])
            #print("XYZ = ",coord_xyz)

        cv2.imshow("LIVE FEED" ,frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rawCapture.truncate(0)

    cv2.destroyAllWindows()
