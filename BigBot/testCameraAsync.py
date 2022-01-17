import io
import time
import picamera
import cv2
import numpy as np
from cv2 import aruco

from picamera import PiCamera
from picamera.array import PiRGBArray

from Deplacement.Robot import *
from armMov import *


DIM=(1280, 960)
camera_matrix=np.array([[630.932402116786, 0.0, 585.6531301759157], [0.0, 631.6869826709609, 478.8413904560236], [0.0, 0.0, 1.0]])
distortion_coeff=np.array([[-0.06670587491284909], [0.1057157290509116], [-0.13122001638126551], [0.04714118291127774]])

markerSizeInCM = 5
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
parameters =  aruco.DetectorParameters_create()


camera = PiCamera()
camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
camera.contrast = 0
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)
frame = None

# Create the in-memory stream
async def CaptureImg():
     for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame_pi.array

async def analyze_img():
    if(frame is not None):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            ret = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, camera_matrix, distortion_coeff))
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            print(tvec)

async def main():
    while(True):
        tasks = [CaptureImg(), analyze_img()]
        await asyncio.wait(tasks)

