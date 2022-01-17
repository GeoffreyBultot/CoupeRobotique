import time
import picamera
import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
#vid = cv2.VideoCapture(0)
#camera = picamera.PiCamera()
DIM=(1280, 960)


CHECKERBOARD = (6,9)
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
_img_shape = None
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.



camera = PiCamera()
#camera.rotation = 180
#camera.iso = 800 # max ISO to force exposure time to minimum to get less motion blur
#camera.contrast = 0
camera.resolution = DIM
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

i = 0
for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	frame = frame_pi.array
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	# Find the chess board corners
	ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
	# If found, add object points, image points (after refining them)
	print(i)
	if ret == True:
		print("ok")
		i = i+1
		strf = '/home/pi/Documents/Calibration/17_01/'+str(i)+'.png'
		cv2.imwrite(strf,frame)
		cv2.imshow("captured",frame)
		if cv2.waitKey(0) & 0xFF == ord('q'):
			cv2.destroyAllWindows()
		objpoints.append(objp)
		cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
		imgpoints.append(corners)
		if(i==30):
			exit(0)
	else:
		print("nok")
	rawCapture.truncate(0)