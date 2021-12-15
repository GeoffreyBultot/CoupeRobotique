import time
import picamera
import numpy as np
import cv2
#vid = cv2.VideoCapture(0)
#camera = picamera.PiCamera()


CHECKERBOARD = (6,9)
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
_img_shape = None
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

for i in range(40,50):
	with picamera.PiCamera() as camera:
		#camera.resolution = (3280, 2464)
		#camera.resolution = (2592, 1944)
		camera.resolution = (1280, 960)#camera.iso = 800
		strf = './imagesCali_1280_960/image_'+str(i)+'.png'
		camera.capture(strf)
		cv2.imshow(str(i), cv2.imread(strf))
		print(i)
		#time.sleep(1)
		cv2.destroyAllWindows()
		'''
		img = cv2.imread(strf)
		if _img_shape == None:
			_img_shape = img.shape[:2]
		else:
			assert _img_shape == img.shape[:2], "All images must share the same size."
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
		# If found, add object points, image points (after refining them)
		print(i)
		if ret == True:
			print("ok")
			objpoints.append(objp)
			cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
			imgpoints.append(corners)
		else:
			print("nok")
		'''