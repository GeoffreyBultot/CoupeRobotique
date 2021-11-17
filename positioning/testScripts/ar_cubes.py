import cv2
import cv2.aruco as aruco
import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

# Defines the path of the calibration file and the dictonary used
calibration_path = "realsense_d435.npz"
dictionary = aruco.DICT_4X4_100


DIM=(2592, 1944)

mtx=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
#Distorsion matrix
dist=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]])

camera = PiCamera()
#camera.rotation = 180
#camera.resolution = (1920, 1080)
camera.resolution = (2592, 1944)

camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)


def intUndis():
	global map1
	global map2
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(mtx, dist, np.eye(3), mtx, DIM, cv2.CV_16SC2)

def undistort(img, balance=0, dim2=None, dim3=None):
	#img = cv2.resize(img,DIM)
	dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
	assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
	if not dim2:
		dim2 = dim1
	if not dim3:
		dim3 = dim1
	scaled_K = mtx * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
	scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
	new_K = None #cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
	undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)#, borderMode=cv2.BORDER_CONSTANT)
	return(undistorted_img)

intUndis()
for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):	# Get frame from realsense and convert to grayscale image
	img_rgb = frame_pi.array
	img_rgb =  undistort(img_rgb, balance=0, dim2=None, dim3=None)

	img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
	
	# Detect markers on the gray image
	res = aruco.detectMarkers(img_gray, aruco.getPredefinedDictionary(dictionary))
	
	# Draw each marker 
	for i in range(len(res[0])):
		# Estimate pose of the respective marker, with matrix size 1x1
		rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(res[0][i], 1, mtx, dist)
		
		# Define the ar cube
		# Since we previously set a matrix size of 1x1 for the marker and we want the cube to be the same size, it is also defined with a size of 1x1x1
		# It is important to note that the center of the marker corresponds to the origin and we must therefore move 0.5 away from the origin 
		axis = np.float32([[-0.5, -0.5, 0], [-0.5, 0.5, 0], [0.5, 0.5, 0], [0.5, -0.5, 0],
						   [-0.5, -0.5, 1], [-0.5, 0.5, 1], [0.5, 0.5, 1],[0.5, -0.5, 1]])
		# Now we transform the cube to the marker position and project the resulting points into 2d
		imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
		imgpts = np.int32(imgpts).reshape(-1, 2)

		# Now comes the drawing. 
		# In this example, I would like to draw the cube so that the walls also get a painted
		# First create six copies of the original picture (for each side of the cube one)
		side1 = img_rgb.copy()
		side2 = img_rgb.copy()
		side3 = img_rgb.copy()
		side4 = img_rgb.copy()
		side5 = img_rgb.copy()
		side6 = img_rgb.copy()
		
		# Draw the bottom side (over the marker)
		side1 = cv2.drawContours(side1, [imgpts[:4]], -1, (255, 0, 0), -2)
		# Draw the top side (opposite of the marker)
		side2 = cv2.drawContours(side2, [imgpts[4:]], -1, (255, 0, 0), -2)
		# Draw the right side vertical to the marker
		side3 = cv2.drawContours(side3, [np.array(
			[imgpts[0], imgpts[1], imgpts[5],
			 imgpts[4]])], -1, (255, 0, 0), -2)
		# Draw the left side vertical to the marker
		side4 = cv2.drawContours(side4, [np.array(
			[imgpts[2], imgpts[3], imgpts[7],
			 imgpts[6]])], -1, (255, 0, 0), -2)
		# Draw the front side vertical to the marker
		side5 = cv2.drawContours(side5, [np.array(
			[imgpts[1], imgpts[2], imgpts[6],
			 imgpts[5]])], -1, (255, 0, 0), -2)
		# Draw the back side vertical to the marker
		side6 = cv2.drawContours(side6, [np.array(
			[imgpts[0], imgpts[3], imgpts[7],
			 imgpts[4]])], -1, (255, 0, 0), -2)
		
		# Until here the walls of the cube are drawn in and can be merged
		img_rgb = cv2.addWeighted(side1, 0.1, img_rgb, 0.9, 0)
		img_rgb = cv2.addWeighted(side2, 0.1, img_rgb, 0.9, 0)
		img_rgb = cv2.addWeighted(side3, 0.1, img_rgb, 0.9, 0)
		img_rgb = cv2.addWeighted(side4, 0.1, img_rgb, 0.9, 0)
		img_rgb = cv2.addWeighted(side5, 0.1, img_rgb, 0.9, 0)
		img_rgb = cv2.addWeighted(side6, 0.1, img_rgb, 0.9, 0)
		
		# Now the edges of the cube are drawn thicker and stronger
		img_rgb = cv2.drawContours(img_rgb, [imgpts[:4]], -1, (255, 0, 0), 2)
		for i, j in zip(range(4), range(4, 8)):
			img_rgb = cv2.line(img_rgb, tuple(imgpts[i]), tuple(imgpts[j]), (255, 0, 0), 2)
		img_rgb = cv2.drawContours(img_rgb, [imgpts[4:]], -1, (255, 0, 0), 2)
		
	# Display the result
	cv2.imshow("AR-Example", cv2.resize(img_rgb,(1920,1000)))
	rawCapture.truncate(0)
	# If [ESC] pressed, close the application
	if cv2.waitKey(100) == 'q':
		print("Application closed")
		break
# Close all cv2 windows
cv2.destroyAllWindows()