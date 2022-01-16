import paho.mqtt.client as mqtt
import _thread
import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video import FPS
from cv2 import aruco
import matplotlib.pyplot as plt
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import json
import time 
import os

C_t_refresh_Seconds = 0.00
client = mqtt.Client()

#Labo avec 30 photos 2592 x 1944 le 15/11
DIM=(2592, 1944)
#Camera matrix
K=np.array([[1271.6340818922563, 0.0, 1167.4678127068892], [0.0, 1267.583299646622, 938.5488313394765], [0.0, 0.0, 1.0]])
#Distorsion matrix
D=np.array([[-0.08022559999087736], [0.10435020556133874], [-0.11171079602705103], [0.03853140815187616]])


def data_Thread(theadID):
	while True:
		print('[DEBUG	] Connecting to the TTN Broker...')
		client.connect("192.168.0.13", 1883, 60)
		#client.connect("172.30.40.67", 1883, 60)
		client.loop_forever()

#vid = cv2.VideoCapture(0) # define a video capture object

def mqtt_pubData(ids,corners):
	userdata = []
	for i in range(0,len(ids)):
		marker = {}
		marker["id"]= int(ids[i][0])
		marker["x"]= int(corners[i][0][0][0])
		marker["y"]= int(corners[i][0][0][1])
		userdata.append(marker)
	payload_json = json.dumps(userdata)
	#client.publish("data", payload=payload_json, qos=0, retain=False)





def inversePerspective(rvec, tvec):
	R, _ = cv2.Rodrigues(rvec)
	R = np.matrix(R).T
	invTvec = np.dot(-R, np.matrix(tvec))
	invRvec, _ = cv2.Rodrigues(R)
	return invRvec, invTvec

def relativePosition(rvec1, tvec1, rvec2, tvec2):
	rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape(
		(3, 1))
	rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

	# Inverse the second marker, the right one in the image
	invRvec, invTvec = inversePerspective(rvec2, tvec2)

	orgRvec, orgTvec = inversePerspective(invRvec, invTvec)
	# print("rvec: ", rvec2, "tvec: ", tvec2, "\n and \n", orgRvec, orgTvec)

	info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
	composedRvec, composedTvec = info[0], info[1]

	composedRvec = composedRvec.reshape((3, 1))
	composedTvec = composedTvec.reshape((3, 1))
	return composedRvec, composedTvec

def computeMcalibMatrix(rvec42,tvec42):
	M_calib    = np.matrix(cv2.Rodrigues(rvec42)[0])
	newrow = [[0,0,0,1]]
	tvecCol = np.reshape(tvec42, (3, 1))
	M_calib=np.c_[M_calib,tvecCol]
	M_calib = np.vstack((M_calib, newrow))
	return M_calib

def changeReference(tvec,matrix):
	tvec = np.reshape(tvec, (3, 1))
	tvec = np.vstack((tvec,[1]))
	#print(tvec)
	
	return (inv(matrix) * tvec)


#--- initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
#camera.rotation = 180
camera.iso = 1600 # max ISO to force exposure time to minimum to get less motion blur
camera.contrast = 100
camera.image_denoise = True
#camera.saturation = 100
#camera.exposure_mode = "auto"
#camera.image_effect = "negative"
#camera.resolution = (1280, 960)
camera.resolution = (2592, 1944) 
#camera.resolution = (1640, 1232)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=camera.resolution)

#--- start imutils fps counter
fps = FPS().start()

if __name__ == '__main__':
	#start the MQTT thread
	#_thread.start_new_thread( data_Thread, (1,"Thread-1", 2) )
	#print("[DEBUG	] Thread MQTT Started")
	timeStart = 0
	#TODO resolution to define
	#change_res(1280, 960)  #https://www.codingforentrepreneurs.com/blog/open-cv-python-change-video-resolution-or-scale
	#change_res(2592, 1944)  #https://www.codingforentrepreneurs.com/blog/open-cv-python-change-video-resolution-or-scale
	#change_res(1920, 1080) #https://www.codingforentrepreneurs.com/blog/open-cv-python-change-video-resolution-or-scale

	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250 )
	parameters =  aruco.DetectorParameters_create()
	
	#while(True):
	M_calib = None
	
	for frame_pi in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		frame = frame_pi.array
		t1 = time.time()
		#pathent = "./images/Labeau/test7.png"
		gray = cv2.cvtColor((frame), cv2.COLOR_BGR2GRAY)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		frame = aruco.drawDetectedMarkers(frame, corners)
		os.system('clear')
		if ids is not None:
			fig = plt.figure(figsize=(4,4))
			ax = fig.add_subplot(111, projection='3d')
			#ax = fig.add_subplot(111)
			for i in range(0,len(ids)):
				if(ids[i][0] == 42):
					markerSizeInCM = 9.6
				else:
					markerSizeInCM = 7
				rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], markerSizeInCM, K, D)
				str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0][0][0], tvec[0][0][1], tvec[0][0][2])
				font = cv2.FONT_HERSHEY_PLAIN
				cv2.putText(frame, str_position, (0, (i+1)*100), font, 4, (0, 255, 0), 2, cv2.LINE_AA)
				#-- Obtain the rotation matrix tag->camera
				if(ids[i][0]==42):
					M_calib = computeMcalibMatrix(rvec[0][0],tvec[0][0])
					print("id = "+str(ids[i][0]))
					'''
					vect = changeReference(tvec[0][0],M_calib)
					ax.scatter(vect[0,0],vect[1,0],vect[2,0],label = str(ids[i][0]))
					print(vect)
					'''
					tvec =tvec[0][0]
					ax.scatter(tvec[0],tvec[1],tvec[2],label = str(ids[i][0]))
					#print(np.reshape(changeReference(tvec[0][0],M_calib),(1, 4)))
				else:
					if M_calib is not None:
						print("id = "+str(ids[i][0]))
						vect = changeReference(tvec[0][0],M_calib)
						'''
						print(vect)
						#vect = (np.reshape(changeReference(tvec[0][0],M_calib),(1, 4)))[0][0]
						ax.scatter(vect[0,0],vect[1,0],vect[2,0],label = str(ids[i][0]))
						'''
						tvec =tvec[0][0]
						ax.scatter(tvec[0],tvec[1],tvec[2],label = str(ids[i][0]))
			ax.legend()
			ax.grid(True)
			plt.show()
			time.sleep(1)
			#aruco.drawAxis(frame, K, D, rvec[i], tvec[i], 20)
			for j in range(0, len(ids)):
				line_thickness = 1
				if(ids[j][0] == 42):
					#dessine le milieu de chaque tag
					corner_42 = corners[j][0]
					M = cv2.moments(corner_42)
					x_sum = int(M["m10"] / M["m00"])
					y_sum = int(M["m01"] / M["m00"])
					frame = cv2.circle(frame, (int(x_sum),int(y_sum)), radius=5, color=(0, 0, 255), thickness=2)
					#corners_abcd = corner_42.reshape((4, 2))
					
					#TODO 
					'''
					# Computing rotation matrix
					rotation_matrix = np.zeros(shape=(3,3))
					cv2.Rodrigues(rvec[i], rotation_matrix)
					#Apply rotation matrix to point
					original_point = tvec[i] #np.matrix([[1],[0],[0]])
					rotated_point = rotation_matrix*original_point
					'''
			mqtt_pubData(ids,corners)			
			
			
		#frame = cv2.resize(frame, (960, 540))
		#frame = cv2.imread("1")
		#cv2.imshow("Img",cv2.resize(frame,(720,480)))
		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)

		fps.update()
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print("fps " + str(1/(time.time()-t1)))
		
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))        
# Destroy all the windows
cv2.destroyAllWindows()