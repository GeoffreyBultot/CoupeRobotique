import numpy as np
import cv2
from pylab import *
import math 
from dictionary import *
from TableDrawer import *

def draw_rectangle(image, centre, colour ,theta, width, height):
	theta = np.radians(theta)
	c, s = np.cos(theta), np.sin(theta)
	R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
	# print(R)
	p1 = [ + width / 2,  + height / 2]
	p2 = [- width / 2,  + height / 2]
	p3 = [ - width / 2, - height / 2]
	p4 = [ + width / 2,  - height / 2]
	p1_new = np.dot(p1, R)+ centre
	p2_new = np.dot(p2, R)+ centre
	p3_new = np.dot(p3, R)+ centre
	p4_new = np.dot(p4, R)+ centre
	img = cv2.line(image, (int(p1_new[0, 0]), int(p1_new[0, 1])), (int(p2_new[0, 0]), int(p2_new[0, 1])), colour, 1)
	img = cv2.line(img, (int(p2_new[0, 0]), int(p2_new[0, 1])), (int(p3_new[0, 0]), int(p3_new[0, 1])), colour, 1)
	img = cv2.line(img, (int(p3_new[0, 0]), int(p3_new[0, 1])), (int(p4_new[0, 0]), int(p4_new[0, 1])), colour, 1)
	img = cv2.line(img, (int(p4_new[0, 0]), int(p4_new[0, 1])), (int(p1_new[0, 0]), int(p1_new[0, 1])), colour, 1)
	#img = cv2.line(img, (int(p2_new[0, 0]), int(p2_new[0, 1])), (int(p4_new[0, 0]), int(p4_new[0, 1])), colour, 1)
	#img = cv2.line(img, (int(p1_new[0, 0]), int(p1_new[0, 1])), (int(p3_new[0, 0]), int(p3_new[0, 1])), colour, 1)

	return img

def drawInTableJeu(ids,tvecs,rvecs):
	width = 1080
	height = 700
	widthCentimeters = 300
	heightCentimeters = 200
	scale_width = width/widthCentimeters
	scale_height = height/heightCentimeters
	frame = cv2.imread('tableJeu.jpg') 
	frame = cv2.rotate(frame, cv2.ROTATE_180)
	frame = cv2.resize(frame,(width,height))
	idx_42 = ids.index([42])
	tvec42 = tvecs[idx_42]
	for i in range(0,len(tvecs)):
		tvec_Marker = tvecs[i]
		rvec_Marker = rvecs[i]
		tvec_Marker = [tvec42[0] - tvec_Marker[0],tvec42[1] - tvec_Marker[1],tvec42[2] - tvec_Marker[2]]
		x = int((150+tvec_Marker[0])*scale_width)
		y = int((heightCentimeters-125-tvec_Marker[1])*scale_height)
		#print(x,y) 
		id = ids[i][0]
		z_angle = rvec_Marker[2]
		frame = cv2.circle(frame,(x,y), radius=5,color=dictColors[id], thickness = -1)
		#text = f'{id} [{int(tvec_Marker[0][0,0])},{int(tvec_Marker[1][0,0])}]'
		text = f'{id} [{int(tvec_Marker[0])},{int(tvec_Marker[1])}]'
		frame = cv2.putText(frame, text, (x-50,y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,0),  1, cv2.LINE_AA,False)
		frame = draw_rectangle(frame,(x,y), dictColors[id],z_angle, dict_sizes[id]*scale_width ,dict_sizes[id]*scale_width)
	return frame
