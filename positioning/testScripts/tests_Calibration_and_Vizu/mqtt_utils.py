import math
import numpy as np
import cv2
import paho.mqtt.client as mqtt
import _thread
import json
from picamera.array import PiRGBArray
from picamera import PiCamera
import threading
from cv2 import aruco
from pylab import *
import time
import math 
from dictionary import *
from TableDrawer import *
from mqtt_utils import *



def on_connect(client, userdata, flags, rc):
	print("Connected with result code ")


client = mqtt.Client()
client.on_connect = on_connect
C_IP_MQTT = "172.30.40.68"


def initClient():
	_thread.start_new_thread( data_Thread, (1 ,) )

def data_Thread(theadID):
	while True:
		print('[DEBUG	] Connecting to the TTN Broker...')
		#client.connect("192.168.0.13", 1883, 60)
		print(client.connect(C_IP_MQTT, 1883, 60))
		client.loop_forever()

def createPayload(id,x,y,z,rz):
	marker = {}
	marker["id"]= int(id)
	marker["x"]= int(x+150) #relative to the border
	marker["y"]= int(125.0-y) #relative to the border
	marker["z"] = int(z)
	marker["rz"] = int(rz)
	payload_json = json.dumps(marker)
	return payload_json


def mqtt_pubData(ids,tvecs,rvecs):
	for k in dict_sizes:
		j=0
		for i in range(0,len(ids)):
			if(ids[i][0] == k):
				j = j + 1
				payload_json = createPayload(ids[i][0],tvecs[i][0],tvecs[i][1],tvecs[i][2],rvecs[i][2])
				client.publish(f"data/{ids[i][0]}_{j}", payload=payload_json, qos=0, retain=False)
	
	robotPose = findRobotCenter(ids,tvecs,rvecs)
	#print(foundBot)
	if(robotPose != False and robotPose != None ):
		
		x,y,a = robotPose
		x = int((x))
		y = int((y))
		payload_json = createPayload(99,x,y,0,a)
		client.publish("BigBot/2", payload=payload_json, qos=0, retain=False)
	'''
	try:
		idx1 = ids.index([1])
		idx2 = ids.index([2])
		x,y,a = determineMidpoint(tvecs[idx1],tvecs[idx2])
		z = (tvecs[idx1][2]+tvecs[idx2][2])/2
		payload_json = createPayload(99,x,y,z,a)
		client.publish("BigBot/2", payload=payload_json, qos=0, retain=False)
	except:
		try:
			i = ids.index([1])
			payload_json = createPayload(ids[i][0],tvecs[i][0],tvecs[i][1],tvecs[i][2],rvecs[i][2])
			client.publish("BigBot/2", payload=payload_json, qos=0, retain=False)
		except:
			try:
				i = ids.index([2])
				payload_json = createPayload(ids[i][0],tvecs[i][0],tvecs[i][1],tvecs[i][2],rvecs[i][2])
				client.publish("BigBot/2", payload=payload_json, qos=0, retain=False)
			except:
				pass
		pass
	'''

def determineMidpointAndAngle180(point1,point2):
	x1 = point1[0]
	y1 = point1[1]
	x2 = point2[0]
	y2 = point2[1]
	diff_vect = [point1[0] - point2[0],point1[1] - point2[1]]
	midpoint_X = point1[0]-diff_vect[0]/2
	midpoint_Y = point1[1]-diff_vect[1]/2
	angle = math.degrees(math.atan(diff_vect[1]/diff_vect[0])) #todo add a try if 0/0
	return midpoint_X, midpoint_Y, angle


#FL FR RL RR

def findRobotCenter(ids,tvecs,rvecs):
	foundTagsBot = []
	for tagID in tagsOnRobot:
		if(tagID in ids):
			foundTagsBot.append(ids[ids.index(tagID)])
	if(foundTagsBot != []):
		#check si une paire de tags est détectée
		for i in range(0,len(pairsTag)):
			if(pairsTag[i][0] in foundTagsBot and pairsTag[i][1] in foundTagsBot):#Si on a trouvé 2 tags qui vont de paire
					#print("found bot wit pair [",pairsTag[i][0],",",pairsTag[i][1],"]")
					idx_tag1 = ids.index(pairsTag[i][0])
					idx_tag2 = ids.index(pairsTag[i][1])
					x,y,a = getRobotPoseFrom2tags(tvecs[idx_tag1],tvecs[idx_tag2])
					a = (a+anglePairTag[i])%360
					a = (180 - a) % 360
					#print(150+x,125-y,a)
					return x,y,a
		#ici on va check si on trouve le bot avec juste un des tags
	else:
		return False


def getRobotPoseFrom2tags(tag_gauche,tag_droit):
	#Calcul de la position milieu entre 2 tags
	mid_x,mid_y,angle = determineMidpointAndAngle180(tag_gauche,tag_droit)
	if((tag_gauche[0])<(tag_droit[0])):#tag 1 à gauche : 0,180
		angle = angle+90
	else:
		angle = angle+270
	
	lenRobot = 22
	lenTag = 7
	d = lenRobot/2-lenTag/2
	x_robot=mid_x + d*math.cos(math.radians(angle))
	y_robot=mid_y + d*math.sin(math.radians(angle))
	return x_robot, y_robot, angle