import math
import numpy as np
import cv2
import paho.mqtt.client as mqtt
from cv2 import aruco
from pylab import *
import math 
from dictionary import *
from TableDrawer import *
import paho.mqtt.client as mqtt
import threading
import json

def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))

	# Subscribing in on_connect() means that if we lose the connection and
	# reconnect then subscriptions will be renewed.
	client.subscribe("data/#")

data_id = []
data_id_topic = []
data_rvec = []
data_tvec = []





# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, message):
	global data_id
	global data_id_topic
	global data_rvec
	global data_tvec
	msg = message.payload.decode("utf-8")
	#print("message received " ,str(msg))
	#print("message topic=",message.topic)
	msg = json.loads(msg)
	id_this_Topic = message.topic.split("/")[1]
	id_tag = int(id_this_Topic.split("_")[0])
	tvec = [msg["x"],msg["y"]]
	rotation = msg["rz"]
	
	if(msg["id" ] == 42):
		tag42X = msg["x"]
		tag42Y = msg["y"]

	if(not id_this_Topic in data_id_topic):
		data_id.append(id_tag)
		data_id_topic.append(id_this_Topic)
		data_rvec.append(rotation)
		data_tvec.append(tvec)
	else:
		index = data_id_topic.index(id_this_Topic)
		data_id[index]   = id_tag
		data_tvec[index] = tvec
		data_rvec[index] = rotation
		
class data_Thread (threading.Thread):
	def __init__(self, threadID, name, counter):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.counter = counter

	def run(self):
		self.client = mqtt.Client()
		#self.client.username_pw_set(TTN_USERNAME, password=TTN_PASSWORD)
		self.client.on_connect = on_connect
		self.client.on_message = on_message
		print('[DEBUG	] Connecting to the TTN Broker...')
		self.client.connect("192.168.1.39", 1883, 60)
		#self.client.connect("192.168.1.105", 1883, 60)
		#self.client.connect("192.168.1.106", 1883, 60)
		print("[DEBUG	] Starting " + self.name)
		self.client.loop_forever()
		
	def disconnect(self):
		self.client.disconnect()
		print("Exiting " + self.name)


if __name__ == '__main__':
	dataThread = data_Thread(1, "TTN Thread", 1)
	dataThread.daemon = True
	dataThread.start()
	print("[DEBUG	] Thread Started")
	while(True):
		if(data_id != []):
			frame = drawInTableJeu(data_id,data_tvec,data_rvec)
			cv2.imshow("vizu Plato", frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				cv2.destroyAllWindows()
				exit(0)
