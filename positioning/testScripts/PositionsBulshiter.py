import paho.mqtt.client as mqtt
import threading
import time
import json
from random import *
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))

	# Subscribing in on_connect() means that if we lose the connection and
	# reconnect then subscriptions will be renewed.
	client.subscribe("game/2")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
	print(msg.topic+" "+str(msg.payload))


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
		print('[DEBUG	] Connecting to the MQTT Broker...')
		self.client.connect("192.168.0.4", 1883, 60)
		print("[DEBUG	] Starting " + self.name)
		self.client.loop_forever()
		
	def disconnect(self):
		self.client.disconnect()
		print("Exiting " + self.name)
		
# Let's see if you inserted the required data
if __name__ == '__main__':
	dataThread = data_Thread(1, "MQTT Thread", 1)
	dataThread.daemon = True
	dataThread.start()
	print("[DEBUG	] Thread Started")
	while True:
		maxLight = 255
		time.sleep(2)
		position = {}
		position = {}
		position["x"] = 44
		position["y"] = 44
		position["orientation"] = 0
		payload_json = json.dumps(position)
		dataThread.client.publish("games/togo", payload=payload_json, qos=0, retain=False)

		position["x"]= 0
		position["y"]= 0
		position["orientation"] = 0
		position["angle2pos"] = 45
		payload_json = json.dumps(position)
		dataThread.client.publish("games/mainBot", payload=payload_json, qos=0, retain=False)
		
		
		
		time.sleep(1)
		position["x"]= 44
		position["y"]= 44
		position["orientation"] = 0
		position["angle2pos"] = 45
		payload_json = json.dumps(position)
		dataThread.client.publish("games/mainBot", payload=payload_json, qos=0, retain=False)
		time.sleep(5);