#!/usr/bin/python
from Robot import *
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv2 import aruco
import math
import _thread
import paho.mqtt.client as mqtt
import json


TOPIC_BIG_BOT = "BigBot"

def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("connected OK Returned code=",rc)
        client.subscribe(TOPIC_BIG_BOT + "/2")
    else:
        print("Bad connection Returned code=",rc)


def on_message(client, userdata, message):
    global data_id
    global data_id_topic
    global data_rvec
    global data_tvec
    global JeanMichelDuma
    msg = message.payload.decode("utf-8")
    msg = json.loads(msg)
    if ( msg["rz"] == JeanMichelDuma.orientationZ ):
        return
    if(message.topic == (TOPIC_BIG_BOT+"/2")):
        JeanMichelDuma.positionX = msg["x"]
        JeanMichelDuma.positionY = msg["y"]
        JeanMichelDuma.orientationZ = msg["rz"]
        #print("X Y RZ + " + str(msg["x"]) +  str(msg["y"]) +  str(msg["rz"]))

C_IP_MQTT = "172.30.40.68"
client = mqtt.Client()
client.on_connect = on_connect

def data_Thread(theadID):
    while True:
        print('[DEBUG	] Connecting to the TTN Broker...')
        print(client.connect(C_IP_MQTT, 1883, 60))
        client.loop_forever()


""" x
^
|
|
0----->y """


if __name__ == '__main__':
    JeanMichelDuma = Robot()
    JeanMichelDuma.DEBUG = 0
    _thread.start_new_thread( data_Thread, (1 ,) )
    print("[DEBUG	] Thread MQTT Started")
    client.on_connect = on_connect
    client.on_message = on_message
    JeanMichelDuma.speed = JeanMichelDuma.dict_speed['Slow']
    while(True):
        try:
            #if(JeanMichelDuma.setOrientation(340)): test la rotation
            if(JeanMichelDuma.goToUsingLocation(105,150)):
                print("steaup")
                JeanMichelDuma.stopMotors()
                exit()
            else:
                pass
                #time.sleep(0.15)#
                #print("Not detected")
                #JeanMichelDuma.stopMotors()
        except:
            JeanMichelDuma.stopMotors()
            exit()

