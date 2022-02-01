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
from Zones_Strategy import dict_zones


TOPIC_BIG_BOT = "BigBot"

def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("connected OK Returned code=",rc)
        client.subscribe(TOPIC_BIG_BOT + "/2")
    else:
        print("Bad connection Returned code=",rc)


def on_message(client, userdata, message):
    global data_id
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

StartX = dict_zones['Start'][0]
StartY = dict_zones['Start'][1]
GallerieRougeX = dict_zones['Galerie_Rouge'][0]
GallerieRougeY = dict_zones['Galerie_Rouge'][1]
GallerieVertX = dict_zones['Galerie_Vert'][0]
GallerieVertY = dict_zones['Galerie_Vert'][1]
GallerieBleuX = dict_zones['Galerie_Bleu'][0]
GallerieBleuY = dict_zones['Galerie_Bleu'][1]

DistribMatX = dict_zones['DispenserMat'][0]
DistribMatY = dict_zones['DispenserMat'][1]

CampementX = dict_zones['Campement'][0]
CampementY = dict_zones['Campement'][1]


if __name__ == '__main__':
    JeanMichelDuma = Robot()
    JeanMichelDuma.DEBUG = 0
    JeanMichelDuma.orientationZ = 30
    _thread.start_new_thread( data_Thread, (1 ,) )
    print("[DEBUG	] Thread MQTT Started")
    client.on_connect = on_connect
    client.on_message = on_message
    #JeanMichelDuma.speed = JeanMichelDuma.dict_speed['Medium']
    targetX = dict_zones['DispenserMat'][0] / 10
    targetY = dict_zones['DispenserMat'][1] / 10
    print(f"Target = {targetX} , {targetY}")
    targetAngle = 30
    time.sleep(.5)
    while(True):
        try:
            while(not JeanMichelDuma.setOrientation(270,3)):
                print(JeanMichelDuma.orientationZ)
                pass
            distance = abs(JeanMichelDuma.positionX - GallerieBleuX)
            steps = stepsFromCm(distance)*2
            JeanMichelDuma.goForward(steps)
            startX = JeanMichelDuma.positionX
            startY = JeanMichelDuma.positionY
            while(not JeanMichelDuma.monitorSteps(startX,startY,GallerieBleuX,GallerieBleuY) ):
                pass
            while(not JeanMichelDuma.setOrientation(0,4)):
                pass
            exit()
            """ while(not JeanMichelDuma.goToNewVersion(targetX,targetY)):
                JeanMichelDuma.goToDistributeur() """
        except:
            e = sys.exc_info()[0]
            print(e)
            JeanMichelDuma.stopMotors()
            exit()

