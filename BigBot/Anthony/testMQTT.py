import json
from enum import IntEnum
import paho.mqtt.client as mqtt
import _thread
import time
import serial
from Robot import *

C_IP_MQTT = "172.30.40.20"

client = mqtt.Client() 
robotX = 0
robotY = 0
tag42X = 0
tag42Y = 0

newMessage = False


PORT = "/dev/ttyUSB0"

ser = serial.Serial (PORT, 
        baudrate = 115200)

def serialWriteReg(reg):
	try: 
		reg = reg.to_bytes(1,'big')
		ser.write(reg)
	except:
		print("ERROR USB")
		if(ser.isOpen()):
			ser.write(0)
			ser.close()
		else:
			ser.open() 


def on_message(client, userdata, message):
    global newMessage
    global robotX,robotY
    global tag42X,tag42Y
    if(newMessage == False):
        msg = message.payload.decode("utf-8")
        #print("message received " ,str(msg))
        #print("message topic=",message.topic)
        msg = json.loads(msg)
        if(msg["id" ] == 42):
            tag42X = msg["x"]
            tag42Y = msg["y"]
            #print("Tag 42 X Y  = " + str(tag42X) + " " + str(tag42Y))
        elif(msg["id" ] == 17):
            robotX = msg["x"]
            robotY = msg["y"]
            #print("ROBOT X Y  = " + str(robotX) + " " + str(robotY))
        newMessage = True
    else:
        print("RONPICH")
        time.sleep(0.1)




def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("connected OK Returned code=",rc)
        client.subscribe("data/#")
        newMessage = False
    else:
        print("Bad connection Returned code=",rc)
        
def data_Thread(theadID):
    while True:
        print('[DEBUG	] Connecting to the TTN Broker...')
        #client.connect("192.168.0.13", 1883, 60)
        while( client.connect(C_IP_MQTT, 1883, 60) != 0):
            pass#client.subscribe("data/42")
        print("connected")
        client.loop_forever()


if __name__ == '__main__':
    
    client.on_message = on_message
    client.on_connect = on_connect
    _thread.start_new_thread( data_Thread, (1 , ) )
    offset_max_x = 10
    offset_max_y = 2
    offset_max_angle = 5


    
    while(True):
        if(newMessage == True):
            reg = 0
            print("Tag 42 X Y  = " + str(tag42X) + " " + str(tag42Y))
            print("ROBOT X Y  = " + str(robotX) + " " + str(robotY))
            distX = robotX - tag42X
            distY = robotY - tag42Y
            print("distX = " + str(distX) + " distY = " + str(distY))
            if(abs(distX) > offset_max_x):
                reg = goForward()
                print("goForward")
            elif(abs(distY) > offset_max_y):
                if(distY < 0):
                    reg = goRight()
                    print("goRight")
                else:
                    reg = goLeft()
                    print("goLeft")
            else:
                reg = stopMotors()
            serialWriteReg(reg)
            newMessage = False
            



    


