import serial
import math
import time
import sys
import numpy as np
from .utils import computeDict,_set_motor
import _thread

#TODO Calibration avec item dans le chasse neige

""" 
x
^
|
|
0----->y
 """
#region Création objet




class Robot:
	posArray = [[]]
	dict_speed = {
		'Very slow' : 128,
		'Slow' : 64,
		'Medium' : 32,
		'Fast' : 8,
	}


	def __init__(self):
		
		self.positionX = 0
		self.positionY = 0
		self.orientationZ = 0
		self.offsetX = 2.8
		self.offsetY = 5.2 #la camera est 5.2cm à droite du centre du robot
		self.deadBandY = 1
		self.deadBandAngle = 8
		self.reg = 0
		self.PORT = "/dev/tty_ARDUINO_USB"
		self.ser = serial.Serial (self.PORT, baudrate = 115200)
		self.DEBUG = True
		self.dict = computeDict()
		self.speed = self.dict_speed['Very slow']
		#self.posArray = self.initArray
		#_thread.start_new_thread( data_Thread, (1 , ) )

	def setSerial(self,port,baudrate = 115200):
		if(self.ser.isOpen()):
			self.ser.close()
		self.ser = serial.Serial(port,baudrate) 

	def serialWriteReg(self):
		try: 
			start = 123
			reg = self.reg
			divPWM = self.speed
			stop = 253
			array = [start,reg,divPWM,stop]
			message = bytearray(array)	
			self.ser.write(message)
		except:
			e = sys.exc_info()[0]
			print(e)
			if(self.ser.isOpen()):
				self.ser.write(0)
				self.ser.close()
			else:
				self.ser.open() 
	""" def serial_Thread():
		while True:
			print('[DEBUG	] Start thread reading serial')
			client.loop_forever() """
#endregion


#region NEW CODE

	def goToSelfCamera(self,targetXYZ,targetAngle): #return 1 si on est arrivé, sinon 0
		targetX = targetXYZ[0]
		targetY = targetXYZ[1]
		targetZ = targetXYZ[2]
		dist = math.sqrt(targetX**2 + targetY**2 + targetZ**2)
		targetY = targetY + self.offsetY
		print("Target ANGLE = ",targetAngle)
		angle_normalized = targetAngle%60
		print("Target X = " + str(targetX) + "Target Y = " + str(targetY))
		print("Angle = " + str(targetAngle))
		print("Dist = ",dist)

		if(dist > 60):
			self.approachTarget(targetX,targetY)
			print("APPROACHING")

		elif(angle_normalized > 30+self.deadBandAngle or angle_normalized <30 - self.deadBandAngle): #corrige l'angle
			self.speed = self.dict_speed['Very slow']
			self.correctAngle(targetAngle)
			print("RECTIFYING ANGLE")
				
		elif(abs(targetY) > self.deadBandY): #s'aligne 
			print("ALIGNING")
			self.speed = self.dict_speed['Slow']
			if(targetY < 0):
				self.goLeft()
			else:
				self.goRight()
		elif(targetX > self.offsetX): #approche finale tout droit
			if(targetX > 8):
				self.speed = self.dict_speed['Medium']
			else:
				self.speed = self.dict_speed['Slow']			
			print("BRRRR")
			self.goForward()
		else: #arrived
			self.block()
			return 1
		return 0
	
	def correctAngle(self,angle):
		if abs(angle) > 30+self.deadBandAngle:#Normal
			angle = abs(angle%60)
			if(angle > 30+self.deadBandAngle):
				print("RotateRight")
				self.rotationRight()
			elif(angle <30 - self.deadBandAngle):
				print("RotateLeft")
				self.rotationLeft()
		else: #Si angle [-38 ; 38 ]
			if angle>0:
				if (angle>30+self.deadBandAngle or angle<30-self.deadBandAngle):
					print("RotateLeft")
					self.rotationLeft()
			elif (angle<-30-self.deadBandAngle or angle>-30+self.deadBandAngle):
				print("RotateRight")
				self.rotationRight()


		
	def approachTarget(self,targetX,targetY): #s'approche de la position en s'alignant d'abord en Y et puis en avançant
		if(abs(targetY) > self.deadBandY + 5):
			self.speed = self.dict_speed['Slow']
			if(targetY < 0):
				self.goLeft()
			else:
				self.goRight()
		else:
			self.speed = self.dict_speed['Fast']
			self.goForward()
		
	def goToUsingLocation(self,targetX,targetY, offset_max_distance = 40,offset_max_angle = 10):
		deltaX = self.positionX - targetX
		deltaY = self.positionY - targetY
		distance = math.sqrt(deltaX**2 + deltaY**2)
		print("Distance to Target = " +str(distance))
		if (deltaY == 0):
			deltaY = 0.01
		angleToTarget =  math.degrees(math.atan(deltaX/deltaY)) -90
		print("Current Angle = " + str(self.orientationZ))
		print("Angle To Target = " + str(angleToTarget))
		if(abs(angleToTarget - self.orientationZ) > offset_max_angle): #orientation vers target
			if(self.orientationZ > angleToTarget):
				self.rotationLeft()
				print("Left")
			else:
				self.rotationRight()
				print("Right")	
		elif(distance > offset_max_distance):
			self.goForward()
		return distance


	def updatePos(self,posX,posY):
		self.posArray.append([posX,posY])
		if(len(self.posArray) > 10):
			self.posArray.pop(0)
			
		
	 

#endregion

#region DEBUG

	def goToDebugAligned(self,targetX,targetY, offset_max_distance_x = 15,offset_max_distance_y = 5):
		deltaX = self.positionX - targetX
		deltaY = self.positionY - targetY
		if(deltaX > offset_max_distance_x):
			self.goForward()
		elif(deltaX < -offset_max_distance_x):
			self.goBackwards()
		elif(deltaY > offset_max_distance_y):
			self.goLeft()
		elif(deltaY < -offset_max_distance_y):
			self.goRight()
		distance = deltaX**2 + deltaY**2
		return distance

	def debugIndivMotors(self):
		print("Debugging Motors BRRRRRRR")
		while(True):
			#for motor in self.Emotor:
			for motor in range(0,4):
				print(motor)
				self.reg = _set_motor(motor,0,1)
				self.serialWriteReg()
				time.sleep(1)
				self.reg = _set_motor(motor,1,1)
				self.serialWriteReg()
				time.sleep(1)

	def debugDirections(self):
		while(True):
			for key in self.dict:
				self.reg = self.dict[key]
				print(key, '->', self.dict[key])
				self.serialWriteReg()
				time.sleep(3)
			
	def goForward(self):
		self.reg = self.dict['forward']
		self.serialWriteReg()
	def goBackward(self):
		self.reg = self.dict['backward']
		self.serialWriteReg()
	def stopMotors(self):
		self.reg = self.dict['stop']
		self.serialWriteReg()
	def rotationRight(self):
		self.reg = self.dict['rotationRight']
		self.serialWriteReg()
	def rotationLeft(self):
		self.reg = self.dict['rotationLeft']
		self.serialWriteReg()
	def goLeft(self):
		self.reg = self.dict['left']
		self.serialWriteReg()
	def goRight(self):
		self.reg = self.dict['right']
		self.serialWriteReg()
	def diagLeft(self):
		self.reg = self.dict['diagLeft']
		self.serialWriteReg()
	def diagRight(self):
		self.reg = self.dict['diagRight']
		self.serialWriteReg()
	def block(self):
		self.reg = self.dict['block']
		self.serialWriteReg()


