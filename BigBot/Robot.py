from enum import IntEnum
from motorLogic import goForward
import serial
import math

"""TODO 
Serveur MQTT
Logique déplacement

!!!
Plutot que de calculer chaque fois les registre
foutre tout dans un dictionnaire et copier le binaire
Je suis extrêmement con

"""

class Robot:
	positionX, positionY = 0
	orientationZ = 0
	FORWARD = 1
	BACKWARD = 0
	ON = 1
	OFF = 0
	reg = 0
	DEBUG = 1

	
	


	class Edir(IntEnum):
		DIR_FL = 0
		DIR_FR = 2
		DIR_RL = 4
		DIR_RR = 6
	

	class Estate(IntEnum):
		STATE_FL  = 1
		STATE_FR  = 3
		STATE_RL  = 5
		STATE_RR  = 7


	class Emotor(IntEnum):
		FL = 0
		FR = 1
		RL = 2
		RR = 3
		n = 4

	def __init__(self):
		self.positionX = 0
		self.positionY = 0
		self.orientationZ = 0
		self.reg = 0
		self.PORT = "/dev/ttyUSB0"
		self.ser = serial.Serial (self.PORT, 
        baudrate = 115200)
		self.DEBUG = 1


	def setSerial(self,port,baudrate = 115200):
		if(self.ser.isOpen()):
			self.ser.close()
		self.ser = serial.Serial(port,baudrate) 




	def serialWriteReg(self):
		try: 
			reg = self.reg.to_bytes(1,'big')
			self.ser.write(reg)
		except:
			print("ERROR USB")
			if(self.ser.isOpen()):
				self.ser.write(0)
				self.ser.close()
			else:
				self.ser.open() 

	def goToSelfCamera(self,targetX,targetY,offset_max_x = 15,offset_max_angle = 3):
		anglexy = math.degrees(math.atan(targetX / targetY )) #calcule l'angle
		if(anglexy < 0):
			anglexy = -(90+anglexy)
		elif(anglexy > 0):
			anglexy = 90-anglexy
	
		if(anglexy > offset_max_angle or anglexy < -offset_max_angle  ):
			if(anglexy > 0):
				self.rotationRight()
			else:
				self.rotationLeft()
		elif(targetX > offset_max_x):
			self.goForward()
		else:
			self.stopMotors()
		self.serialWriteReg()
		return

	def goToUsingLocation(self,targetX,targetY, offset_max_distance = 20,offset_max_angle = 3):
		deltaX = self.positionX - targetX
		deltaY = self.positionY - targetY
		distance = deltaX**2 + deltaY**2
		angleToTarget =  math.atan(deltaX/deltaY)
		if(abs(angleToTarget) > offset_max_angle): #orientation vers target
			if(self.orientationZ > angleToTarget):
				self.rotationRight()
			else:
				self.rotationLeft()
		elif(distance > offset_max_distance):
			self.goForward()
		return distance





	



	def _computeDict(self):
		stop = 0
		rotright = self.rotationRight()
		rotleft = self.rotationLeft()
		goforward = self.goForward()
		goback = self.goBackwards()
		left = self.goLeft()
		right = self.goRight()
		diagleft = self.diagLeft()
		diagright = self.diagRight()
		self.dict = {'stop' : stop, 'rotationRight' : rotright, 'rotationLeft' : rotleft,
		'goFoward' : goforward,'goBackwards' : goback, 'goLeft' : left, 'goRight' : right,
		'diagLeft' : diagleft, 'diagRight' : diagright}
	



	def stopMotors(self):
		self.reg = 0
	
	def rotationRight(self):
		val = 0
		val = self.set_value(val,self.ON,self.Estate.STATE_FL)
		val = self.set_value(val,self.ON,self.Estate.STATE_FR)
		val = self.set_value(val,self.ON,self.Estate.STATE_RL)
		val = self.set_value(val,self.ON,self.Estate.STATE_RR)

		val = self.set_value(val,self.FORWARD,self.Edir.DIR_FL)
		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_FR)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_RL)
		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_RR)
		self.reg = val
		if(self.DEBUG):
			print("Rotation right ")

	def rotationLeft(self):
		val = 0
		val = self.set_value(val,self.ON,self.Estate.STATE_FL)
		val = self.set_value(val,self.ON,self.Estate.STATE_FR)
		val = self.set_value(val,self.ON,self.Estate.STATE_RL)
		val = self.set_value(val,self.ON,self.Estate.STATE_RR)

		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_FL)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_FR)
		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_RL)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_RR)
		self.reg = val
		if(self.DEBUG):
			print("Rotation left ")
	
	def goForward(self):
		val = 0
		val = self.set_value(val,self.ON,self.Estate.STATE_FL)
		val = self.set_value(val,self.ON,self.Estate.STATE_FR)
		val = self.set_value(val,self.ON,self.Estate.STATE_RL)
		val = self.set_value(val,self.ON,self.Estate.STATE_RR)

		val = self.set_value(val,self.FORWARD,self.Edir.DIR_FL)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_FR)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_RL)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_RR)

		self.reg = val
		if(self.DEBUG):
			print(" Forward ")

	def goBackwards(self):
		val = 0
		val = self.set_value(val,self.ON,self.Estate.STATE_FL)
		val = self.set_value(val,self.ON,self.Estate.STATE_FR)
		val = self.set_value(val,self.ON,self.Estate.STATE_RL)
		val = self.set_value(val,self.ON,self.Estate.STATE_RR)

		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_FL)
		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_FR)
		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_RL)
		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_RR)

		self.reg = val
		if(self.DEBUG):
			print(" Backward ")


	def goLeft(self):
		val = 0
		val = self.set_value(val,self.ON,self.Estate.STATE_FL)
		val = self.set_value(val,self.ON,self.Estate.STATE_FR)
		val = self.set_value(val,self.ON,self.Estate.STATE_RL)
		val = self.set_value(val,self.ON,self.Estate.STATE_RR)

		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_FL)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_FR)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_RL)
		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_RR)
		self.reg = val
		if(self.DEBUG):
			print(" left ")

	def goRight(self):
		val = 0
		val = self.set_value(val,self.ON,self.Estate.STATE_FL)
		val = self.set_value(val,self.ON,self.Estate.STATE_FR)
		val = self.set_value(val,self.ON,self.Estate.STATE_RL)
		val = self.set_value(val,self.ON,self.Estate.STATE_RR)

		val = self.set_value(val,self.FORWARD,self.Edir.DIR_FL)
		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_FR)
		val = self.set_value(val,self.BACKWARD,self.Edir.DIR_RL)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_RR)
		self.reg = val
		if(self.DEBUG):
			print(" right ")

	def diagLeft(self):
		val = 0
		val = self.set_value(val,self.OFF,self.Estate.STATE_FL)
		val = self.set_value(val,self.ON,self.Estate.STATE_FR)
		val = self.set_value(val,self.ON,self.Estate.STATE_RL)
		val = self.set_value(val,self.OFF,self.Estate.STATE_RR)

		val = self.set_value(val,self.FORWARD,self.Edir.DIR_FR)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_RL)
		self.reg = val
		if(self.DEBUG):
			print(" diagLeft ")

	def diagRight(self):
		val = 0
		val = self.set_value(val,self.ON,self.Estate.STATE_FL)
		val = self.set_value(val,self.OFF,self.Estate.STATE_FR)
		val = self.set_value(val,self.OFF,self.Estate.STATE_RL)
		val = self.set_value(val,self.ON,self.Estate.STATE_RR)
		
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_FL)
		val = self.set_value(val,self.FORWARD,self.Edir.DIR_RR)
		self.reg = val
		if(self.DEBUG):
			print(" diagRight ")

	def set_motor(self,n,dir,state): 
		val = 0
		if(dir):
			val = self.set_bit(val,2*n)
		else:
			val = self.clear_bit(val,2*n)
		if(state):
			val = self.set_bit(val,2*n + 1)
		else:
			val = self.clear_bit(val,2*n + 1)
		self.reg = val

	def set_bit(value, bit):
		return value | (1<<bit)

	def clear_bit(value, bit):
		return  value & ~(1<<bit)

	def set_value(self,temp,value,bit):
		if(value):
			temp = self.set_bit(temp,bit)
		else:
			temp = self.clear_bit(temp,bit)
		return temp

