import serial
import math
import time
import sys
from utils import computeDict,_set_motor

class Robot:
	positionX = 0
	positionY = 0
	orientationZ = 0
	reg = 0
	DEBUG = 1
	offsetCenter = 6

	

	def __init__(self):
		self.positionX = 0
		self.positionY = 0
		self.orientationZ = 0
		self.reg = 0
		#self.PORT = "/dev/ttyUSB0"
		self.PORT = "/dev/tty_ARDUINO_USB"#"/dev/ttyACM0"
		self.ser = serial.Serial (self.PORT, 
        baudrate = 115200)
		self.DEBUG = 1
		self.dict = computeDict()


	def setSerial(self,port,baudrate = 115200):
		if(self.ser.isOpen()):
			self.ser.close()
		self.ser = serial.Serial(port,baudrate) 

	def serialWriteReg(self):
		try: 
			reg = self.reg.to_bytes(1,'big')
			self.ser.write(reg)
		except:
			e = sys.exc_info()[0]
			print(e)
			if(self.ser.isOpen()):
				self.ser.write(0)
				self.ser.close()
			else:
				self.ser.open() 

	def goToSelfCamera(self,targetX,targetY,targetAngle,offset_max_x = 8,offset_max_y = 3):
		""" anglexy = math.degrees(math.atan(targetX / targetY )) #calcule l'angle
		if(anglexy < 0):
			anglexy = -(90+anglexy)
		elif(anglexy > 0):
			anglexy = 90-anglexy
		print("Angle XY = " + str(anglexy))
	
		if(anglexy > offset_max_angle or anglexy < -offset_max_angle  ):
			if(anglexy > 0):
				self.rotationRight()
			else:
				self.rotationLeft() """

		targetY = targetY + self.offsetCenter
		angle = targetAngle%60
		if(angle > 38 or angle <22):
			if(angle >38):
				self.rotationRight()
			if(angle <22):
				self.rotationLeft()
		
		elif(abs(targetY) > offset_max_y):
			if(targetY < 0):
				self.goLeft()
			else:
				self.goRight()
		elif(targetX > offset_max_x):
			self.goForward()
		else:
			self.stopMotors()
		if(self.DEBUG):
			print("Target X = " + str(targetX) + " Y = " + str(targetY) )
		dist = math.sqrt(targetX**2 + targetY**2)
		return dist

	def goToUsingLocation(self,targetX,targetY, offset_max_distance = 20,offset_max_angle = 10):
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


