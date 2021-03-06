
import serial
import math
import time
import sys
import numpy as np
from .utils import computeDict,_set_motor,getDistance,stepsFromCm
from enum import Enum,auto

"""
x
^
|
|
0----->y
 """
#region Cr�ation objet

class Robot:
    posArray = [[]]
    dict_speed = {
        'Very slow' : 128,
        'Slow' : 27,
        'Medium' : 16,
        'Fast' : 8,
    }

    def __init__(self):

        self.positionX = 0
        self.positionY = 0
        self.orientationZ = 90
        self.offsetX = 1#2.65
        self.offsetY = 3.8 #la camera est 5.2cm � droite du centre du robot
        self.offsetDistrib = 17.5
        self.deadBandY = 1
        self.deadBandAngle = 8
        self.reg = 0
        self.PORT = "/dev/tty_ARDUINO_USB"
        self.ser = serial.Serial (self.PORT, baudrate = 115200)
        self.DEBUG = True
        self.dict = computeDict()
        self.speed = self.dict_speed['Very slow']
        self.cmdStep = 0x01
        self.cmdNormal = 0x02
        self.stepsForRotation = 5550
        self.stoppedByObstacle = False
        
        self.is_object_in_dir = {'front': False,
                                'right': False,
                                'back': False,
                                'left': False,
        }


    def setSerial(self,port,baudrate = 115200):
        if(self.ser.isOpen()):
            self.ser.close()
        self.ser = serial.Serial(port,baudrate)

    def serialWriteReg(self,cmd,steps = 0):
        if(self.stoppedByObstacle):
            return
        try:
            start = 123 #start byte
            stop = 253 #steaupent byte
            reg = self.reg
            if(cmd == self.cmdNormal):
                divPWM = self.speed
                array = [start,cmd,reg,0,divPWM,stop]
            elif(cmd == self.cmdStep):
                arrayofBytes = steps.to_bytes(2,'big') #split les steps en 2 byte
                MSB = arrayofBytes[0]
                LSB = arrayofBytes[1]
                array = [start,cmd,reg,MSB,LSB,stop]
            message = bytearray(array)
            self.ser.write(message)
        except:
            e = sys.exc_info()[0]
            print(e)
            if(self.ser.isOpen()):
                self.ser.write(0)
                self.ser.close()
                self.ser = serial.Serial (self.PORT, baudrate = 115200)
            else:
                self.ser.open()
#endregion


#region NEW CODE

    def goToSelfCamera(self,targetXYZ,targetAngle): #return 1 si on est arriv�, sinon 0
        targetX = targetXYZ[0]
        targetY = targetXYZ[1]
        targetY = targetY + self.offsetY
        dist = getDistance(targetXYZ)# + targetZ**2)
        print("Target ANGLE =", round(targetAngle, 2))
        angle_normalized = targetAngle % 60
        print("Target X = " + str(round(targetX, 2)) + " Target Y = " + str(round(targetY, 2)))
        print("Angle = " + str(round(targetAngle, 2)))

        if(dist > 25):
            self.approachTarget(targetX,targetY)
            #print("APPROACHING")

        elif(angle_normalized > 30 + self.deadBandAngle or angle_normalized < 30 - self.deadBandAngle): #corrige l'angle
            self.speed = self.dict_speed['Slow']
            self.correctAngle(targetAngle)
            #print("RECTIFYING ANGLE")

        elif(abs(targetY) > (self.deadBandY)): #s'aligne
            #print("ALIGNING")
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
            #print("BRRRR")
            self.goForward()

        else: #arrived
            self.block()
            #self.stopMotors()
            return 1
        return 0


    def correctAngle(self,angle):
        print("Aiming for angle = ", angle)
        angle = angle % 360
        angle = angle % 120
        if(angle >= 60): #sélectionne un des 2 cote
            steps = self.stepsForAngle(angle-60)
            if(angle > 90):
                print("RotateRight")
                self.rotationRight(steps)
            else:
                print("RotateLeft")
                self.rotationLeft(steps)
        else:           #l'autre cote
            steps = self.stepsForAngle(angle-30)
            if(angle > 30):
                print("RotateRight")
                self.rotationRight(steps)
            else:
                print("RotateLeft")
                self.rotationLeft(steps)
        self.waitForSteps(steps)



    def approachTarget(self, targetX, targetY): #s'approche de la position en s'alignant d'abord en Y et puis en avan�ant
        if(abs(targetY) > (self.deadBandY + 0.4*targetX)):
            self.speed = self.dict_speed['Medium']
            if(targetY < 0):
                self.goLeft()
            else:
                self.goRight()
        else:
            self.speed = self.dict_speed['Medium']
            self.goForward()


    def approachTargetUsingRotation(self, targetX, targetY, offset = 8): #s'approche en utilisant le chemin le plus court
        targetX -= self.positionX
        targetY -= self.positionY
        dist = math.sqrt(targetX**2 + targetY**2)
        print("Dist = ", dist)

        quadrants = self._quadrant(targetX,targetY)
        if(quadrants == self.Quandrants.FIRST or quadrants == self.Quandrants.FOURTH ): #x positif
            angle = 90 + math.degrees(math.atan(targetX / targetY ))
        elif(quadrants == self.Quandrants.SECOND or quadrants == self.Quandrants.THIRD): #x negatif
            angle = 270 + math.degrees(math.atan(targetX / targetY ))
        elif(quadrants == self.Quandrants.POS_X ):
            angle = 90
        elif(quadrants == self.Quandrants.NEG_X ):
            angle = 270
        elif(quadrants == self.Quandrants.POS_Y ):
            angle = 180
        elif(quadrants == self.Quandrants.NEG_Y ):
            angle = 0
        while(not self._setOrientation(angle,3)): #bloque tant qu'on a pas fait l'angle
            pass
        steps = stepsFromCm(dist)
        self.goForward(steps)
        self.waitForSteps(steps)
        return 1

    def goToUsingLocation(self,targetX,targetY, targetAngle): #TODO translater quand on est align�* Rotate until I see tag ENT ENTENT
        offset_max_distance = 8
        deltaX = self.positionX - targetX
        deltaY = self.positionY - targetY
        distance = math.sqrt(deltaX**2 + deltaY**2)
        print("Distance to Target = " +str(distance))
        if (deltaX == 0):
            deltaX = 0.01
        if (deltaX < 0):
            angleToTarget =  math.degrees(math.atan(deltaY/deltaX)) +90
        else:
            angleToTarget =  (math.degrees(math.atan(deltaY/deltaX)) +270)
            print("Second cas")
        print(f"deltaX = {deltaX}, deltaY = {deltaY}")
        print("Current Angle = " + str(self.orientationZ))
        print("Angle To Target = " + str(angleToTarget))
        print("Current pos X,Y = " + str(self.positionX) + " , " + str(self.positionY))
        if(distance > offset_max_distance):
            if(self._setOrientation(angleToTarget)): #si �a return 1 c'est que la rotation est OK
                self.speed = self.dict_speed['Medium'] #donc on peut avancer
                print("Forward")
                self.goForward()
            return False
        print("WOW C FINI INCROYABLE")
        return True


    def goToDistributeur(self,targetXYZ):
        offsetX = 2
        targetX = targetXYZ[0]
        targetY = targetXYZ[1]
        targetY = targetY + self.offsetY #TODO
        print("Target X = ",targetX)
        print("Target Y = ",targetY)
        self.speed = self.dict_speed['Slow']
        if(abs(targetY) > self.deadBandY-0.1): #s'aligne
            print("ALIGNING")
            if(targetY < 0):
                self.goLeft()
            else:
                self.goRight()
            return 0
        elif(targetX > self.offsetDistrib + offsetX):
            self.goForward()
            return 0
        elif(targetX < self.offsetDistrib - offsetX):
            self.goBackward()
            return 0
        self.block()
        return 1

    def _setOrientation(self,angleToReach,offset_max_angle = 12):
        '''angleToAchieve = ((angleToReach-self.orientationZ +540)%360)-180  #https://math.stackexchange.com/questions/110080/shortest-way-to-achieve-target-angle/2898118)
        if(abs(angleToAchieve) > 18):
            self.speed = self.dict_speed['Medium'] #si on doit faire un grand angle, on va vite
        else:
            self.speed = self.dict_speed['Slow']
        if(abs(angleToAchieve) > offset_max_angle):
            if(angleToAchieve  > 0):
                self.rotationRight()
                #print("right")
            else:
                self.rotationLeft()
                #print("left")
            return 0
        return 1'''
        angleToAchieve = ((angleToReach-self.orientationZ +540)%360)-180
        print("Angle to achieve : ", angleToAchieve)
        stepsToDo = self.stepsForAngle(angleToAchieve)
        usingSteps = False
        self.speed = self.dict_speed['Slow']
        if(abs(angleToAchieve) > 25):
            usingSteps = True
        if(abs(angleToAchieve) > offset_max_angle):
            if(angleToAchieve  > 0):
                if(usingSteps):
                    self.rotationRight(stepsToDo)
                else:
                    self.rotationRight()
            else:
                if(usingSteps):
                    self.rotationLeft(stepsToDo)
                else:
                    self.rotationLeft()
            if(usingSteps):
                print("Rotating using steps")
                self.waitForSteps(stepsToDo)
            return 0
        return 1


    def setOrientation(self,angleToReach,offset_max_angle = 15):
        if(self._setOrientation(angleToReach,offset_max_angle)):
            print("ENT LENTGLE EST BONENT")
            self.stopMotors()
            return 1
        else:
            return 0

    def goToNewVersion(self,targetX,targetY,offset = 5):
        deltaX = targetX -  self.positionX
        deltaY = targetY -  self.positionY
        arrived = 0
        self.debugPositions(targetX, targetY)

        if(self.setOrientationForTranslation()): #on est align� c'est bon
            if(deltaY > offset):
                self.translateOnYAxis(1)
            elif(deltaY < -offset):
                self.translateOnYAxis(0)
            elif(deltaX > offset):
                self.translateOnXAxis(1)
            elif(deltaX < -offset):
                self.translateOnXAxis(0)
            else:
                arrived = 1
                self.stopMotors()
        return arrived



    def translateOnXAxis(self,dir): #if dir = 1 X augmente, sinon X diminue
        self.speed = self.dict_speed['Medium']
        cadran = round(self.orientationZ / 90)
        angle = cadran*90
        if (angle == 90 ):
            if(dir):
                self.goForward()
            else:
                self.goBackward()
        elif (angle == 180 ):
            if(dir):
                self.goLeft()
            else:
                self.goRight()
        elif (angle == 270 ):
            if(dir):
                self.goBackward()
            else:
                self.goForward()
        elif (angle == 0 or angle == 360 ):
            if(dir):
                self.goRight()
            else:
                self.goLeft()

    def translateOnYAxis(self,dir):
        self.speed = self.dict_speed['Medium']
        cadran = round(self.orientationZ / 90)
        angle = cadran*90
        if(angle == 90):
            if(dir):
                self.goRight()
            else:
                self.goLeft()
        if(angle == 180):
            if(dir):
                self.goForward()
            else:
                self.goBackward()
        if(angle == 270):
            if(dir):
                self.goLeft()
            else:
                self.goRight()
        if(angle == 0 or angle == 360):
            if(dir):
                self.goBackward()
            else:
                self.goForward()

    def setOrientationForTranslation(self):
        offsetAngle = 7
        cadran = self.orientationZ // 90
        angleToDir = self.orientationZ % 90
        if(angleToDir < 45):
            if(self._setOrientation(cadran * 90,offsetAngle)):
                return 1
        else:
            if(self._setOrientation((cadran+1) * 90,offsetAngle)):
                return 1
        return 0

    def monitorSteps(self,targetX,targetY): #TODO
        currentDeltaX = abs(self.positionX - targetX)
        currentDeltaY = abs(self.positionY - targetY)
        """ startDeltaX = startX - targetX
        startDeltaY = startY - targetY
        start_dist = getDistance([startDeltaX,startDeltaY])
        print("Original distance = ",start_dist)
        current_dist = getDistance([currentDeltaX,currentDeltaY])
        print("Current distance = ",current_dist)
        if(current_dist > start_dist+15): #La distance grandit, y a une couille
            print("Something is wrong in the direction")
            return 1 """
        if(currentDeltaX < 5 and currentDeltaY < 5): #on est arrivé
            print("Arrived")
            return 1
        else:
            return 0

    def waitForSteps(self,steps):
        time.sleep(steps/1300)

    def stepsForAngle(self,angle):
        ratio = angle / 360
        steps = self.stepsForRotation * ratio
        return int(4*abs(steps))

    class Quandrants(Enum):
        FIRST = auto()
        SECOND = auto()
        THIRD = auto()
        FOURTH = auto()
        POS_X = auto()
        NEG_X = auto()
        POS_Y = auto()
        NEG_Y = auto()
        ERROR = auto()



    # Function to check quadrant
    def _quadrant(self,x,y):
        temp = 0
        if (x > 0 and y > 0):
            temp = self.Quandrants.FIRST

        elif (x < 0 and y > 0):
            temp = self.Quandrants.SECOND

        elif (x < 0 and y < 0):
            temp = self.Quandrants.THIRD

        elif (x > 0 and y < 0):
            temp = self.Quandrants.FOURTH

        elif (x == 0 and y > 0):
            temp = self.Quandrants.POS_Y

        elif (x == 0 and y < 0):
            temp = self.Quandrants.NEG_Y

        elif (y == 0 and x < 0):
            temp = self.Quandrants.NEG_X

        elif (y == 0 and x > 0):
            temp = self.Quandrants.POS_X
        else:
            temp = self.Quandrants.ERROR
        return temp


#endregion

#region DEBUG

    def debugPositions(self, targetX, targetY):
        print("My PositionX = " + str(self.positionX) + " My PositionY = " + str(self.positionY))
        print("TargetX = " + str(targetX) + " TargetY = " + str(targetY))


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
                self.serialWriteReg(self.cmdNormal)
                time.sleep(1)
                self.reg = _set_motor(motor,1,1)
                self.serialWriteReg(self.cmdNormal)
                time.sleep(1)

    def debugDirections(self):
        self.speed = self.dict_speed['Medium']
        while(True):
            for key in self.dict:
                self.reg = self.dict[key]
                print(key, '->', self.dict[key])
                self.serialWriteReg(self.cmdNormal)
                time.sleep(3)

    def debugDirections(self):
            self.speed = self.dict_speed['Medium']
            while(True):
                for key in self.dict:
                    self.reg = self.dict[key]
                    print(key, '->', self.dict[key])
                    self.serialWriteReg(self.cmdNormal)
                    time.sleep(3)

    def goForward(self,steps = 0):
        self.reg = self.dict['forward']
        if not self.is_object_in_dir['front']:
            if(steps != 0):
                self.serialWriteReg(self.cmdStep,steps)
            else:
                self.serialWriteReg(self.cmdNormal)

    def goBackward(self,steps = 0):
        self.reg = self.dict['backward']
        if not self.is_object_in_dir['back']:
            if(steps != 0):
                self.serialWriteReg(self.cmdStep,steps)
            else:
                self.serialWriteReg(self.cmdNormal)

    def stopMotors(self,steps = 0):
        self.reg = self.dict['stop']
        if(steps != 0):
            self.serialWriteReg(self.cmdStep,steps)
        else:
            self.serialWriteReg(self.cmdNormal)

    def rotationRight(self,steps = 0):
        self.reg = self.dict['rotationRight']
        if(steps != 0):
            self.serialWriteReg(self.cmdStep,steps)
        else:
            self.serialWriteReg(self.cmdNormal)

    def rotationLeft(self,steps = 0):
        self.reg = self.dict['rotationLeft']
        if(steps != 0):
            self.serialWriteReg(self.cmdStep,steps)
        else:
            self.serialWriteReg(self.cmdNormal)

    def goLeft(self,steps = 0):
        self.reg = self.dict['left']
        if not self.is_object_in_dir['left']:
            if(steps != 0):
                self.serialWriteReg(self.cmdStep,steps)
            else:
                self.serialWriteReg(self.cmdNormal)

    def goRight(self,steps = 0):
        self.reg = self.dict['right']
        if not self.is_object_in_dir['right']:
            if(steps != 0):
                self.serialWriteReg(self.cmdStep,steps)
            else:
                self.serialWriteReg(self.cmdNormal)

    def block(self,steps = 0):
        self.reg = self.dict['block']
        if(steps != 0):
            self.serialWriteReg(self.cmdStep,steps)
        else:
            self.serialWriteReg(self.cmdNormal)

    def getCurrentDirection(self):
        return(list(self.dict.keys())[list(self.dict.values()).index(self.reg)])

