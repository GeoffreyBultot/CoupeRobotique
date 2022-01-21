from ArmRX24F.arm import *
from Ventouse.ventouse import *
from ServoStock.servoStock import *
import asyncio

ventouse = Ventouse(2, 3, GPIO.BCM)
arm = Arm()

slotPositionDrop = [[47, 88, 55],
                    [62, 80, 50],
                    [80, 85, 30],
                    [103, 65, 25]]

slotPositionGrab = [[65, 50, 70],
                    [80, 55, 50],
                    [100, 40, 35],
                    [120, 20, 40]]


def grabElementGround():
    arm.isInside = False
    arm.MAX_OVERALL_SPEED = 25
    print("---------------\nSetting before sunct")
    arm.setServosOurAngle([13, 30, 86])
    time.sleep(0.5)

    arm.setServosOurAngle([20,20,50])
    time.sleep(0.5)

    arm.setServosOurAngle([49, 30, 9])
    time.sleep(0.2)

    arm.setServosOurAngle([57, 15, 17])
    print("Sucking")
    ventouse.sunct()


def setupAfterGrab():
    arm.isInside = False
    arm.MAX_OVERALL_SPEED = 20
    print("---------------\n Back A BIT")
    arm.setServosOurAngle([6, 71, 23])
    time.sleep(0.5)

    print("---------------\nSetting Higher")
    arm.setServosOurAngle([20, 60, 45])
    time.sleep(0.5)

    print("---------------\nSetting Inside")
    arm.setServosOurAngle([32, 67, 80])
    time.sleep(0.4)


def setArmPosDistrib(uid):
    if arm.isInside:
        setOutsideFromInside()
    arm.MAX_OVERALL_SPEED = 30

    print("---------------\nSetting before sunct Distrib")
    arm.setServosOurAngle([40, 0, 0])
    time.sleep(0.5)
    if uid == 0:
        arm.setServosOurAngle([16, 62, -52])
        time.sleep(0.7)
    elif uid == 1:
        arm.setServosOurAngle([18, 62, -52])
        time.sleep(0.7)
    elif uid == 2:
        arm.setServosOurAngle([20, 57, -45])
        time.sleep(0.7)


def suckAndSetArmUpDistrib():
    arm.isInside = False
    ventouse.sunct()

    arm.MAX_OVERALL_SPEED = 20

    #Going Up
    arm.setServosOurAngle([0, 60, -90])


def setSlotId(slot):
    arm.isInside = False
    arm.MAX_OVERALL_SPEED = 20
    if slot>=0:
        print("---------------\nSetting 1st")
        arm.setServosOurAngle(slotPositionDrop[0]) 
        time.sleep(0.5)
    
    if slot>=1:
        print("---------------\nSetting 2d")
        arm.setServosOurAngle(slotPositionDrop[1])  
        time.sleep(0.4)

    if slot>=2:
        print("---------------\nSetting 3rd")
        arm.setServosOurAngle(slotPositionDrop[2]) 
        time.sleep(0.4)

    if slot>=3:
        print("---------------\nSetting 4th")
        arm.setServosOurAngle(slotPositionDrop[3])
        time.sleep(0.4)

    time.sleep(0.5)
    ventouse.drop()
    arm.MAX_OVERALL_SPEED = 30

    if slot>=3:
        print("---------------\nSetting 3rd")
        arm.setServosOurAngle(slotPositionDrop[2]) #98.07, 
        time.sleep(0.5)

    if slot>0:
        print("---------------\nSetting 1st")
        arm.setServosOurAngle(slotPositionDrop[0]) #98.07, 
        time.sleep(0.5)

    print("---------------\nSetting Preparation")
    arm.setServosOurAngle([40, 90, 30]) #98.07, 
    time.sleep(0.5)


def grabElementSlot(slot):
    arm.isInside = False
    arm.MAX_OVERALL_SPEED = 20
    print("---------------\nSetting Inside")
    arm.setServosOurAngle([32, 67, 80])
    time.sleep(0.4)
    
    if slot>=0:
        arm.MAX_OVERALL_SPEED = 15
        print("---------------\nGrabbing 1st")
        arm.setServosOurAngle(slotPositionGrab[0])
        time.sleep(0.5)
    
    if slot>=1:
        print("---------------\nGrabbing 2d")
        arm.setServosOurAngle(slotPositionGrab[1])
        time.sleep(0.5)

    if slot>=2:
        print("---------------\nGrabbing 3rd")
        arm.setServosOurAngle(slotPositionGrab[2])
        time.sleep(0.5)

    if slot>=3:
        print("---------------\nGrabbing 4th")
        arm.setServosOurAngle(slotPositionGrab[3])
        time.sleep(0.5)

    print("Sucking")
    ventouse.sunct()

    print("Going Up")

    arm.MAX_OVERALL_SPEED = 25

    #ventouse.drop()
    time.sleep(0.5)

    if slot == 0:
        arm.setServosOurAngle([52, 88, 63])
    elif slot == 1:
        arm.setServosOurAngle(slotPositionDrop[1])
    elif slot == 2:
        arm.setServosOurAngle(slotPositionDrop[2])
    elif slot == 3:
        arm.setServosOurAngle(slotPositionDrop[3])
    
    time.sleep(0.5)

    if slot>=3:
        print("---------------\nSetting 3rd")
        arm.setServosOurAngle(slotPositionDrop[2])
        time.sleep(0.5)

    if slot>0:
        print("---------------\nSetting 1st")
        arm.setServosOurAngle(slotPositionDrop[0])
        time.sleep(0.5)

    print("---------------\nSetting Preparation")
    arm.setServosOurAngle([20, 90, 70])
    time.sleep(0.5)


def setArmBotGallery():
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = False
    arm.setServosOurAngle([-9, 50.3, -25])
    time.sleep(1)


'''def dropGallery():
    ventouse.drop()
    print("---------------\nSetting Inside")'''


def setArmTopGallery():
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = False
    arm.setServosOurAngle([-30, 0, 29])
    time.sleep(1)


def throw():
    arm.setServosOurAngle([20,20,20])
    time.sleep(1)
    
    arm.MAX_OVERALL_SPEED = 30
    arm.setServosOurAngle([-25,0,0])
    time.sleep(0.15)

    ventouse.drop()
    time.sleep(0.1)

    arm.MAX_OVERALL_SPEED = 20
    arm.setServosOurAngle([20,20,20])
    time.sleep(0.2)


def flex():
    arm.MAX_OVERALL_SPEED = 100
    arm.setServosOurAngle([0,0,0])
    time.sleep(0.1)
    arm.setServosOurAngle([0,-45,0])
    time.sleep(0.1)
    arm.setServosOurAngle([0,-45,-45])
    time.sleep(0.1)
    arm.setServosOurAngle([0,-0,-45])
    time.sleep(0.1)


def hideOutside():
    arm.MAX_OVERALL_SPEED = 50
    arm.setServosOurAngle([45,0,0])
    time.sleep(0.2)
    arm.setServosOurAngle([0,45,45])
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = False


def hideInside():
    arm.MAX_OVERALL_SPEED = 50
    arm.setServosOurAngle([45,0,0])
    time.sleep(0.2)
    arm.setServosOurAngle([45,90,90])
    time.sleep(0.2)
    arm.setServosOurAngle([90,92,92])
    time.sleep(0.4)
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = True


def setOutsideFromInside():
    arm.MAX_OVERALL_SPEED = 50
    arm.setServosOurAngle([20,90,90])
    time.sleep(0.4)
    arm.setServosOurAngle([20,90,60])
    time.sleep(0.2)
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = False


'''def main():
    try:
        ventouse.setDefault()

        arm.enableTorqueAll()
        arm.setMaxTorqueAll(100)
        arm.setTorqueLimitAll(100)
        arm.MAX_OVERALL_SPEED = 20

        servo = ServoStock(13, 400, GPIO.BCM)

        arm.setServosOurAngle([90,92,92])
        servo.setDefault()
        time.sleep(0.5)
        servo.stopPwm()

        for i in range(3,-1, -1):
            grabElementGround()
            setupAfterGrab()
            setSlotId(i)

        hideInside()

        servo.setReverse()
        time.sleep(0.5)
        servo.stopPwm()

        for i in range(0, 4):
            grabElementSlot(i)
            setArmTopGallery()
            ventouse.drop()

        servo.setDefault()

            
    except KeyboardInterrupt:
        #GPIO.cleanup()
        x = input("Disable torque ?")
        if x == "O":
            arm.disableTorqueAll()

main()'''