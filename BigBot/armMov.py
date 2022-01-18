from ArmRX24F.arm import *
from Ventouse.ventouse import *
from ServoStock.servoStock import *
import asyncio

ventouse = Ventouse(2, 3, GPIO.BCM)
arm = Arm()

slotPositionDrop = [[52, 88, 55],
                    [67, 85, 45],
                    [80, 85, 30],
                    [100, 70, 25]]

slotPositionGrab = [[65, 56, 70],
                    [80, 65, 45],
                    [100, 45, 47],
                    [120, 20, 40]]


async def grabElementGround():
    arm.isInside = False
    arm.MAX_OVERALL_SPEED = 25
    print("---------------\nSetting before sunct")
    arm.setServosOurAngle([13, 30, 86])
    await asyncio.sleep(0.5)

    arm.setServosOurAngle([20,20,50])
    await asyncio.sleep(0.5)

    arm.setServosOurAngle([49, 30, 9])
    await asyncio.sleep(0.2)

    arm.setServosOurAngle([57, 15, 17])
    print("Sucking")
    await ventouse.sunct()


async def setupAfterGrab():
    arm.isInside = False
    arm.MAX_OVERALL_SPEED = 20
    print("---------------\n Back A BIT")
    arm.setServosOurAngle([6, 71, 23])
    await asyncio.sleep(0.5)

    print("---------------\nSetting Higher")
    arm.setServosOurAngle([20, 60, 45])
    await asyncio.sleep(0.5)

    print("---------------\nSetting Inside")
    arm.setServosOurAngle([32, 67, 80])
    await asyncio.sleep(0.4)


async def setArmPosDistrib(uid):
    if arm.isInside:
        await setOutsideFromInside()
    arm.MAX_OVERALL_SPEED = 30

    print("---------------\nSetting before sunct Distrib")
    arm.setServosOurAngle([40, 0, 0])
    await asyncio.sleep(0.5)
    if uid == 0:
        arm.setServosOurAngle([16, 62, -52])
        await asyncio.sleep(0.7)
    elif uid == 1:
        arm.setServosOurAngle([18, 62, -52])
        await asyncio.sleep(0.7)
    elif uid == 2:
        arm.setServosOurAngle([20, 57, -45])
        await asyncio.sleep(0.7)


async def suckAndSetArmUpDistrib():
    arm.isInside = False
    await ventouse.sunct()

    arm.MAX_OVERALL_SPEED = 20

    #Going Up
    arm.setServosOurAngle([0, 60, -90])


async def setSlotId(slot):
    arm.isInside = False
    arm.MAX_OVERALL_SPEED = 10
    if slot>=0:
        print("---------------\nSetting 1st")
        arm.setServosOurAngle(slotPositionDrop[0]) 
        await asyncio.sleep(0.5)
    
    if slot>=1:
        print("---------------\nSetting 2d")
        arm.setServosOurAngle(slotPositionDrop[1])  
        await asyncio.sleep(0.4)

    if slot>=2:
        print("---------------\nSetting 3rd")
        arm.setServosOurAngle(slotPositionDrop[2]) 
        await asyncio.sleep(0.4)

    if slot>=3:
        print("---------------\nSetting 4th")
        arm.setServosOurAngle(slotPositionDrop[3])
        await asyncio.sleep(0.4)

    await asyncio.sleep(0.5)
    await ventouse.drop()
    arm.MAX_OVERALL_SPEED = 30

    if slot>=3:
        print("---------------\nSetting 3rd")
        arm.setServosOurAngle(slotPositionDrop[2]) #98.07, 
        await asyncio.sleep(0.5)

    if slot>0:
        print("---------------\nSetting 1st")
        arm.setServosOurAngle(slotPositionDrop[0]) #98.07, 
        await asyncio.sleep(0.5)

    print("---------------\nSetting Preparation")
    arm.setServosOurAngle([40, 90, 30]) #98.07, 
    await asyncio.sleep(0.5)

    arm.MAX_OVERALL_SPEED = 50
    print("---------------\nSet inside")
    arm.setServosOurAngle([90, 92, 92]) #98.07, 
    await asyncio.sleep(0.5)
    arm.isInside = True


async def grabElementSlot(slot):
    arm.isInside = False
    arm.MAX_OVERALL_SPEED = 20
    print("---------------\nSetting Inside")
    arm.setServosOurAngle([32, 67, 80])
    await asyncio.sleep(0.4)
    
    if slot>=0:
        arm.MAX_OVERALL_SPEED = 15
        print("---------------\nGrabbing 1st")
        arm.setServosOurAngle(slotPositionGrab[0])
        await asyncio.sleep(0.5)
    
    if slot>=1:
        print("---------------\nGrabbing 2d")
        arm.setServosOurAngle(slotPositionGrab[1])
        await asyncio.sleep(0.5)

    if slot>=2:
        print("---------------\nGrabbing 3rd")
        arm.setServosOurAngle(slotPositionGrab[2])
        await asyncio.sleep(0.5)

    if slot>=3:
        print("---------------\nGrabbing 4th")
        arm.setServosOurAngle(slotPositionGrab[3])
        await asyncio.sleep(0.5)

    print("Sucking")
    await ventouse.sunct()

    print("Going Up")

    arm.MAX_OVERALL_SPEED = 40

    #await ventouse.drop()
    await asyncio.sleep(0.5)

    if slot == 0:
        arm.setServosOurAngle([52, 88, 63])
    elif slot == 1:
        arm.setServosOurAngle(slotPositionDrop[1])
    elif slot == 2:
        arm.setServosOurAngle(slotPositionDrop[2])
    elif slot == 3:
        arm.setServosOurAngle(slotPositionDrop[3])
    
    await asyncio.sleep(0.5)

    if slot>=3:
        print("---------------\nSetting 3rd")
        arm.setServosOurAngle(slotPositionDrop[2])
        await asyncio.sleep(0.5)

    if slot>0:
        print("---------------\nSetting 1st")
        arm.setServosOurAngle(slotPositionDrop[0])
        await asyncio.sleep(0.5)

    print("---------------\nSetting Preparation")
    arm.setServosOurAngle([20, 90, 70])
    await asyncio.sleep(0.5)


async def setArmBotGallery():
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = False
    arm.setServosOurAngle([-9, 50.3, -25])
    await asyncio.sleep(1)


async def dropGallery():
    await ventouse.drop()
    print("---------------\nSetting Inside")
    #arm.setServosOurAngle([32, 67, 80])
    await hideInside()


async def setArmTopGallery():
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = False
    arm.setServosOurAngle([-30, 0, 29])
    await asyncio.sleep(1)


async def throw():
    arm.setServosOurAngle([20,20,20])
    await asyncio.sleep(1)
    
    arm.MAX_OVERALL_SPEED = 30
    arm.setServosOurAngle([-25,0,0])
    await asyncio.sleep(0.15)

    await ventouse.drop()
    await asyncio.sleep(0.1)

    arm.MAX_OVERALL_SPEED = 20
    arm.setServosOurAngle([20,20,20])
    await asyncio.sleep(0.2)


async def flex():
    arm.MAX_OVERALL_SPEED = 40
    arm.setServosOurAngle([0,0,0])
    await asyncio.sleep(0.1)
    arm.setServosOurAngle([0,-45,0])
    await asyncio.sleep(0.1)
    arm.setServosOurAngle([0,-45,-45])
    await asyncio.sleep(0.1)
    arm.setServosOurAngle([0,-0,-45])
    await asyncio.sleep(0.1)


async def hideOutside():
    arm.MAX_OVERALL_SPEED = 50
    arm.setServosOurAngle([45,0,0])
    await asyncio.sleep(0.2)
    arm.setServosOurAngle([0,45,45])
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = False


async def hideInside():
    arm.MAX_OVERALL_SPEED = 50
    arm.setServosOurAngle([45,0,0])
    await asyncio.sleep(0.2)
    arm.setServosOurAngle([45,90,90])
    await asyncio.sleep(0.2)
    arm.setServosOurAngle([90,92,92])
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = True


async def setOutsideFromInside():
    arm.MAX_OVERALL_SPEED = 50
    arm.setServosOurAngle([20,90,90])
    await asyncio.sleep(0.4)
    arm.setServosOurAngle([20,90,60])
    await asyncio.sleep(0.2)
    arm.MAX_OVERALL_SPEED = 20
    arm.isInside = False


async def main():
    try:
        arm.setServosOurAngle([0,0,0])
        '''ventouse.setDefault()

        arm.enableTorqueAll()
        arm.setMaxTorqueAll(100)
        arm.setTorqueLimitAll(100)
        arm.MAX_OVERALL_SPEED = 20

        servo = ServoStock(13, 400, GPIO.BCM)

        servo.setDefault()
        servo.stopPwm()

        await asyncio.sleep(0.5)

        await grabElementSlot(0)

        #await setArmBotGallery()
        await setArmTopGallery()

        await asyncio.sleep(5)

        await dropGallery()

        await setArmPosDistrib(0)
        #Bot should drive forward util connection with el
        await asyncio.sleep(5)

        await suckAndSetArmUpDistrib()
        #Bot should drive back a bit
        await asyncio.sleep(5)

        await setupAfterGrab()

        await setSlotId(0)'''

            
    except KeyboardInterrupt:
        #GPIO.cleanup()
        x = input("Disable torque ?")
        if x == "O":
            arm.disableTorqueAll()

asyncio.run(main())