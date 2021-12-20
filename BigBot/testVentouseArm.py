from ArmRX24F.arm import *
from Ventouse.ventouse import *
import asyncio

ventouse = Ventouse(2, 3, GPIO.BCM)
armTest = Arm()

async def grabElement():
    print("---------------\nSetting before sunct")
    armTest.setServoToXYWithAngle(4.4, 20.5, 90)
    print("---------------\nOn the ELEMENT")
    armTest.setServoToXYWithAngle(4.4, 20.75, 90)
    print("Sucking")
    await ventouse.sunct()


def fromElToFrontBot():
    print("---------------\nSetting Higher")
    armTest.setServoToXYWithAngle(4.4, 11.5, 119.1)
    time.sleep(0.5)

    print("---------------\nSetting Inside")
    armTest.setServoToXYWithAngle(-2.5, 7.8, 185)
    time.sleep(1)

def fromFrontToEl():
    print("---------------\nSetting Inside")
    armTest.setServoToXYWithAngle(-2.5, 7.8, 185)
    time.sleep(0.5)

    print("---------------\nSetting Higher")
    armTest.setServoToXYWithAngle(4.4, 11.5, 119.1)


async def settingOnSlot(uid):
    armTest.MAX_OVERALL_SPEED = 20
    base_x = -16.1
    newX = base_x + (2.85*uid)
    armTest.setServoToXYWithAngle(newX, 7.8, 185)
    time.sleep(1)
    await ventouse.drop()
    armTest.MAX_OVERALL_SPEED = 30


async def main():
    try:
        ventouse.setDefault()

        armTest.enableTorqueAll()
        #armTest.setMaxTorqueAll(100)
        #armTest.setTorqueLimitAll(100)
        armTest.MAX_OVERALL_SPEED = 20

        print("---------------\nDeault 0 pos")
        while 1:
            armTest.setServosOurAngle([0,0,0])
            time.sleep(1)

            armTest.setServosOurAngle([90,0,0])
            time.sleep(1)

        '''await grabElement()
        fromElToFrontBot()

        print("---------------\nSetting 4th Slot")
        await settingOnSlot(4)

        fromFrontToEl()

        await grabElement()
        fromElToFrontBot()

        print("---------------\nSetting 3rd Slot")
        await settingOnSlot(3)

        fromFrontToEl()

        await grabElement()
        fromElToFrontBot()

        print("---------------\nSetting 2d Slot")
        await settingOnSlot(2)

        fromFrontToEl()

        await grabElement()
        fromElToFrontBot()

        print("---------------\nSetting 1st Slot")
        await settingOnSlot(1)

        fromFrontToEl()

        print("---------------\n0 pos")
        armTest.setServosOurAngle([0,0,0])'''
            
    except KeyboardInterrupt:
        GPIO.cleanup()

asyncio.run(main())