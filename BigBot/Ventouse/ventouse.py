import RPi.GPIO as GPIO
import asyncio
import time

class Ventouse():
    def __init__(self, pinElectroValve, pinAirPump, modeGpio):
        self.electroValve = pinElectroValve
        self.airPump = pinAirPump
        GPIO.setmode(modeGpio)

        GPIO.setup(pinElectroValve, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(pinAirPump, GPIO.OUT, initial=GPIO.LOW)

    def setDefault(self):
        GPIO.output(self.electroValve, 0)
        GPIO.output(self.airPump, 0)

    def sunct(self):
        GPIO.output(self.airPump, 1)
        time.sleep(1)
        GPIO.output(self.airPump, 0)

    def drop(self):
        GPIO.output(self.electroValve, 1)
        time.sleep(0.2)
        GPIO.output(self.electroValve, 0)

'''async def main():
    try:
        ventouse = Ventouse(2, 3, GPIO.BCM)
        #await ventouse.setDefault()
        while True:
            value = input("Enter value S (sunct) or D (drop):\n")
            if value =="S":
                await ventouse.sunct()
            elif value == "D":
                await ventouse.drop()

            #await asyncio.sleep(2)
            
    except KeyboardInterrupt:
        GPIO.cleanup()

asyncio.run(main())'''