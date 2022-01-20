import RPi.GPIO as GPIO
import asyncio
import time

class ServoStock():
    time_180    = 2500    #us
    time_0        = 500     #us

    def __init__(self, pinServo, frequency, modeGpio):
        self.Frequency = frequency
        self.servo = pinServo
        GPIO.setmode(modeGpio)
        GPIO.setup(self.servo, GPIO.OUT)
        self.p = GPIO.PWM(self.servo, self.Frequency) # GPIO7 for PWM with 100Hz; ~15ms
        self.p.start(0)

    def setAngle(self, angle):
        t_on = self.time_0+angle*(self.time_180 - self.time_0)/180
        DutyCycle = 100 * t_on/(1000000/self.Frequency)
        print("DutyCycle :", DutyCycle)
        self.p.ChangeDutyCycle(DutyCycle)

    def setDefault(self): #500uS
        self.setAngle(0)
        print("[DEBUG  ] Set servo stock to 0 degree")

    def setReverse(self): #2500uS
        self.setAngle(170) #180 doesn't work because it's 100% duty cycle
        print("[DEBUG  ] Set servo stock to 180 degrees")

    def stopPwm(self):
        self.p.ChangeDutyCycle(0)
        

'''print("Test")
try:
    servo = ServoStock(13, 400, GPIO.BCM)
    servo.setDefault()
    time.sleep(3)
    while True:
        servo.setDefault()
        time.sleep(3)
        servo.setReverse()
        time.sleep(3)

        servo.stopPwm()
        print("Steaup")
        time.sleep(2)
        #servo.startPwm()
        #print("Steaurt")
        #time.sleep(2)

except KeyboardInterrupt:
    GPIO.cleanup()'''