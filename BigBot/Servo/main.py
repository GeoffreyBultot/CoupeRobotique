import RPi.GPIO as GPIO
import time

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 100) # GPIO 17 for PWM with 66Hz; ~15ms
p.start(2)
try:
    while True:
        p.ChangeDutyCycle(3)
        print("3%")
        time.sleep(5)

        p.ChangeDutyCycle(21)
        print("21%")
        time.sleep(0.5)
        p.ChangeDutyCycle(0)
        time.sleep(5)
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()