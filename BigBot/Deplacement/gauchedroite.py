from Robot import *
import time
import utils

r = Robot()
time.sleep(1)

for i in range(40,16,-5):
    r.speed = i
    r.goLeft()
    time.sleep(i/100)
time.sleep(2)

r.stopMotors()
