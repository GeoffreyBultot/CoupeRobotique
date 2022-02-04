from Robot import *
import time
import utils

r = Robot()
time.sleep(1)

stepsGallery = stepsFromCm(70)
stepsStart = stepsFromCm(77)
print(stepsStart)
print(stepsGallery)

tour = 5435
r.rotationLeft(1)
time.sleep(0.1)


r.goForward(stepsStart)
print(r.stepsForAngle(103))
r.rotationLeft(r.stepsForAngle(103))
time.sleep(5)
time.sleep(5)
r.stopMotors()
