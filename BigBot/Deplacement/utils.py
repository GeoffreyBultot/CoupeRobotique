from enum import IntEnum
from math import sqrt
FORWARD = 1
BACKWARD = 0
ON = 1
OFF = 0


class Edir(IntEnum):
    DIR_FL = 0
    DIR_FR = 2
    DIR_RL = 4
    DIR_RR = 6


class Estate(IntEnum):
    STATE_FL  = 1
    STATE_FR  = 3
    STATE_RL  = 5
    STATE_RR  = 7


class Emotor(IntEnum):
    FL = 0
    FR = 1
    RL = 2
    RR = 3

def getDistance(list):
    return sqrt(list[0]**2 + list[1]**2)


def computeDict():
    stop = 0
    rotright = _rotationRight()
    rotleft = _rotationLeft()
    goforward = _goForward()
    goback = _goBackwards()
    left = _goLeft()
    right = _goRight()
    block = _block()
    dict = {'stop' : stop, 'rotationRight' : rotright, 'rotationLeft' : rotleft,
    'forward' : goforward,'backward' : goback, 'left' : left, 'right' : right,
    'block' : block}
    return dict


def stopMotors():
    return 0

def _block():
    val = 0
    val = set_value(val,OFF,Estate.STATE_FL)
    val = set_value(val,OFF,Estate.STATE_FR)
    val = set_value(val,OFF,Estate.STATE_RL)
    val = set_value(val,OFF,Estate.STATE_RR)

    val = set_value(val,FORWARD,Edir.DIR_FL)
    val = set_value(val,FORWARD,Edir.DIR_FR)
    val = set_value(val,FORWARD,Edir.DIR_RL)
    val = set_value(val,FORWARD,Edir.DIR_RR)
    print(val)
    return val

def _rotationRight():
    val = 0
    val = set_value(val,ON,Estate.STATE_FL)
    val = set_value(val,ON,Estate.STATE_FR)
    val = set_value(val,ON,Estate.STATE_RL)
    val = set_value(val,ON,Estate.STATE_RR)

    val = set_value(val,FORWARD,Edir.DIR_FL)
    val = set_value(val,BACKWARD,Edir.DIR_FR)
    val = set_value(val,FORWARD,Edir.DIR_RL)
    val = set_value(val,BACKWARD,Edir.DIR_RR)
    
    return val

def _rotationLeft():
    val = 0
    val = set_value(val,ON,Estate.STATE_FL)
    val = set_value(val,ON,Estate.STATE_FR)
    val = set_value(val,ON,Estate.STATE_RL)
    val = set_value(val,ON,Estate.STATE_RR)

    val = set_value(val,BACKWARD,Edir.DIR_FL)
    val = set_value(val,FORWARD,Edir.DIR_FR)
    val = set_value(val,BACKWARD,Edir.DIR_RL)
    val = set_value(val,FORWARD,Edir.DIR_RR)

    return val

def _goForward():
    val = 0
    val = set_value(val,ON,Estate.STATE_FL)
    val = set_value(val,ON,Estate.STATE_FR)
    val = set_value(val,ON,Estate.STATE_RL)
    val = set_value(val,ON,Estate.STATE_RR)

    val = set_value(val,FORWARD,Edir.DIR_FL)
    val = set_value(val,FORWARD,Edir.DIR_FR)
    val = set_value(val,FORWARD,Edir.DIR_RL)
    val = set_value(val,FORWARD,Edir.DIR_RR)

    return val

def _goBackwards():
    val = 0
    val = set_value(val,ON,Estate.STATE_FL)
    val = set_value(val,ON,Estate.STATE_FR)
    val = set_value(val,ON,Estate.STATE_RL)
    val = set_value(val,ON,Estate.STATE_RR)

    val = set_value(val,BACKWARD,Edir.DIR_FL)
    val = set_value(val,BACKWARD,Edir.DIR_FR)
    val = set_value(val,BACKWARD,Edir.DIR_RL)
    val = set_value(val,BACKWARD,Edir.DIR_RR)

    return val


def _goLeft():
    val = 0
    val = set_value(val,ON,Estate.STATE_FL)
    val = set_value(val,ON,Estate.STATE_FR)
    val = set_value(val,ON,Estate.STATE_RL)
    val = set_value(val,ON,Estate.STATE_RR)

    val = set_value(val,BACKWARD,Edir.DIR_FL)
    val = set_value(val,FORWARD,Edir.DIR_FR)
    val = set_value(val,FORWARD,Edir.DIR_RL)
    val = set_value(val,BACKWARD,Edir.DIR_RR)

    return val

def _goRight():
    val = 0
    val = set_value(val,ON,Estate.STATE_FL)
    val = set_value(val,ON,Estate.STATE_FR)
    val = set_value(val,ON,Estate.STATE_RL)
    val = set_value(val,ON,Estate.STATE_RR)

    val = set_value(val,FORWARD,Edir.DIR_FL)
    val = set_value(val,BACKWARD,Edir.DIR_FR)
    val = set_value(val,BACKWARD,Edir.DIR_RL)
    val = set_value(val,FORWARD,Edir.DIR_RR)
    

    return val


def _set_motor(n,dir,state): 
    val = 0
    if(dir):
        val = set_bit(val,2*n)
    else:
        val = clear_bit(val,2*n)
    if(state):
        val = set_bit(val,2*n + 1)
    else:
        val = clear_bit(val,2*n + 1)
    return val
    

def set_bit(value, bit):
    return value | (1<<bit)

def clear_bit(value, bit):
    return  value & ~(1<<bit)

def set_value(temp,value,bit):
    if(value):
        temp = set_bit(temp,bit)
    else:
        temp = clear_bit(temp,bit)
    return temp
