import ydlidar
import math
import threading
import time

is_object_in_dir={}
def lidar():
    global is_object_in_dir
    
    is_object_in_dir['front'] = False
    is_object_in_dir['right'] = False
    is_object_in_dir['back'] = False
    is_object_in_dir['left'] = False
    distanceMax = 300 #mm
    robotSize = 260 #mm
    size_gliss = 5

    last_positive = [False]*size_gliss

    ydlidar.os_init()
    laser = ydlidar.CYdLidar()

    port = "/dev/tty_LIDAR"

    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan = ydlidar.LaserScan()
        try:
            while ret and ydlidar.os_isOk() :
                r = laser.doProcessSimple(scan)
                if r:
                    frontFound = False
                    rightFound = False
                    leftFound = False
                    backFound = False
                    for p in scan.points:
                        last_positive.pop(0)
                        curr_angle = int(math.degrees(p.angle)+180)
                        dist = (p.range)*254 #*0.0254*10*1000 #Il renvoie le range en 1/10 de inches, *0.0254 => to dm, *10 to m, *1000 to mm

                        if robotSize/2 < dist < distanceMax:
                            last_positive.append(True)
                        else:
                            last_positive.append(False)

                        if False not in last_positive:
                            '''
                            ***FRONT***
                            ****90°****
                            0°*****180° RIGHT
                            ***270°****
                            ***BACK****
                            '''
                            #print(f"Caught object with angle {curr_angle}° and range : {round(dist/10, 2)}cm") 
                            if 45 <= curr_angle <= 135:
                                #print("Object on the front")
                                frontFound = True

                            elif 135 < curr_angle < 225:
                                #print("Object on the right")
                                rightFound = True

                            elif 225 <= curr_angle < 315:
                                #print("Object on the back")
                                backFound = True

                            elif (315 <= curr_angle <= 360) or (0 <= curr_angle < 45):
                                #print("Object on the left")
                                leftFound = True

                    if frontFound:
                        is_object_in_dir['front'] = True
                    else:
                        is_object_in_dir['front'] = False

                    if rightFound:
                        is_object_in_dir['right'] = True
                    else:
                        is_object_in_dir['right'] = False 

                    if backFound:
                        is_object_in_dir['back'] = True
                    else:
                        is_object_in_dir['back'] = False

                    if leftFound:
                        is_object_in_dir['left'] = True
                    else:
                        is_object_in_dir['left'] = False

                    time.sleep(0.05)
                    
                else:
                    print("Failed to get Lidar Data.")
            laser.turnOff()    
        except KeyboardInterrupt:
            laser.turnOff()
            laser.disconnecting()
        
    laser.disconnecting()


def brrrRobotTest():
    print("brrr je roule")
    dirRobot = {}
    dirRobot['front'] = False
    dirRobot['right'] = False
    dirRobot['back'] = True
    dirRobot['left'] = False
    while True:
        if dirRobot['front']: 
            if not is_object_in_dir['front']:
                pass#Robot.GoForward()
            else:
                print("OBJECT ON THE FRONT")

        if dirRobot['right']: 
            if not is_object_in_dir['right']:
                pass#Robot.GoRight()
            else:
                print("OBJECT ON THE RIGHT")

        if dirRobot['back']:
            if not is_object_in_dir['back']:
                pass#Robot.GoBack()
            else:
                print("OBJECT ON THE BACK")

        if dirRobot['left']:
            if not is_object_in_dir['left']:
                pass#Robot.GoLeft()
            else:
                print("OBJECT ON THE LEFT")
        
print("Ent started")
is_object_in_dir = {} #GLOBAL
threading.Thread(target=lidar).start()
threading.Thread(target=brrrRobotTest).start()
