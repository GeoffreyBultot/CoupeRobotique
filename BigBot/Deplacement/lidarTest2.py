import os
import ydlidar
import math
import matplotlib.pyplot as plt
import threading
import json
import time
from statistics import median

def draw():
    global is_plot
    global final_x
    global final_y
    while is_plot:
        plt.figure(1)
        plt.cla()
        plt.ylim(-500,500)
        plt.xlim(-500,500)
        #print(f"X :\n{x}")
        #print(f"Y :\n{y}")
        plt.scatter(final_x, final_y, c='r', s=8)
        plt.pause(0.001)
    plt.close("all")

is_plot = True

distanceMax = 300 #mm
robotSize = 260 #mm

angles_count_dict = {}

final_x = [0]*360
final_y = [0]*360

size_gliss = 3
tresh = 0

for i in range(360):
    angles_count_dict[i] = []

threading.Thread(target=draw).start()

ydlidar.os_init();
laser = ydlidar.CYdLidar();
ports = ydlidar.lidarPortList();
port = "/dev/ydlidar";
for key, value in ports.items():
    port = value;

print(port)
laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000);
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF);
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True);

ret = laser.initialize();
if ret:
    ret = laser.turnOn();
    scan = ydlidar.LaserScan()
    while ret and ydlidar.os_isOk() :
        r = laser.doProcessSimple(scan);
        if r:
            angle = []
            ran = []
            intensity = []
            for p in scan.points:
              curr_angle = int(math.degrees(p.angle)+180)
              dist = (p.range)*0.254*1000
              
              #if curr_angle < 5:
                #print("Angle = " + str(curr_angle) + " : " + str(dist))
              
              angle.append(curr_angle)
              ran.append(dist)
              
              #med = median(angles_count_dict[curr_angle])
              final_x[curr_angle] = int(dist * math.cos(math.radians(curr_angle)))
              final_y[curr_angle] = int(dist * math.sin(math.radians(curr_angle)))
            
              '''if(dist > 0):
                
                if(dist > robotSize/2 and dist < distanceMax ):
                  print("Angle = " + str(curr_angle) + " : " + str(int(dist)))
              
                  
                    
                else:
                  final_x[curr_angle] = 0
                  final_y[curr_angle] = 0
              else:
                final_x[curr_angle] = 0
                final_y[curr_angle] = 0'''
            time.sleep(0.05)
              
        else:
            print("Failed to get Lidar Data.")
    laser.turnOff();
laser.disconnecting();