import PyLidar3
import math
import time
import matplotlib.pyplot as plt
import threading
import json
from statistics import median

direction = {
    "stop":0,
    "Forward":1,
    "Backward":2,
    "Left":3,
    "Right":4
}

offset_angle = {
    "stop":0,
    "Forward":135,
    "Backward":315,
    "Left":45,
    "Right":225
}

dir = 'Forward'

distanceMax = 300 #mm
robotSize = 260 #mm

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

final_x = [0]*360
final_y = [0]*360

size_gliss = 3
tresh = 0

countTurn = 0

angles_count_dict = {}

for i in range(360):
    angles_count_dict[i] = {}
    angles_count_dict[i]['lisPosVal'] = []
    angles_count_dict[i]['count'] = 0
    angles_count_dict[i]['median'] = 0

PORT_LIDAR = "/dev/ttyUSB1"#"/dev/tty_LIDAR_USB"
Obj = PyLidar3.YdLidarX4(PORT_LIDAR)  #PyLidar3.your_version_of_lidar(port,chunk_size)

#threading.Thread(target=draw).start()
if(Obj.Connect()):
    print(Obj.GetDeviceInfo())
    print(Obj.GetHealthStatus())
    gen = Obj.StartScanning()
    t = time.time() # start time
    try:
        while (time.time() - t) < 30:#True: #scan for 30 seconds
            data = next(gen)
            countTurn += 1
            print(f"Total turn : {countTurn}", end="\r")
            '''print("--------------------NOUVO TOURENT--------------------")
            count = 0
            data = next(gen)
            for i in range(0,360):
                dist = data[i]

                val_count = angles_count_dict[i]['count']
                #list_angles = angles_count_dict[i]['gliss']
                #list_angles.pop(0)

                if(dist > 0):
                    if(dist > robotSize/2 and dist < distanceMax ):
                        #print(dist)
                        print("Angle = " + str(i) + " : " + str(int(dist)))

                        if len(angles_count_dict[i]['lisPosVal']) >= 5:
                            angles_count_dict[i]['lisPosVal'].pop(0)

                        angles_count_dict[i]['lisPosVal'].append(dist)
                        
                        #list_angles.append(True)

                        if val_count != size_gliss:
                            val_count += 1
                    else:
                        val_count -= 1
                        #list_angles.append(False)
                else:
                    val_count -= 1
                    #list_angles.append(False)

                #angles_count_dict[i]['gliss'] = list_angles
                angles_count_dict[i]['count'] = val_count

            #print("Filter")
            for i in range(360):
                if angles_count_dict[i]['count'] >= tresh:
                    med = median(angles_count_dict[i]['lisPosVal'])
                    angles_count_dict[i]['median'] = med
                    final_x[i] = int(med * math.cos(math.radians(i)))
                    final_y[i] = int(med * math.sin(math.radians(i)))
                    print(f"Angles {i} : count :\n")
                    print(json.dumps(angles_count_dict[i], sort_keys=False, indent=4))
                    print(f"final_x[i]: {int(final_x[i])}\nfinal_y[i]: {int(final_y[i])}")
                else:
                    final_x[i] = 0
                    final_y[i] = 0'''

    except Exception as e:
        print(e)
        print("[DEBUG] closing LIDAR")
        is_plot = False
        Obj.StopScanning()
        Obj.Disconnect()

    print("--------FINISHED--------")
    print(f"Total turn : {countTurn}")

    is_plot = False
    Obj.StopScanning()
    Obj.Disconnect()
else:
    print("Error connecting to device")