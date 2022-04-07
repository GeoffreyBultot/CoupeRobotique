import PyLidar3
import math
import time
import matplotlib.pyplot as plt
import threading

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

count_loop = 0

angles_count_dict = {}

for i in range(360):
    angles_count_dict[i] = 0

PORT_LIDAR = "/dev/ttyUSB1"#"/dev/tty_LIDAR_USB"
Obj = PyLidar3.YdLidarX4(PORT_LIDAR)  #PyLidar3.your_version_of_lidar(port,chunk_size)
threading.Thread(target=draw).start()
if(Obj.Connect()):
    print(Obj.GetDeviceInfo())
    gen = Obj.StartScanning()
    t = time.time() # start time
    try:
        while True: #scan for 30 seconds
            print("--------------------NOUVO TOURENT--------------------")
            count = 0
            data = next(gen)
            for i in range(0,360):
                dist = data[i]
                if(dist > 0):
                    if(dist > robotSize/2 and dist < distanceMax ):
                        #print(dist)
                        count+=1
                        print("Angle = " + str(i) + " : " + str(int(dist)))
                        val = angles_count_dict[i]
                        val += 1
                        angles_count_dict[i] = val

            '''if(count>10):
                #print(count)
                print("t tropres")'''

            count_loop+=1

            if count_loop % 5 == 0:
                print("Filter")
                for i in range(360):
                    if angles_count_dict[i] >= 3:
                        final_x[i] = int(data[i] * math.cos(math.radians(i)))
                        final_y[i] = int(data[i] * math.sin(math.radians(i)))
                        print(f"Angles {i} : count : {angles_count_dict[i]}")
                        print(f"final_x[i]: {int(final_x[i])}\nfinal_y[i]: {int(final_y[i])}")
                    else:
                        final_x[i] = 0
                        final_y[i] = 0

                    angles_count_dict[i] = 0


    except:
        print("[DEBUG] closing LIDAR")
        is_plot = False
        Obj.StopScanning()
        Obj.Disconnect()
    is_plot = False
    Obj.StopScanning()
    Obj.Disconnect()
else:
    print("Error connecting to device")