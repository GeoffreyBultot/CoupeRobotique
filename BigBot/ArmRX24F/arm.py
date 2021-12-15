from dynamixel_sdk import *
from utils import *
import sys, tty, termios

class Arm():
    NBR_SERVO                   = 3
    # Control table address
    ADDR_MX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
    ADDR_MX_GOAL_POSITION       = 30
    ADDR_MX_PRESENT_POSITION    = 36

    # Protocol version
    PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

    # Default setting
    BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
    DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

    MIN_POS_ID                  = [169, 177, 118]
    MAX_POS_ID                  = [936, 839, 788]
    MID_POS_ID                  = [512, 512, 512]

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    def __init__(self):
        self.initPortAndPacketHandler()
        pass
        
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


    def initPortAndPacketHandler(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            self.getch()
            quit()

        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            self.getch()
            quit()


    def enableTorqueID(self, uid):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, uid, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def disableTorqueID(self, uid):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, uid, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def enableTorqueAll(self):
        for i in range(0, self.NBR_SERVO):
            self.enableTorqueID(i)


    def disableTorqueAll(self):
        for i in range(0, self.NBR_SERVO):
            self.disableTorqueID(i)


    def moveServoIdToGoalPos(self, uid, goal_position):
        if goal_position < self.MIN_POS_ID[uid]:
            print(f"Position {goal_position} for servo {uid} lower than MIN VALUE {self.MIN_POS_ID[uid]}")
            goal_position = self.MIN_POS_ID[uid]

        elif goal_position > self.MAX_POS_ID[uid]:
            print(f"Position {goal_position} for servo {uid} higher than MAX VALUE {self.MAX_POS_ID[uid]}")
            goal_position = self.MAX_POS_ID[uid]

        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, uid, self.ADDR_MX_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        self.enableTorqueID(uid)


    def readServoIdPos(self, uid):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, uid, self.ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        return dxl_present_position


    def setServoRealAngle(self, uid, angle):
        pos = int(angle/0.29)
        offset = self.MID_POS_ID[uid]-512
        pos += offset
        print(f"Setting servo to pos : {pos} with offset : {offset}")
        self.moveServoIdToGoalPos(uid, pos)


    def setServoOurAngle(self, uid, angle):
        if uid == 0:
            newAngle = 150-angle
        elif uid == 1:
            newAngle = 150+angle
        elif uid == 2:
            newAngle = 150-angle
        else:
            print("Wrong ID sent")

        print(f"Setting servo {uid} to real angle {newAngle}")
        
        self.setServoRealAngle(uid, newAngle)

    
    def setServoToXYWithAngle(self, x, y, phi):
        [angle1, angle2, angle3] = returnAngleFromXYAndAngle(x, y, phi)
        self.setServoOurAngle(0, angle1)
        #time.sleep(1)
        self.setServoOurAngle(1, angle2)
        #time.sleep(1)
        self.setServoOurAngle(2, angle3)
        #time.sleep(1)


armTest = Arm()
armTest.enableTorqueAll()

while 1:
    uid = int(input("Servo ID :\n"))
    angle = float(input("Angle position :\n"))
    armTest.setServoOurAngle(uid, angle)
    '''x = float(input("X position :\n"))
    y = float(input("Y position :\n"))
    p = int(input("Phi angle  :\n"))

    armTest.setServoToXYWithAngle(x, y, p)'''
