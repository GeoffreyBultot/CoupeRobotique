from dynamixel_sdk import *
from .utils import *
import sys, tty, termios

class Arm():
    NBR_SERVO                   = 3
    # Control table address
    ADDR_AX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
    ADDR_AX_GOAL_POSITION       = 30
    ADDR_AX_PRESENT_POSITION    = 36
    ADDR_AX_MOVING_SPEED        = 32
    ADDR_AX_TORQUE_LIMIT        = 34
    ADDR_AX_MAX_TORQUE          = 14

    # Data Byte Length
    LEN_AX_GOAL_POSITION       = 2
    LEN_AX_PRESENT_POSITION    = 2

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
    MID_POS_ID                  = [505, 508, 500]

    MAX_OVERALL_SPEED           = 100

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)

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

    
    def setMaxTorqueId(self, uid, perc):
        max_torque = int((1023/100)*perc)
        print(f"Setting {uid} with mov_speed {max_torque}")
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, uid, self.ADDR_AX_MAX_TORQUE, max_torque)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    
    def setMaxTorqueAll(self, perc):
        for i in range(0, self.NBR_SERVO):
            self.setMaxTorqueId(i, perc)

    
    def setTorqueLimit(self, uid, perc):
        torque_limit = int((1023/100)*perc)
        print(f"Setting {uid} with mov_speed {torque_limit}")
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, uid, self.ADDR_AX_TORQUE_LIMIT, torque_limit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    
    def setTorqueLimitAll(self, perc):
        for i in range(0, self.NBR_SERVO):
            self.setTorqueLimit(i, perc)


    def enableTorqueID(self, uid):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, uid, self.ADDR_AX_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def disableTorqueID(self, uid):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, uid, self.ADDR_AX_TORQUE_ENABLE, self.TORQUE_DISABLE)
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


    def changeIdMoveSpeed(self, uid, perc):
        mov_speed = int((1023/100)*perc)
        print(f"Setting {uid} with mov_speed {mov_speed}")
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, uid, self.ADDR_AX_MOVING_SPEED, mov_speed)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    
    def changeAllMoveSpeed(self, perc):
        for i in range(0, self.NBR_SERVO):
            self.changeIdMoveSpeed(i, perc) 


    def moveServoIdToGoalPos(self, uid, goal_position):
        if goal_position < self.MIN_POS_ID[uid]:
            print(f"Position {goal_position} for servo {uid} lower than MIN VALUE {self.MIN_POS_ID[uid]}")
            goal_position = self.MIN_POS_ID[uid]

        elif goal_position > self.MAX_POS_ID[uid]:
            print(f"Position {goal_position} for servo {uid} higher than MAX VALUE {self.MAX_POS_ID[uid]}")
            goal_position = self.MAX_POS_ID[uid]

        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, uid, self.ADDR_AX_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        self.enableTorqueID(uid)

    
    def moveAllServosToGoalPositions(self, goal_positions):
        param_goal_position0 = [DXL_LOBYTE(goal_positions[0]), DXL_HIBYTE(goal_positions[0])]
        dxl_addparam_result = self.groupSyncWrite.addParam(0, param_goal_position0)
        if dxl_addparam_result != True:
            print("0 groupSyncWrite addparam failed")

        param_goal_position1 = [DXL_LOBYTE(goal_positions[1]), DXL_HIBYTE(goal_positions[1])]
        dxl_addparam_result = self.groupSyncWrite.addParam(1, param_goal_position1)
        if dxl_addparam_result != True:
            print("1 groupSyncWrite addparam failed")

        param_goal_position2 = [DXL_LOBYTE(goal_positions[2]), DXL_HIBYTE(goal_positions[2])]
        dxl_addparam_result = self.groupSyncWrite.addParam(2, param_goal_position2)
        if dxl_addparam_result != True:
            print("2 groupSyncWrite addparam failed")

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()


    def readServoIdPos(self, uid):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, uid, self.ADDR_AX_PRESENT_POSITION)
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

    
    def setServosRealAngles(self, angles):
        positions = []

        for i in range(0, self.NBR_SERVO):
            pos = int(angles[i]/0.29)
            offset = self.MID_POS_ID[i]-512
            pos += offset
            positions.append(pos)

        self.moveAllServosToGoalPositions(positions)


    def setServoIdOurAngle(self, uid, angle):
        if uid == 0:
            newAngle = 240-angle
        elif uid == 1:
            newAngle = 150+angle
        elif uid == 2:
            newAngle = 150-angle
        else:
            print("Wrong ID sent")

        print(f"Setting servo {uid} to real angle {newAngle}")
        
        self.setServoRealAngle(uid, newAngle)

    def setMovSpeedBeforeMoving(self, angles):
        diffAngle = []
        for i in range(0, self.NBR_SERVO):
            pos = self.readServoIdPos(i)
            currentAngle = pos * 0.29
            diffAngle.append(abs(angles[i]-currentAngle))

        max_diff = max(diffAngle)
        
        for i in range(0, self.NBR_SERVO):
            if diffAngle[i] == max_diff:
                self.changeIdMoveSpeed(i, self.MAX_OVERALL_SPEED)
            else:
                percentage = (diffAngle[i]/max_diff)
                move_speed = int(self.MAX_OVERALL_SPEED*percentage)
                self.changeIdMoveSpeed(i, move_speed)

    
    def setServosOurAngle(self, angles):
        newAngles = []
        newAngles.append(240-angles[0])
        newAngles.append(150+angles[1])
        newAngles.append(150-angles[2])

        self.setMovSpeedBeforeMoving(newAngles)
        self.setServosRealAngles(newAngles)


    def setServoToXYWithAngle(self, x, y, phi):
        angles = returnAngleFromXYAndAngle(x, y, phi)

        self.setServosOurAngle(angles)


'''armTest = Arm()
armTest.enableTorqueAll()
#armTest.disableTorqueAll()
#armTest.setMaxTorqueAll(100)
#armTest.setTorqueLimitAll(100)
try:
    armTest.setServoOurAngle(0, 0)
    armTest.setServoOurAngle(1, 0)
    armTest.setServoOurAngle(2, 0)
    while 1:
        uid = int(input("Servo ID :\n"))
        angle = float(input("(0) our Angle position :\n"))
        armTest.setServoOurAngle(uid, angle)

        pass
except KeyboardInterrupt:
        armTest.disableTorqueAll()'''