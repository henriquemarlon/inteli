import time
import utils.DobotDllType as DType
from controllers.publisher_controller import Cycle


# Constants for connection messages

CON_STR = {
    DType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    DType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    DType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

# Dobot Class
class Dobot():

    # Initializes the Dobot

    def __init__(self, x = 0, y = 0, z = 0, r = 0):
        self.api = DType.load()
        self.x_home = x
        self.y_home = y
        self.z_home = z
        self.r_home = r
        self.connected = False
        self.picking = True
        self.verifyConnection()

    # Disconnects the Dobot

    def __del__(self):
        try:
            self.disconnect()
        except:
            print("Já está desconectado!")

    # Verifies Dobot's connection

    def verifyConnection(self):

        # If it's connected, just output its state
        # Otherwise, it will try to connect with it and
        # Then returns the status of connection. If the
        # connection is successfull, it will set all of
        # the base parameters.

        if(self.connected):
            print("Dobot already connected!")
        else:
            state = DType.ConnectDobot(self.api, "", 115200)[0]
            if(state == DType.DobotConnect.DobotConnect_NoError):
                print("Connect Status: ", CON_STR[state])
                DType.SetQueuedCmdClear(self.api)

                DType.SetHOMEParams(self.api, self.x_home, self.y_home, self.z_home, self.r_home, isQueued=1)
                DType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
                DType.SetPTPCommonParams(self.api, 100, 100, isQueued=1)

                DType.SetHOMECmd(self.api, temp=0, isQueued=1)
                self.connected = True
                return self.connected
            else:
                print("Unable to connect")
                print("Connect status:",CON_STR[state])
                return self.connected
            

    # Returns it to the Home position and disconnect

    def disconnect(self):
        self.moveHome()
        DType.DisconnectDobot(self.api)

    # Inserts a delay between commands

    def commandDelay(self, lastIndex):
        DType.SetQueuedCmdStartExec(self.api)
        while lastIndex > DType.GetQueuedCmdCurrentIndex(self.api)[0]:
            DType.dSleep(2000)
        DType.SetQueuedCmdStopExec(self.api)

    # Moves the Dobot to the coodinates (x,y,z,r)

    def moveArmXY(self,x,y,z,r):
        lastIndex = DType.SetPTPCmd(self.api, 2, x, y, z, r)[0]
        self.commandDelay(lastIndex)

    # Rotates the suction tool by 'rot' degrees
    
    def rotateTool(self, rot):
        x,y,z,r = self.getPos()
        self.moveArmXY(x,y,z,r)
        self.moveArmXY(x,y,z,r+rot)
        self.moveArmXY(x,y,z,r)
        
    # Draws a line with the specificated size  

    def drawLine(self,x,y,z,r,size):
        self.moveArmXY(x,y,z,r)
        self.moveArmXY(x+size,y,z,r)
        self.moveArmXY(x,y,z,r)

    # Returns to Default position

    def moveHome(self):
        lastIndex = DType.SetPTPCmd(self.api, 2, self.x_home, self.y_home, self.z_home, self.r_home, 1)[0]
        self.commandDelay(lastIndex)
    
    # Prints in terminal all the data from
    # It and them returns x,y,z,r in array form
    
    def getPos(self):
        (x, y, z, r, j1, j2, j3, j4) = DType.GetPose(self.api)
        print(f"X-Axis: {x}; Y-Axis: {y}; Z-Axis: {z}; R-Axis: {r}")
        print(f"Joint1: {j1}; Joint2: {j2}; Joint3: {j3}; Joint4: {j4};")
        return [x,y,z,r]

    def pickToggle(self):
        DType.SetHHTTrigMode(self.api, hhtTrigMode=1)
        DType.SetHHTTrigOutputEnabled(self.api, True)
        # lastIndex = DType.SetPTPCmd(self.api, DType.PTPMode.PTPMOVLXYZMode, self.x_home, self.y_home, 40, 0)[0]
        lastIndex = DType.SetEndEffectorGripper(self.api, 1, self.picking, isQueued=0)[0]
        self.picking = False if self.picking else True
        print(DType.GetHHTTrigOutput(self.api))
        self.commandDelay(lastIndex)
        

class Routine():
    def __init__(self, dobot):
        self.arm = Dobot(225, 3, 140, 0)
        self.cy = Cycle()
        self.arm.moveHome()
        self.arm.pickToggle()

    def start_routine(self):
        # #bandeja 1
        self.cy.run()
        self.arm.moveArmXY(174, 222, 77, 51)
        self.arm.moveArmXY(265, 175, -11, 32)
        self.arm.moveArmXY(64, 169, -11, 67)
        self.arm.moveArmXY(276, 202, -11, 35)
        self.arm.moveArmXY(66, 195, -11, 69)
        self.arm.moveArmXY(268, 241, -11, 41)
        self.arm.moveArmXY(64, 271, -11, 75)
        self.arm.moveArmXY(260, 266, -11, 44)
        self.arm.moveArmXY(174, 222, 77, 51)

        # #bandeja 2
        self.arm.moveHome()
        self.arm.moveArmXY(325, -36, -8, -7)
        self.arm.rotateTool(-90)
        self.arm.moveArmXY(181, -18, -8, -6)
        self.arm.moveArmXY(313, 52, -8, 8)
        self.arm.rotateTool(-80)
        self.arm.moveArmXY(177, 43, -8, 12)
        self.arm.moveHome()

        # #bandeja 3
        self.cy.stop()
        self.arm.moveArmXY(185, -229, 77, -51)
        self.arm.moveArmXY(185, -229, -10, -51)
        time.sleep(2)