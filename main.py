from pygcode import Line, GCodeDwell
from pygcode.gcodes import GCodeRapidMove
from dobot import DobotDllType as dType
from tqdm import tqdm
import time
import turtle
import sys

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

api = dType.load()
state = dType.ConnectDobot(api, "COM4", 115200)[0]
print("Connect status:", CON_STR[state])

gcode_offset = [-43, 0, 0]
griddle_home = [200, 50, 75]

class Home:
    def execute(self):
        return dType.SetHOMECmd(api, 0, isQueued=1)[0]

    def __repr__(self):
        return "<HOME>"

class PumpOn:
    def execute(self):
        return dType.SetEndEffectorGripper(api, True, False, isQueued=1)[0]

    def __repr__(self):
        return "<PUMP_ON>"

class PumpOff:
    def execute(self):
        return dType.SetEndEffectorGripper(api, True, True, isQueued=1)[0]
    
    def __repr__(self):
        return "<PUMP_OFF>"

class PumpDisable:
    def execute(self):
        return dType.SetEndEffectorGripper(api, True, True, isQueued=1)[0]

    def __repr__(self):
        return "<PUMP_DISABLE>"

class Move:
    def __init__(self, x, y):
        self.x = griddle_home[0] + x
        self.y = griddle_home[1] + y

    def execute(self):
        return dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, self.x, self.y, griddle_home[2], 0, isQueued=1)[0]

    def __repr__(self):
        return "<MOVE x=" + str(self.x) + " y=" + str(self.y) + ">"

class Feedrate:
    def __init__(self, feed):
        self.feed = feed*60

    def execute(self):
        return dType.SetPTPJointParams(api, 300, 200, 300, 200, 300, 200, 300, 200, 1)[0]

    def __repr__(self):
        return "<FEEDRATE feed=" + str(self.feed) + ">"

class Wait:
    def __init__(self, ms):
        self.ms = ms

    def execute(self):
        return dType.SetWAITCmd(api, self.ms, isQueued=1)[0]

    def __repr__(self):
        return "<WAIT ms=" + str(self.ms) + ">"

    def __add__(self, other):
        return Wait(self.ms + other.ms)

    def __radd__(self, other):
        if other == 0:
            return self
        else:
            return self.__add__(other)

class SetIO:
    def __init__(self, port, level):
        self.port = port
        self.level = level

    def execute(self):
        dType.SetIODO(api, self.port, self.level, isQueued=1)

    def __repr__(self):
        return "<SETIO port=" + str(self.port) + " level=" + + str(self.level) + ">"

def load_gcode_commands(filename):
    gfile = open(filename)

    lines = gfile.read().split("\n")

    print("Processing GCODE...")
    commandList = []

    for line in tqdm(lines):
        # Comment
        if line == "" or line[0] == ";":
            continue

        # Pump On
        if "M106" in line:
            commandList.append(PumpOn())
            continue

        # Pump Off
        if "M107" in line:
            commandList.append(PumpOff())
            continue

        if "Help homing" in line:
            continue

        gcodeLine = Line(line).block.gcodes
        if len(gcodeLine) > 0:
            if type(gcodeLine[0]) == GCodeDwell:
                # Set Robot Delay
                wait = gcodeLine[0].get_param_dict("P")["P"]
                commandList.append(Wait(wait))

            if type(gcodeLine[0]) == GCodeRapidMove:
                try:
                    x = gcodeLine[0].get_param_dict()["X"] + gcode_offset[0]
                    y = gcodeLine[0].get_param_dict()["Y"] + gcode_offset[1]

                    commandList.append(Move(y, x))
                except KeyError:
                    pass
                
    # Return last index of queue
    commandList.append(PumpDisable())

    print("Parsed GCODE into", len(commandList), "commands.")

    return commandList

def chunks(l, n):
    n = max(1, n)
    return (l[i:i+n] for i in range(0, len(l), n))

def executeQueue(queue):

    chunk_size = 25
    chunk_set = chunks(queue, chunk_size)

    for c in chunk_set:
        toIndex = -1

        for op in c:
            toIndex = op.execute()
            print(op)

        dType.SetQueuedCmdStartExec(api)
        
        while toIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            time.sleep(0.1)
            
        dType.SetQueuedCmdStopExec(api)
        dType.SetQueuedCmdClear(api)

    dType.SetQueuedCmdClear(api)

def showPlot(commands):

    turtle.tracer(0, 0)
    turtle.color("red")


    # Find first movement point
    last_pt = None
    i = -1
    for x, c in enumerate(commands):
        if type(c) == Move:
            last_pt = c
            i = x
            break

    turtle.penup()
    turtle.goto(last_pt.x, last_pt.y)
    for c in commands[i+1:]:
        if type(c) == PumpOff or type(c) == PumpDisable:
            turtle.penup()
            continue
        if type(c) == PumpOn:
            turtle.pendown()   
            continue
        
        if type(c) == Move:
            turtle.goto(c.x, -c.y)

    turtle.update()

def homeRobot():
    dType.SetHOMECmd(api, 0)

    counterThresh = 15
    tCounter = 0
    lastPose = dType.GetPose(api)
    dType.SetQueuedCmdStartExec(api)

    while tCounter < counterThresh:
        if lastPose == dType.GetPose(api):
            tCounter += 1
        else:
            tCounter = 0

        lastPose = dType.GetPose(api)
        time.sleep(0.1)

    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)
    print("Done Homing")


def main():

    dType.ClearAllAlarmsState(api)

    if state == dType.DobotConnect.DobotConnect_Occupied:
        return 

    ####### STARTUP #######
    dType.SetQueuedCmdClear(api)
    dType.ClearAllAlarmsState(api)
    executeQueue([PumpOff()])

    dType.SetHOMEParams(api, 200, 200, 200, 200, 1)
    # dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, 1)

    homeRobot()

    # Go to griddle home position
    executeQueue([Move(0,0)])

    ########################

    try:
        commands = load_gcode_commands(sys.argv[1])

        # showPlot(commands)
        print("Printing Pancake...")
        executeQueue(commands)

    except IndexError:
        print("Please provide a file to print!")
        
    except FileNotFoundError:
        print("Inputted file was not found")
    

    dType.DisconnectDobot(api)

main()
