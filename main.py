from pygcode import Line, Machine
from pygcode import GCodeDwell, GCodeLinearMove
from pygcode.gcodes import GCodeFeedRate, GCodeRapidMove
from dobot import DobotDllType as dType
from tqdm import tqdm
import time

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

api = dType.load()
state = dType.ConnectDobot(api, "COM3", 115200)[0]
print("Connect status:", CON_STR[state])
dType.ClearAllAlarmsState(api)

home_offset = [100, -60, 0]

def tool_change(tool):
    pass

class Home:
    def execute(self):
        return dType.SetHOMECmd(api, 0, isQueued=1)[0]

class PumpOn:
    def execute(self):
        return dType.SetEndEffectorGripper(api, True, False, isQueued=1)[0]

class PumpOff:
    def execute(self):
        return dType.SetEndEffectorGripper(api, False, False, isQueued=1)[0]

class PumpDisable:
    def execute(self):
        return dType.SetEndEffectorGripper(api, False, False, isQueued=1)[0]

class Move:
    def __init__(self, x, y):
        self.x = home_offset[0] + x
        self.y = home_offset[1] + y

    def execute(self):
        return dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, self.x, self.y, home_offset[2], 0, isQueued=1)[0]

class Feedrate:
    def __init__(self, feed):
        self.feed = feed

    def execute(self):
        return dType.SetPTPJointParams(api, self.feed, 10000, self.feed, 10000, self.feed, 10000, self.feed, 10000, isQueued=1)[0]

class Wait:
    def __init__(self, ms):
        self.ms = ms

    def execute(self):
        return dType.SetWAITCmd(api, self.ms, isQueued=1)[0]


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

        gcodeLine = Line(line).block.gcodes
        if len(gcodeLine) > 0:
            if type(gcodeLine[0]) == GCodeDwell:
                # Set Robot Delay
                wait = gcodeLine[0].get_param_dict("P")["P"]
                commandList.append(Wait(wait))

            if len(gcodeLine) > 1 and type(gcodeLine[1]) == GCodeFeedRate:

                feed = int(str(gcodeLine[1])[1:])/60
                commandList.append(Feedrate(feed))

            if type(gcodeLine[0]) == GCodeRapidMove:
                try:
                    x = gcodeLine[0].get_param_dict()["X"]
                    y = gcodeLine[0].get_param_dict()["Y"]
                    commandList.append(Move(x, y))
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

    chunk_size = 10
    chunk_set = chunks(queue, chunk_size)

    for c in chunk_set:
        toIndex = -1

        for op in c:
            toIndex = op.execute()

        dType.SetQueuedCmdStartExec(api)

        while toIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)
            
        dType.SetQueuedCmdStopExec(api)
        dType.SetQueuedCmdClear(api)

    dType.SetQueuedCmdClear(api)

def showPlot(commands):
    extruding = False
    for c in commands:
        if type(c) == PumpOff or type(c) == PumpDisable:
            extruding = False
        if type(c) == PumpOn:
            extruding = True        

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
    print("Done Homing")

def main():
    commands = load_gcode_commands("pancake.gcode")

    ####### STARTUP #######
    dType.ClearAllAlarmsState(api)
    dType.SetQueuedCmdClear(api)
    executeQueue([PumpDisable()])

    dType.SetHOMEParams(api, 200, 200, 200, 200, 1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, 1)
    dType.SetPTPCommonParams(api, 100, 100, 1)

    homeRobot()

    # Go to griddle home position
    executeQueue([Move(0,0)])

    ########################

    commands = load_gcode_commands("pancake.gcode")
    showPlot(commands)
    print("Printing Pancake...")
    executeQueue(commands)

main()
dType.DisconnectDobot(api)
dType.SetQueuedCmdClear(api)
