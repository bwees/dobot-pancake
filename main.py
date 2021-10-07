from pygcode import Line, Machine
from pygcode import GCodeDwell, GCodeLinearMove
from pygcode.gcodes import GCodeFeedRate
from dobot import DobotDllType as dType

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

api = dType.load()
state = dType.ConnectDobot(api, "COM3", 115200)[0]
print("Connect status:", CON_STR[state])
dType.ClearAllAlarmsState(api)

home_offset = [200, -45, 0]

def tool_change(tool):
    pass

class PumpOn:
    def execute(self):
        return dType.SetEndEffectorGripper(api, True, True, isQueued=1)[0]

class PumpOff:
    def execute(self):
        return dType.SetEndEffectorGripper(api, True, True, isQueued=1)[0]

class PumpDisable:
    def execute(self):
        return dType.SetEndEffectorGripper(api, False, False, isQueued=1)[0]

class Move:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def execute(self):
        return dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, home_offset[0] + self.x, home_offset[1] + self.y, home_offset[2], 0, isQueued=1)[0]

class Feedrate:
    def __init__(self, feed):
        self.feed = feed

    def execute(self):
        return dType.SetPTPJointParams(api, self.feed, 1000, self.feed, 1000, self.feed, 1000, self.feed, 1000, isQueued=1)[0]

class Wait:
    def __init__(self, ms):
        self.ms = ms

    def execute(self):
        return dType.SetWAITCmd(api, self.ms, isQueued=1)[0]


def load_gcode_commands(filename):
    gfile = open(filename)
    m = Machine()   

    print("Processing GCODE...")
    commandList = []

    for line in gfile:
        # Comment
        if line[0] == ";":
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
            m.process_gcodes(*gcodeLine)
            if type(gcodeLine[0]) == GCodeDwell:
                # Set Robot Delay
                wait = gcodeLine[0].get_param_dict("P")["P"]
                commandList.append(Wait(wait))

            if len(gcodeLine) > 1 and type(gcodeLine[1]) == GCodeFeedRate:

                feed = int(str(gcodeLine[1])[1:])/60
                commandList.append(Feedrate(feed))

            else:
                # Move to position
                x, y, z = m.abs_pos.values.values()
                commandList.append(Move(x, y))
                    
    # Return last index of queue
    commandList.append(PumpDisable())

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

def showPlot(commands):
    extruding = False
    for c in commands:
        if type(c) == PumpOff or type(c) == PumpDisable:
            extruding = False
        if type(c) == PumpOn:
            extruding = True
        


def main():
    dType.SetQueuedCmdClear(api)

    dType.SetHOMEParams(api, 200, 200, 200, 200, 1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, 1)
    dType.SetPTPCommonParams(api, 100, 100, 1)

    if len(dType.GetAlarmsState(api)) > 0:
        print("HOMING ROBOT")
        dType.ClearAllAlarmsState(api)
        dType.SetHOMECmd(api, 0)

    executeQueue([Move(0,0)])

    commands = load_gcode_commands("pancake.gcode")
    showPlot(commands)
    print("Printing Pancake...")
    executeQueue(commands)
    
    dType.DisconnectDobot(api)


main()