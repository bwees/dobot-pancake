from pygcode import Line, Machine
from pygcode import GCodeDwell
from dobot import DobotDllType as dType

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

api = dType.load()
print("done load")
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:", CON_STR[state])

def tool_change(tool):
    pass

def run_gcode(filename):
    gfile = open(filename)
    m = Machine()   
    
    for line in gfile:
        if line[0] == ";":
            continue

        # Pump On
        if "M106" in line:
            print("PUMP ON")
            continue

        # Pump Off
        if "M107" in line:
            print("PUMP OFF")
            continue

        gcodeLine = Line(line).block.gcodes
        if len(gcodeLine) > 0:
            m.process_gcodes(*gcodeLine)
            if type(gcodeLine[0]) == GCodeDwell:
                print(gcodeLine[0].get_param_dict("P")["P"])
                # ROBOT WAIT HERE
            else:
                # MOVE ROBOT
                continue