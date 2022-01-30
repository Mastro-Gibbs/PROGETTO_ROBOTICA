"""import os
import sys
from _ast import stmt

script_dir = os.path.dirname(__file__)
mymodule_dir = os.path.join(script_dir, '..', 'alpha', 'beta')

sys.path.append("/home/marco/Scrivania/ISRLAB/Pycharm/PycharmProjects/RemoteApiPython")

for p in sys.path:
    print(p)
"""
from RemoteApiPython import simConst as sc
from RemoteApiPython import sim as s
from pycsim import CSim

from math import pi
from time import sleep

def radians_to_degree(rad):
    return rad * 180 / pi


clientID = s.simxStart(
    connectionAddress="127.0.0.1",
    connectionPort=19997,
    waitUntilConnected=True,
    doNotReconnectOnceDisconnected=True,
    timeOutInMs=5000,
    commThreadCycleInMs=5)

"""
c = Connection()
clientID, api = c.connect()
print(api)
"""

def_op_mode = s.simx_opmode_oneshot_wait
world = "Robotnik_maze_2"
robot = "Freenove4wd"
#robot = "Robotnik_Summit_XL"
wall = "80cmHighWall200cm17"

if clientID != -1:
    s.simxStartSimulation(clientID, s.simx_opmode_oneshot_wait)
    print("connesso")
    print(clientID)
    code_r, handle_r = s.simxGetObjectHandle(clientID, robot, def_op_mode)
    handle_parent = s.sim_handle_parent

    #code_f, handle_f = s.simxGetObjectHandle(clientID, "Floor0", def_op_mode)
    # print(s.simxGetObjectPosition(code_r, handle_r, handle_parent, s.simx_opmode_streaming))

    # obbligatori
    s.simxGetObjectOrientation(code_r, handle_r, handle_parent, s.simx_opmode_streaming)
    sleep(0.2)  #  altrimenti per leggere correttamente l'orientazione ci vogliono tanti cicli
    c, arr = s.simxGetObjectOrientation(code_r, handle_r, handle_parent, s.simx_opmode_buffer) #obbligatorio fuori dal ciclo
    print(arr)
    in_pos = radians_to_degree(arr[2])
    # print(fabs(-2))
    i = 1
    go = True
    while go:
        print(i)
        # c, arr = s.simxGetObjectPosition(code_r, handle_r, handle_parent, s.simx_opmode_buffer)
        c, arr = s.simxGetObjectOrientation(code_r, handle_r, handle_parent, sc.simx_opmode_buffer)
        print(f"error:{c}")
        if c == 1:
            exit()
        print(arr)
        a = radians_to_degree(arr[0])
        b = radians_to_degree(arr[1])
        g = radians_to_degree(arr[2])
        print(f"[{a}], [{b}], [{g}]")
        difference = abs(g - in_pos)
        if 85 < difference < 95:
            print("90°")
            exit()
        i = i + 1
        sleep(0.1)

        # [0.0014319804031401873, -2.753866465354804e-05, -0.0001683259179117158] orientazione iniziale
        # [-0.00032220943830907345, -0.0015633389120921493, -1.6790106296539307] finale

        """returnCode = s.simxSetObjectOrientation(clientID, number
        objectHandle, number
        relativeToObjectHandle, array
        eulerAngles, number
        operationMode)"""

