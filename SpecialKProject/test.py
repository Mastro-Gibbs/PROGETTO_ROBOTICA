from RemoteApiPython import simConst as sc
from RemoteApiPython import sim as s
from time import sleep
from pycsim import CSim, common
#
# clientID = s.simxStart(
#     connectionAddress="127.0.0.1",
#     connectionPort=19997,
#     waitUntilConnected=True,
#     doNotReconnectOnceDisconnected=True,
#     timeOutInMs=5000,
#     commThreadCycleInMs=5)


# if clientID != -1:
#     print("connesso")
#     print(clientID)
#     robot = "Freenove4wd"
    # code_r, handle_r = s.simxGetObjectHandle(clientID, robot, s.simx_opmode_oneshot_wait)
    # s.simxGetObjectOrientation(code_r, handle_r, s.sim_handle_parent, s.simx_opmode_streaming)
    # sleep(0.2)
    # c, arr = s.simxGetObjectOrientation(code_r, handle_r, s.sim_handle_parent, s.simx_opmode_buffer)
    # print(arr)

if __name__ == '__main__':
    with CSim.connect("127.0.0.1", 19997) as api:
        api.simulation.start()
        try:
            print(api._id)

            robot = "Freenove4wd"
            code_r, handle_r = s.simxGetObjectHandle(api._id, robot, s.simx_opmode_oneshot_wait)
            s.simxGetObjectOrientation(code_r, handle_r, s.sim_handle_parent, s.simx_opmode_streaming)
            sleep(0.2)
            c, arr = s.simxGetObjectOrientation(code_r, handle_r, s.sim_handle_parent, s.simx_opmode_buffer)
            print(arr)

        except common.NotFoundComponentError as e:
            print(e)
            print("Have you opened the right scene inside Coppelia SIM?")
            exit(-1)




