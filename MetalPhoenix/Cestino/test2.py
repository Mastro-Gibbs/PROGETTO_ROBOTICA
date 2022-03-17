from Cestino.pycsim import CSim, common
from Cestino.RemoteApiPython import simConst as sc, sim as s
from time import sleep
from math import pi

world = "Robotnik_maze_2"
robot = "Freenove4wd"


class RobotnikSummitXL:

    def __init__(self, api: CSim):
        print("Init")
        self._api = api

        self.clientID = api._id
        print(self.clientID)
        self.def_op_mode = sc.simx_opmode_oneshot_wait
        self.code_r, self.handle_r = s.simxGetObjectHandle(self.clientID, robot, self.def_op_mode)
        self.handle_parent = s.sim_handle_parent
        self.c, self.arr = s.simxGetObjectOrientation(self.code_r, self.handle_r, self.handle_parent,
                                                      s.simx_opmode_streaming)
        print(self.arr)

        # self.set_degree_orientation([self.degree_to_radians(0), self.degree_to_radians(0), self.degree_to_radians(0)])

    def get_degree_orientation(self):
        c, arr = s.simxGetObjectOrientation(self.code_r, self.handle_r, self.handle_parent, s.simx_opmode_buffer)
        a = self.radians_to_degree(arr[0])
        b = self.radians_to_degree(arr[1])
        g = self.radians_to_degree(arr[2])
        return [a, b, g]

    def radians_to_degree(self, rad):
        return rad * 180 / pi


if __name__ == "__main__":
    with CSim.connect("127.0.0.1", 19997) as api:
        # api.simulation.start()
        try:
            r = RobotnikSummitXL(api)
            # kl = keyboard.KeyboardListener()
            print("Start")
        except common.NotFoundComponentError as e:
            print(e)
            print("Have you opened the right scene inside Coppelia SIM?")
            exit(-1)
        sleep(0.2)
        print(r.get_degree_orientation())
