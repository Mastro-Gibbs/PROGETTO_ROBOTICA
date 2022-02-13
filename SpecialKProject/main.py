from pycsim import CSim, common
from Controller.ControllerTest import ControllerTest

import math
from enum import Enum

DEBUG = False


class Action(Enum):
    GO_FORWARD = 1
    GO_BACKWARD = 2
    ROTATE_LEFT = 3
    ROTATE_RIGHT = 4
    STOP = 5


actions = [1, 3, 1, 4, 1, 3, 1, 3, 1, 4, 1, 4, 1, 4, 1, 3]



def rotation_test(pb, vel):
    pb.rotate_to_final_g(vel, 0)
    pb.rotate_to_final_g(vel, 90)
    pb.rotate_to_final_g(vel, -180)
    pb.rotate_to_final_g(vel, 90)
    pb.rotate_to_final_g(vel, -90)





if __name__ == '__main__':
    with CSim.connect("127.0.0.1", 19997) as api:
        api.simulation.start()
        try:
            # print(api._id)
            print("Start")
            ctrl = ControllerTest(api)

            # ctrl.algorithm()
            ctrl.algorithm()
        except common.NotFoundComponentError as e:
            print(e)
            print("Have you opened the right scene inside Coppelia SIM?")
            exit(-1)
        except KeyboardInterrupt:
            print("\n\nSHUT DOWN!")
            exit(0)
