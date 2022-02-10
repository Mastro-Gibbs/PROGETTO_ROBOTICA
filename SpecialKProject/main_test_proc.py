import utility
from pycsim import CSim, common

from PhysicalBody import PhysicalBody, Clockwise
import time
import math
from enum import Enum


class Action(Enum):
    GO_FORWARD = 1
    GO_BACKWARD = 2
    ROTATE_LEFT = 3
    ROTATE_RIGHT = 4
    STOP = 5


actions = [1, 3, 1, 4, 1, 3, 1, 3, 1, 4, 1, 4, 1, 4, 1, 3]
actions = [1, 180, 1, 90, 1, 180, 1, -90, 1, 180, 1, 90, 1, 0, 1, 90, 1, 0, 1, 90, 1, 180,
           1, -90, 1, 180, 1, -90, 1, 180, 1, -90, 1, 180, 1, 90, 1, 0, 1, 90, 1, 180, 1,
           90, 1, 0, 1, 90, 1, 180, 1, 90, 1]


def rotation_test(pb, vel):
    pb.rotate_to_final_g(vel, 0)
    pb.rotate_to_final_g(vel, 90)
    pb.rotate_to_final_g(vel, -180)
    pb.rotate_to_final_g(vel, 90)
    pb.rotate_to_final_g(vel, -90)


def algorithm(pb):
    vel = 2
    vel_rot = 45 * math.pi / 180
    i = 0
    print(Action.GO_FORWARD)
    while len(actions) != i:
        print("f")
        if actions[i] == 1:
            while pb.get_front_distance() > 0.35:
                pb.move_forward(vel)
            pb.stop()
        elif actions[i] != 1:
            pb.rotate_to_final_g(vel_rot, actions[i])
        i += 1


if __name__ == '__main__':
    with CSim.connect("127.0.0.1", 19997) as api:
        api.simulation.start()
        try:
            print(api._id)
            pb = PhysicalBody(api)
            pb.stop()
            GO = True
            # time.sleep(2)
            print("Start")
            vel = 45 * math.pi / 180
            vel = 10
            # algorithm(pb)

            time.sleep(2)
            pb.start_proc()
            print(type(pb.get_g()))
            #rotation_test(pb, vel)
            while True:
                print("prc", pb.get_g())
                print(pb.get_orientation_degrees()[2])
                time.sleep(2)

            """ # rotation_test(pb, vel)
            pb.start_proc()
            # time.sleep(3)
            print("--------------", pb.get_g())
            # rotation_test(pb, vel)
            while True:
                print("--------------", pb.get_g())
                #time.sleep(3)"""

        except common.NotFoundComponentError as e:
            print(e)
            print("Have you opened the right scene inside Coppelia SIM?")
            exit(-1)
