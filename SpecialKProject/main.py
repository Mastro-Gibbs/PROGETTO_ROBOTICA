import utility
from pycsim import CSim, common
from PhysicalBody import PhysicalBody, Clockwise
import time
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
    vel = 5
    vel_rot = 45 * math.pi / 180
    i = 0
    direction = 90
    print(Action.GO_FORWARD)
    while len(actions) != i:
        print("f")
        if actions[i] == 1:
            while pb.get_front_distance() > 0.45:
                pb.move_forward(vel)
                pb.balance(direction)
            pb.stop()
        elif actions[i] != 1:
            pb.rotate_to_final_g(vel_rot, actions[i])
            direction = actions[i]
        i += 1


if __name__ == '__main__':
    with CSim.connect("127.0.0.1", 19997) as api:
        api.simulation.start()
        try:
            # print(api._id)
            pb = PhysicalBody(api)
            pb.stop()
            pb.setup_reference_system()
            GO = True
            # time.sleep(2)
            print("Start")
            vel = 45 * math.pi / 180
            vel = 3
            # algorithm(pb)
            # rotation_test(pb, vel)
            i = 0
            while True:
                if DEBUG:
                    print(f"[{pb.get_left_distance()},{pb.get_front_distance()}, {pb.get_right_distance()}]")

                pb.move_forward(vel)
                i += 2

                if i % (50 // vel) == 0:
                    pb.balance_line()

        except common.NotFoundComponentError as e:
            print(e)
            print("Have you opened the right scene inside Coppelia SIM?")
            exit(-1)
        except KeyboardInterrupt:
            print("\n\nSHUT DOWN!")
            exit(0)
