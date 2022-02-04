import utility
from pycsim import CSim, common

from PhysicalBody import PhysicalBody, Clockwise
import time
import math


def rotation_test(pb):
    pb.stop()
    time.sleep(0.5)
    pb.rotate(vel, Clockwise.RIGHT)
    time.sleep(5)
    pb.stop()
    pb.rotate(vel, Clockwise.LEFT)
    time.sleep(5)


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
            while not GO:
                pb.move_forward(4)
                print(pb.get_accelerometer())
                if pb.black_color_detected():
                    print("Black color detected")
                    GO = False
            GO = True

            while GO:
                pb.do_rotation(vel, Clockwise.RIGHT, 90)
                time.sleep(5)


        except common.NotFoundComponentError as e:
            print(e)
            print("Have you opened the right scene inside Coppelia SIM?")
            exit(-1)


