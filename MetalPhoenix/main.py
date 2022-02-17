from pycsim import CSim, common
from Controller.ControllerTest import ControllerTest
import threading
import math
from enum import Enum

DEBUG = False


if __name__ == '__main__':
    with CSim.connect("127.0.0.1", 19997) as api:
        api.simulation.start()
        try:
            # print(api._id)
            print("Start")
            ctrl = ControllerTest(api)
            # ctrl.algorithm()
            ctrl.algorithm()
            # ctrl.kill_threads()
        except common.NotFoundComponentError as e:
            print(e)
            print("Have you opened the right scene inside Coppelia SIM?")
            exit(-1)
        except KeyboardInterrupt:
            print("\n\nSHUT DOWN!")
            # ctrl.kill_threads()
            exit(0)
