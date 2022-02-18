"""
from controller import Controller

c = Controller()




"""

import time
import utility
from physical_body import PhysicalBody, ThreadType

if __name__ == "__main__":
    pb = PhysicalBody()
    try:
        pb.thread_begin(ThreadType.th_prox)

        while True:

            print("FRONT: {0}, LEFT: {1}, RIGHT: {2}, BACK: {3}".format(pb.get_proxF(), pb.get_proxL(), pb.get_proxR(), pb.get_proxB()))
            time.sleep(0.7)

        pb.stop()

    except KeyboardInterrupt:
        pb.safe_exit()
