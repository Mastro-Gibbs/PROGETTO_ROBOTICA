
from controller import Controller

c = Controller()
"""

import time

from physical_body import PhysicalBody, ThreadType

if __name__ == "__main__":
    pb = PhysicalBody()
    try:
        pb.thread_begin(ThreadType.th_proxF)

        pb.move_forward(8)

        pf = pb.get_proxF()
        while pf is None or pf > 0.15:
            pf = pb.get_proxF()

        pb.thread_kill(ThreadType.th_proxF)

        pb.stop()

        pb.thread_begin(ThreadType.th_orientation)

        ori = pb.get_orientation_deg()
        while ori is None or ori < 179:
            ori = pb.get_orientation_deg()
            pb.turn(-2.5, 2.5)

        pb.stop()

        pb.thread_kill(ThreadType.th_orientation)

        pb.thread_begin(ThreadType.th_proxF)

        pb.move_forward(8)

        pf = pb.get_proxF()
        while pf is None or pf > 0.15:
            pf = pb.get_proxF()

        pb.thread_kill(ThreadType.th_proxF)

        pb.stop()

        time.sleep(5)

    except KeyboardInterrupt:
        pb.safe_exit()
"""
