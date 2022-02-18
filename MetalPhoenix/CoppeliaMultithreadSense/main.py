
from controller import Controller

c = Controller()




"""

import time
import utility
from physical_body import PhysicalBody, ThreadType

if __name__ == "__main__":
    pb = PhysicalBody()
    try:
        pb.thread_begin(ThreadType.th_proxF)
        pb.thread_begin(ThreadType.th_proxL)
        pb.thread_begin(ThreadType.th_orientation)

        pb.move_forward(8)

        t = False
        while True:
            pf = pb.get_proxF()
            lp = pb.get_proxL()
            if pf is not None and pf < 0.165:
                break

            y = pb.get_orientation_deg()
            if (lp is None or lp > 0.25) and y is not None and not t:
                x = utility.normalize_compass(y, utility.Compass.EST)
                print(x)
                t = True

            if lp < 0.25:
                t = False

        pb.thread_kill(ThreadType.th_proxF)
        pb.thread_kill(ThreadType.th_proxL)
        pb.thread_kill(ThreadType.th_orientation)

        pb.stop()

    except KeyboardInterrupt:
        pb.safe_exit()

"""