"""
from controller import Controller

c = Controller()

try:
    c.algorithm()

except KeyboardInterrupt:
    c.expire()


"""
from physical_body import PhysicalBody, ThreadType

if __name__ == "__main__":
    pb = PhysicalBody()
    try:
        pb.thread_begin(ThreadType.th_prox)
        pb.thread_begin(ThreadType.th_orientation)

        while True:
            print("FRONT: {0}, LEFT: {1}, RIGHT: {2}, BACK: {3}, GATE: {4}".format(pb.get_proxF(), pb.get_proxL(), pb.get_proxR(), pb.get_proxB(), pb.get_gate()))

    except KeyboardInterrupt:
        pb.safe_exit()
