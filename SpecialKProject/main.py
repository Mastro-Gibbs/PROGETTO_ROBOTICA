from pycsim import CSim, common

from VirtualBody import VirtualBody, Clockwise
import time
import math


if __name__ == '__main__':
    with CSim.connect("127.0.0.1", 19997) as api:
        api.simulation.start()
        try:
            print(api._id)
            vb = VirtualBody(api)
            vb.stop()
            time.sleep(2)
            print("Start")
            vel = 45 * math.pi / 180
            while True:
                # vb.turn(-5, +5)
                vb.stop()
                time.sleep(0.5)
                vb.rotate(vel, Clockwise.RIGHT)
                time.sleep(5)
                vb.stop()
                vb.rotate(vel, Clockwise.LEFT)
                time.sleep(5)

                # print(vb.get_degree_orientation())

                #print("distance: ", vb.get_left_distance())

                # time.sleep(2)
                # vb.move_forward(10)
                # time.sleep(2)
                # vb.stop()
                # time.sleep(1)
                # vb.move_backward(1)
                # time.sleep(1)


        except common.NotFoundComponentError as e:
            print(e)
            print("Have you opened the right scene inside Coppelia SIM?")
            exit(-1)



