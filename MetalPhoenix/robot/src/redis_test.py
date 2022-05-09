import controller
from time import sleep

c = controller.Controller()
degs = [180, -90, 0, 90]
old_ori = None

try:
    c.read_sensors()

    while c.orientation is None:
        sleep(0.01)
        c.read_sensors()

    j = 0
    for i in range(0, 10, 1):
        c.rotate_to_final_g(0.5, degs[j])

        if old_ori != c.orientation:
            print(c.orientation)
            old_ori = c.orientation

        sleep(8)
        j += 1
        if j == 4:
            j = 0

except KeyboardInterrupt:
    c.virtual_destructor()
    exit(1)
