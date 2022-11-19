from time import sleep
from controller import Controller, ControllerException


c = None
try:
    c = Controller()
    c.begin()

    c.rotate(180)

    sleep(10)



    c.virtual_destructor()

except KeyboardInterrupt:
    if c:
        c.virtual_destructor()

except ControllerException as e:
    print(e.args[0])