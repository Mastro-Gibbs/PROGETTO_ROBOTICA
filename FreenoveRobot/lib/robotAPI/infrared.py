import RPi.GPIO as GPIO
from time import sleep

from lib.robotAPI.utils import ROBOTAPIConstants as RC
from lib.workerthread import RobotThread

class Infrared:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RC.IR_LEFT,GPIO.IN)
        GPIO.setup(RC.IR_MID,GPIO.IN)
        GPIO.setup(RC.IR_RIGHT,GPIO.IN)

        self.__left_status: bool = False
        self.__mid_status: bool = False
        self.__right_status: bool = False

        self.__discover = RobotThread(target=self.__detect, name='ir_discover')

    def virtual_destructor(self) -> str:
        return self.__discover.bury()

    def begin(self):
        if not self.__discover.is_alive():
            self.__discover.start()

    @property
    def __left(self) -> bool:
        return bool(GPIO.input(RC.IR_LEFT))

    @property
    def __mid(self) -> bool:
        return bool(GPIO.input(RC.IR_MID))

    @property
    def __right(self) -> bool:
        return bool(GPIO.input(RC.IR_RIGHT))

    @property
    def status(self) -> tuple:
        return int(self.__left_status), int(self.__mid_status), int(self.__right_status)

    def __detect(self):
        self.__left_status = False
        self.__mid_status = False
        self.__right_status = False

        while True:
            self.__left_status = True if self.__left_status else self.__left
            self.__mid_status = True if self.__mid_status else self.__mid
            self.__right_status = True if self.__right_status else self.__right

            sleep(0.05)

