import inspect
import ctypes as ct
import RPi.GPIO as GPIO
from time import sleep
from threading import Thread
from lib.robotAPI.utils import ROBOTAPIConstants as RC
from lib.robotAPI.utils import thread_ripper

class Infrared:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RC.IR_LEFT,GPIO.IN)
        GPIO.setup(RC.IR_MID,GPIO.IN)
        GPIO.setup(RC.IR_RIGHT,GPIO.IN)

        self.__left_status: bool = False
        self.__mid_status: bool = False
        self.__right_status: bool = False

        self.__discover = Thread(target=self.__detect, name='ir_discover')

    def virtual_destructor(self):
        try:
            if thread_ripper(self.__discover):
                print(f"Thread {self.__discover.name} buried")
        except ValueError or SystemError as error:
            print(f"Issues while trying to kill the thread {self.__discover.name}")

    def begin(self):
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
        return self.__left_status, self.__mid_status, self.__right_status

    def __detect(self):
        while True:
            self.__left_status = True if self.__left_status else self.__left
            self.__mid_status = True if self.__mid_status else self.__mid
            self.__right_status = True if self.__right_status else self.__right

            sleep(0.05)

