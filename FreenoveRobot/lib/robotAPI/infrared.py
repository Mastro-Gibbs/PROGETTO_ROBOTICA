import inspect
import ctypes as ct
import RPi.GPIO as GPIO
from time import sleep
from threading import Thread
from lib.robotAPI.utils import ROBOTAPIConstants as RC

class Infrared:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RC.IR_LEFT,GPIO.IN)
        GPIO.setup(RC.IR_MID,GPIO.IN)
        GPIO.setup(RC.IR_RIGHT,GPIO.IN)

        self.__left_status: bool = False
        self.__mid_status: bool = False
        self.__right_status: bool = False

        self.__discover = Thread(target=self.__detect, name='discover')

    def virtual_destructor(self):
        if self.__discover.is_alive():
            exctype = SystemExit
            tid = ct.c_long(self.__discover.ident)
            if not inspect.isclass(exctype):
                exctype = type(exctype)
            res = ct.pythonapi.PyThreadState_SetAsyncExc(tid, ct.py_object(exctype))
            if res == 0:
                raise ValueError("invalid thread id")
            elif res != 1:
                ct.pythonapi.PyThreadState_SetAsyncExc(tid, None)
                raise SystemError("PyThreadState_SetAsyncExc failed")

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

