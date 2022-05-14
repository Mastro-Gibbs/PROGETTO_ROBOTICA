import warnings
import functools
import inspect
import ctypes as ct
from enum import IntEnum
from threading import Thread


def deprecated(func):
    """
    This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emitted
    when the function is used.
    """

    @functools.wraps(func)
    def new_func(*args, **kwargs):
        warnings.simplefilter('always', DeprecationWarning)  # turn off filter
        warnings.warn("Call to deprecated function {}.".format(func.__name__),
                      category=DeprecationWarning,
                      stacklevel=2)
        warnings.simplefilter('default', DeprecationWarning)  # reset filter
        return func(*args, **kwargs)
    return new_func


def thread_ripper(th: Thread) -> bool:
    if th.is_alive():
        exctype = SystemExit
        tid = ct.c_long(th.ident)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ct.pythonapi.PyThreadState_SetAsyncExc(tid, ct.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            ct.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")
        
        return True
    return False

class ROBOTAPIConstants(IntEnum):
    FRONT_ECHO_PIN = 22,
    FRONT_TRIGGER_PIN = 27,

    LEFT_ECHO_PIN = 13,
    LEFT_TRIGGER_PIN = 26,

    RIGHT_ECHO_PIN = 5,
    RIGHT_TRIGGER_PIN = 6,

    BUZZER_PIN = 12
    DEFAULT_BUZZER_FREQ = 440,

    IR_LEFT = 14,
    IR_MID = 15,
    IR_RIGHT = 23,

    LED_COUNT = 8,        # Number of LED pixels.
    LED_PIN = 18,         # GPIO pin connected to the pixels (18 uses PWM!).
    LED_FREQ_HZ = 800000, # LED signal frequency in hertz (usually 800khz)
    LED_DMA = 10,         # DMA channel to use for generating signal (try 10)
    LED_BRIGHTNESS = 255, # Set to 0 for darkest and 255 for brightest
    LED_INVERT = False,   # True to invert the signal (when using NPN transistor level shift)
    LED_CHANNEL = 0,      # set to '1' for GPIOs 13, 19, 41, 45 or 53
    LED_ANIM_DELAY = 20,
    LED_ANIM_LOOPS = 5,

    MPU_SMBUS_ID = 1,
    MPU_DEBUG_MODE = 0