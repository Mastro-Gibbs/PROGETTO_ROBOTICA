"""
    This file menage the entire robot api.

    @author: Stefano Fattore
    @last update: 9/5/22
"""
import inspect
import ctypes as ct
from threading import Thread
from lib.robotAPI.motor import Motor, Command
from lib.robotAPI.ultrasonic import Ultrasonic
from lib.robotAPI.led import Led, Color
from lib.robotAPI.buzzer import Buzzer
from lib.robotAPI.utils import deprecated
from lib.robotAPI.utils import ROBOTAPIConstants as RC
from lib.MPU6050lib.MPUSensor import MPUSensor as MPU 


class PhysicalBody:
    def __init__(self) -> None:
        """
            Initialize sensors and actuators instances.
        """

        # motor instance
        self.__motors = Motor()

        # ultrasonic instances
        self.__left_sensor = Ultrasonic(RC.LEFT_ECHO_PIN, RC.LEFT_TRIGGER_PIN)
        self.__front_sensor = Ultrasonic(RC.FRONT_ECHO_PIN, RC.FRONT_TRIGGER_PIN)
        self.__right_sensor = Ultrasonic(RC.RIGHT_ECHO_PIN, RC.RIGHT_TRIGGER_PIN)

        # buzzer instance
        self.__buzzer = Buzzer(RC.BUZZER_PIN)

        # led strip instance
        self.__strip = Led()
        self.__wizard = Thread(target=self.__strip.rainbowCycle, name='wizard', args=(20, 5,))

        # orientation sensor instance (MPU6050)
        self.__mpu6050 = MPU(RC.MPU_SMBUS_ID, RC.MPU_DEBUG_MODE)


    def virtual_destructor(self) -> None:
        """
            This method realizes the virtualization of the destroyer, 
            invokes the equivalent for mpu6050, which takes care of 
            ending the thread that is responsible for capturing 
            the sensor information.

            IT MUST BE INVOKED.
        """
        if self.__wizard.is_alive():
            exctype = SystemExit
            tid = ct.c_long(self.__wizard.ident)
            if not inspect.isclass(exctype):
                exctype = type(exctype)
            res = ct.pythonapi.PyThreadState_SetAsyncExc(tid, ct.py_object(exctype))
            if res == 0:
                raise ValueError("invalid thread id")
            elif res != 1:
                ct.pythonapi.PyThreadState_SetAsyncExc(tid, None)
                raise SystemError("PyThreadState_SetAsyncExc failed")

        self.__mpu6050.virtual_destructor()
        self.__strip.colorWipe(Color(0,0,0), 10)

    def begin(self) -> None:
        """
            This method invokes the equivalent for mpu6050 
            which commands the thread to begin performing its task.

            IT MUST BE INVOKED.
        """
        self.__mpu6050.begin()


    def set_motor_model(self, cmd: Command, rate: int = 0) -> None:
        """
            Set speed value for motors.

            @param cmd: instance of Command -> {RUN, STOP, ROTATEL, ROTATER}
            @param rate: percentage of desired speed between 0% and 100%
        """

        if rate > 100:
            rate = 100
        elif rate < 0:
            rate = 0
        
        _vel = (rate * 4095) // 100

        self.__motors.set_model(cmd, _vel)

    def read_distances(self) -> tuple:
        """
            Reads the values of the proximity sensors (centimeters), 
            returns a tuple containing the same.

            @return tuple: three elements of type int; 
                           return order -> [left, front, right]
        """
        _left = self.__left_sensor.distance
        _front = self.__front_sensor.distance
        _right = self.__right_sensor.distance

        return _left, _front, _right


    def trill(self, freq: int = -1, duty: int = -1) -> None:
        """
            Lets it make a sound.

            Part of this method is deprecated, 
            don't use parameters when the call is invoked.
        """
        if freq == -1 and duty == -1:
            self.__buzzer.emit()
        else:
            # deprecated method
            # self.__buzzer.play(duty, freq)
            pass

    def interrupt_trill(self) -> None:
        """
            Interrupt the emitted sound. 
        """
        self.__buzzer.stop()

    
    def magic_rainbow(self, mode: bool = False) -> None:
        """
            @Warning: possible multithread method.
            @param mode: boolean -> True: activate multithread
                                    False: deactivated multithread/normal mode

            It emits a wonderful animation through the LEDs.
            It turns out to be a blocking method if called in normal mode, 
            otherwise a thread will unleash and take care of the task.
            The thread will live as long as the animation is not finished or 
            virtual destructor will be called.
        """
        if mode:
            self.__wizard.start()
        else:
            self.__strip.rainbowCycle()

    def oritentation(self) -> tuple:
        """
        It informs about the position of the robot on the 
        x, y, z axes also known as roll, pitch and yaw.
        This only works if the begin method is invoked beforehand.

        @return tuple: (int, int, int) -> (roll, pitch, yaw)
        """

        return self.__mpu6050.roll_pitch_yaw

