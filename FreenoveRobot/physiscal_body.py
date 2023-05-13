"""
    This file menage the entire robot api.

    @author: Stefano Fattore
    @last update: 9/5/22
"""

from lib.robotAPI.motor import Motor
from lib.robotAPI.ultrasonic import Ultrasonic
from lib.robotAPI.infrared import Infrared
from lib.robotAPI.button import Button
from lib.robotAPI.led import Led, Color
from lib.robotAPI.buzzer import Buzzer
from lib.workerthread import RobotThread
from lib.robotAPI.utils import ROBOTAPIConstants as RC
from lib.libMPU6050.MPUSensor import MPUSensor as MPU
from lib.libMPU6050.MPUSensor import MPUSensorException
from lib.libctrl.utility import Logger
from lib.libctrl.enums import Color as LoggerColor

from lib.librd.redisdata import BodyData


class PhysicalBodyException(Exception):
    pass


class PhysicalBody:
    def __init__(self) -> None:
        self.__arrow = None
        self.__wizard = None
        self.__strip = None
        self.__button = None
        self.__buzzer = None
        self.__infrared = None
        self.__right_sensor = None
        self.__front_sensor = None
        self.__left_sensor = None
        self.__motors = None
        self.__mpu6050 = None


    def virtual_destructor(self, logger: Logger) -> None:
        """
            This method realizes the virtualization of the destroyer, 
            invokes the equivalent for mpu6050, which takes care of 
            ending the thread that is responsible for capturing 
            the sensor information.

            IT MUST BE INVOKED.
        """
        logger.switch_context('PhysicalBody')
        logger.log('Arresting', LoggerColor.YELLOW)

        msg = self.__wizard.bury()
        logger.log(f'{msg}', LoggerColor.GRAY)

        if self.__arrow:
            msg = self.__arrow.bury()
            logger.log(f'{msg}', LoggerColor.GRAY)

        if BodyData.Yaw.is_enabled():
            msg = self.__mpu6050.virtual_destructor()
            logger.log(f'{msg}', LoggerColor.GRAY)

        msg = self.__infrared.virtual_destructor()
        logger.log(f'{msg}', LoggerColor.GRAY)

        self.__strip.colorWipe(Color(0, 0, 0), 10)

        logger.log('Arrested', LoggerColor.YELLOW)

    def begin(self, button_callback, logger: Logger) -> None:
        """
            This method invokes the equivalent for mpu6050 
            which commands the thread to begin performing its task.

            IT MUST BE INVOKED.

            Initialize sensors and actuators instances.
        """
        logger.switch_context('PhysicalBody')
        logger.log('Initializing', LoggerColor.YELLOW)

        if BodyData.Yaw.is_enabled():
            logger.log('Initializing MPU6050', LoggerColor.GRAY)
            try:
                self.__mpu6050 = MPU(RC.MPU_SMBUS_ID, RC.MPU_DEBUG_MODE)
                logger.log('MPU6050 initialized', LoggerColor.GRAY)
            except MPUSensorException as exc:
                logger.log(f'MPU6050 raised: {exc.args[0]}', LoggerColor.RED)
                raise PhysicalBodyException(exc.args[0])


        # motor instance
        self.__motors = Motor()
        logger.log('Motors      initialized', LoggerColor.GRAY)

        # ultrasonic instances
        self.__left_sensor = Ultrasonic(RC.LEFT_ECHO_PIN, RC.LEFT_TRIGGER_PIN)
        self.__front_sensor = Ultrasonic(RC.FRONT_ECHO_PIN, RC.FRONT_TRIGGER_PIN)
        self.__right_sensor = Ultrasonic(RC.RIGHT_ECHO_PIN, RC.RIGHT_TRIGGER_PIN)

        logger.log('Ultrasonics initialized', LoggerColor.GRAY)

        # infrared instance
        self.__infrared = Infrared()
        logger.log('Infrared    initialized', LoggerColor.GRAY)

        # buzzer instance
        self.__buzzer = Buzzer(RC.BUZZER_PIN)
        logger.log('Buzzer      initialized', LoggerColor.GRAY)

        self.__button = Button(RC.BUTTON_PIN, button_callback)
        logger.log('Button      initialized', LoggerColor.GRAY)

        # led strip instance
        self.__strip = Led()
        logger.log('Leds        initialized', LoggerColor.GRAY)

        self.__wizard = RobotThread(target=self.__strip.rainbowCycle, name='led_wizard',
                                    args=(RC.LED_ANIM_DELAY, RC.LED_ANIM_LOOPS,))

        logger.log('Leds RobotThread initialized, context: __strip.rainbowCycle', LoggerColor.GRAY)

        if BodyData.Yaw.is_enabled():
            self.__mpu6050.begin()

        self.__infrared.begin()

        logger.log('Initialized', LoggerColor.YELLOW, newline=True)

    def set_tuple_motor_model(self, data):
        rum, lum, rlm, llm = data
        self.__motors.left_upper_wheel(lum)
        self.__motors.left_lower_wheel(llm)
        self.__motors.right_upper_wheel(rum)
        self.__motors.right_lower_wheel(rlm)

    def read_distances(self) -> tuple:
        """
            Reads the values of the proximity sensors (centimeters), 
            returns a tuple containing the same.

            @return tuple: (int, int, int) -> (left, front, right)
        """
        _left = self.__left_sensor.distance
        _front = self.__front_sensor.distance
        _right = self.__right_sensor.distance

        return _left, _front, _right

    def infrared_status(self) -> tuple:
        """
            @Warning: multithread method.

            This only works if the begin method is invoked beforehand.
            Give back the values of the infrared sensors.

            @return tuple: (bool[int], bool[int], bool[int]) -> (left, mid, right)
        """
        return self.__infrared.status


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

    def blink_car_arrow(self, frlb) -> None:
        if self.__arrow is not None and self.__arrow.is_alive():
            self.__arrow.bury()
        self.__arrow = RobotThread(target=self.__strip.car_arrow, name='led_arrow', args=(frlb,))
        self.__arrow.start()

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
            self.__wizard = RobotThread(target=self.__strip.rainbowCycle, name='led_wizard', args=(RC.LED_ANIM_DELAY, RC.LED_ANIM_LOOPS,))
            self.__wizard.start()
        else:
            self.__strip.rainbowCycle()
    
    def interrupt_led(self) -> None:
        """
            Interrupt wizard thread if it is alive.
            Turn off leds.
        """
        if self.__wizard is not None and self.__wizard.is_alive():
            self.__wizard.bury()

        if self.__arrow is not None and self.__arrow.is_alive():
            self.__arrow.bury()

        self.__strip.colorWipe(Color(0, 0, 0), 10)
    
    def orientation(self) -> tuple:
        """
        It informs about the position of the robot on the 
        x, y, z axes also known as roll, pitch and yaw.
        This only works if the begin method is invoked beforehand.

        @return tuple: (int, int, int) -> (roll, pitch, yaw)
        """

        return self.__mpu6050.roll_pitch_yaw

