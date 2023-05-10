import RPi.GPIO as GPIO


class Buzzer:
    def __init__(self, pin: int):
        GPIO.setwarnings(False)
        self.__pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__pin, GPIO.OUT)

    def emit(self):
        GPIO.output(self.__pin, True)

    def stop(self):
        GPIO.output(self.__pin, False)
