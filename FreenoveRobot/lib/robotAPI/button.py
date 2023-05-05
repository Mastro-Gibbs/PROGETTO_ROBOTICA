import RPi.GPIO as GPIO


class Button:
    def __init__(self, pin_no, _callback):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(pin_no, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        GPIO.add_event_detect(10, GPIO.RISING, callback=_callback)
