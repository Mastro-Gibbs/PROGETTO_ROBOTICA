from lib.robotAPI.PCA9685 import PCA9685
from enum import Enum

class Command(Enum):
    RUN = 1,
    STOP = 2,
    ROTATEL = 3,
    ROTATER = 4

class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)

    def justify(self, duty):
        if duty > 4095:
            duty = 4095
        elif duty < -4095:
            duty = -4095        
        
        return duty
        
    def left_upper_wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(0, 0)
            self.pwm.setMotorPwm(1, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(1, 0)
            self.pwm.setMotorPwm(0, abs(duty))
        else:
            self.pwm.setMotorPwm(0, 4095)
            self.pwm.setMotorPwm(1, 4095)

    def left_lower_wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(3, 0)
            self.pwm.setMotorPwm(2, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(2, 0)
            self.pwm.setMotorPwm(3, abs(duty))
        else:
            self.pwm.setMotorPwm(2, 4095)
            self.pwm.setMotorPwm(3, 4095)

    def right_upper_wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(6, 0)
            self.pwm.setMotorPwm(7, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(7, 0)
            self.pwm.setMotorPwm(6, abs(duty))
        else:
            self.pwm.setMotorPwm(6, 4095)
            self.pwm.setMotorPwm(7, 4095)

    def right_lower_wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(4, 0)
            self.pwm.setMotorPwm(5, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(5, 0)
            self.pwm.setMotorPwm(4, abs(duty))
        else:
            self.pwm.setMotorPwm(4, 4095)
            self.pwm.setMotorPwm(5, 4095)
            
 
    def set_model(self, cmd: Command, speed: int = 0):
        speed = self.justify(speed)

        if cmd == Command.STOP:
            self.left_upper_wheel(0)
            self.left_lower_wheel(0)
            self.right_upper_wheel(0)
            self.right_lower_wheel(0)
        elif cmd == Command.RUN:
            self.left_upper_wheel(-speed)
            self.left_lower_wheel(-speed)
            self.right_upper_wheel(-speed)
            self.right_lower_wheel(-speed)
        elif cmd == Command.ROTATEL:
            self.left_upper_wheel(speed)
            self.left_lower_wheel(speed)
            self.right_upper_wheel(-speed)
            self.right_lower_wheel(-speed)
        elif cmd == Command.ROTATER:
            self.left_upper_wheel(-speed)
            self.left_lower_wheel(-speed)
            self.right_upper_wheel(speed)
            self.right_lower_wheel(speed)

        
            

