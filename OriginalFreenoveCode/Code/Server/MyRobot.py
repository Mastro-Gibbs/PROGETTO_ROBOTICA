"""
    Classe robot sviluppata durante il corso di robotica
    per controllare il robot kit 4WD SmartCar Freenove
"""

from ADC import Adc
from Ultrasonic import Ultrasonic
from Line_Tracking import Line_Tracking
from Buzzer import Buzzer
from Motor import Motor
from time import sleep


class MyRobot:

    def __init__(self):
        # Riferimenti ai SENSORI del robot
        self._adc = Adc()  # istanza del convertitore analogico/digitale per misurare grandezze fisiche
        self._sens_opt_sx = self._adc.recvADC(0)  # sensore di luce (foto-resistore) di sinistra
        self._sens_opt_dx = self._adc.recvADC(1)  # sensore di luce (foto-resistore) di destra
        self._ultrasonic = Ultrasonic()            # istanza del sottosistema ad ultrasuoni
        self._sens_distanza = 0                    # variabile che memorizza l'ultima lettura di distanza
        self._line_tracker = Line_Tracking()       # istanza del sottosistema di line tracking
        self._lt_sx = self._line_tracker.read_sx()   # variabile che memorizza il sensore ottico sinistro
        self._lt_cn = self._line_tracker.read_cn()   # variabile che memorizza il sensore ottico centro
        self._lt_dx = self._line_tracker.read_dx()   # variabile che memorizza il sensore ottico destra
        # Riferimenti agli ATTUATORI del robot
        self._buzzer = Buzzer()                    # istanza del sottosistema cicalino (buzzer)
        self._motors = Motor()                     # istanza del soottosistema dei motori (4 ruote indipendenti)


    def read_all_sensors(self):
        # inserire il codice per "rinfrescare" il valore di ciascun sensore
        self._sens_opt_sx = self._adc.recvADC(0)
        self._sens_opt_dx = self._adc.recvADC(1)
        self._sens_distance = self._ultrasonic.get_distance()
        self._lt_sx = self._line_tracker.read_sx()
        self._lt_cn = self._line_tracker.read_cn()
        self._lt_dx = self._line_tracker.read_dx()

    def luce_buio(self, valore):
        out = 'luce'
        if valore < 1.0:
            out = 'buio'
        return out

    def sensors_reading(self):
        self.read_all_sensors()
        l_sx = self.luce_buio(self._sens_opt_sx)
        l_dx = self.luce_buio(self._sens_opt_dx)
        out = f"LUCE: ({self._sens_opt_sx}, {self._sens_opt_dx}) ({l_sx}, {l_dx})\n"
        out += f"DIST: {self._sens_distance} \n"
        out += f"LTRK: ({self._lt_sx}, {self._lt_cn}, {self._lt_dx}) \n"
        return out

    def get_sensor_data(self):
        self.read_all_sensors()
        out = {}
        out["optical_sx"] = self._sens_opt_sx
        out["optical_dx"] = self._sens_opt_dx
        out["tracker_sx"] = self._lt_sx
        out["tracker_cn"] = self._lt_cn
        out["tracker_dx"] = self._lt_dx
        out["distance"] = self._sens_distance
        return out    
        
    def beep(self):
        self._buzzer.run('1')
        sleep(0.2)
        self._buzzer.run('0')

    def move_forward(self, veloc=4000):
        # fa muovere il robot in avanti
        self._motors.setMotorModel(veloc, veloc, veloc, veloc)

    def move_backward(self, veloc=2500):
        #muove i motori all'indietro
        self._motors.setMotorModel(-veloc, -veloc, -veloc, -veloc)

    def turn_left(self, veloc=2500):
        # ruota a sinistra il corpo del robot
        vgiro = int(0.2 * veloc)
        self._motors.setMotorModel(-veloc, -veloc, veloc, veloc)

    def turn_right(self, veloc=2500):
        # ruota a destra il corpo del robot
        vgiro = int(0.05 * veloc)
        self._motors.setMotorModel(veloc, veloc, -veloc, -veloc)


    def stop_motors(self):
        # ferma il robot dove si trova, stoppando i motori
        self._motors.setMotorModel(0, 0, 0, 0)

# Programma principale


if __name__ == '__main__':
    R = MyRobot()
    R.move_forward()
    R.beep()
    R.stop_motors()
    while True:
        print(R.sensors_reading())
        sleep(0.1)

