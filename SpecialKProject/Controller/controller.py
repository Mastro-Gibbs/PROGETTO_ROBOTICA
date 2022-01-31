"""

  - Sto in listening;
  - Ottengo le percezioni;
  - Le elaboro;
  - Produco un risultato;
  - Invio i comandi basati sul risultato;
  - Ritorno in listening;

"""

import redis
from queue import Queue
from math import pi
from time import sleep
from multiprocessing import Process

from controller_enums import Action, Key, Compass
from virtualbody import VirtualBody


class Controller:
    def __init__(self, redis: redis.Redis):
        self.us_left = None
        self.us_front = None
        self.us_right = None

        self.ir_left = None
        self.ir_mid = None
        self.ir_right = None

        self.orientation = Compass()    # orientazione corrente, utile solo al controller per l'algoritmo
        self.orientation_raw = None     # valore crudo della orientazione
        self.orientation_target = None  # target di orientazione
        self.balance_target = None

        self.action = Action()  # azione da compiere

        self.rotate_speed = (45 * pi) / 180  # velocità di rotazione
        self.movement_speed = 1.5            # velocità di movimento
        self.stop = 0                        # non c'è bisogno di commento :D
        self.velocity = 0                    # velocità corrente da inviare

        self.goal: bool = False

        self.queue = Queue()  # coda della soluzione

        self.redis = redis
        self.vbody = VirtualBody(redis)

        self.setupper = Process(target=self.__setup)
        self.setupper.start()

        self.setupper.join()

    def events(self) -> None:
        if not self.setupper.is_alive():
            self.setupper.start()
            self.setupper.join()

        self.__elaborate()
        self.__send()

    def __setup(self) -> None:
        while True:
            key = self.vbody.get_perceptions()

            if key == Key.USONIC:
                self.us_left, self.us_front, self.us_right = self.vbody.get_usonic()

            elif key == Key.IRED:
                self.ir_left, self.ir_mid, self.ir_right = self.vbody.get_ired()

            elif key == Key.POS:
                self.orientation_raw = self.vbody.get_orientation()

            sleep(0.005)

    def __send(self) -> None:
        if not self.goal:
            if self.action != Action.ROTATE_RIGHT and self.action != Action.ROTATE_LEFT:
                self.velocity = self.movement_speed
            elif self.action == Action.STOP:
                self.velocity = self.stop
            else:
                self.velocity = self.rotate_speed

            self.vbody.send_command(Key.MOTOR, self.action)
            self.vbody.send_command(Key.MOTOR, self.orientation_target)
            self.vbody.send_command(Key.MOTOR, self.velocity)

        else:
            self.velocity = self.stop
            self.action = Action.STOP

            self.vbody.send_command(Key.MOTOR, self.action)
            self.vbody.send_command(Key.MOTOR, self.orientation_target)
            self.vbody.send_command(Key.MOTOR, self.velocity)
            self.vbody.send_command(Key.BUZZER, 1)
            sleep(2)
            self.vbody.send_command(Key.BUZZER, 0)

    def __elaborate(self):
        self.__calibrate_position()

    def __algoritm(self):
        pass

    def __calibrate_position(self) -> None:
        if self.__adjusts_orientation():
            self.vbody.send_command(Key.MOTOR, self.action)
            self.vbody.send_command(Key.MOTOR, self.orientation_target)
            self.vbody.send_command(Key.MOTOR, self.velocity)

            self.__balance()
            self.vbody.send_command(Key.MOTOR, self.action)
            self.vbody.send_command(Key.MOTOR, self.balance_target)
            self.vbody.send_command(Key.MOTOR, self.velocity)

            self.__restore()
            self.vbody.send_command(Key.MOTOR, self.action)
            self.vbody.send_command(Key.MOTOR, self.orientation_target)
            self.vbody.send_command(Key.MOTOR, self.velocity)

    def __adjusts_orientation(self) -> None:
        if self.action != Action.ROTATE_RIGHT and self.action != Action.ROTATE_LEFT and self.action != Action.STOP:
            if self.orientation == Compass.NORD:
                self.orientation_target = 0
            elif self.orientation == Compass.EST:
                self.orientation_target = -90
            elif self.orientation == Compass.OVEST:
                self.orientation_target = 90
            elif self.orientation == Compass.SUD:
                self.orientation_target = 180

            if self.orientation_raw > (self.orientation_target + 5):
                self.action = Action.ROTATE_RIGHT
            elif self.orientation_raw < (self.orientation_target - 5):
                self.action = Action.ROTATE_LEFT
        else:
            return False

        return True

    """
        da migliorare.. calcolare staticamente la start pos della 4wd, -> autobilanciamento!
        ci mancano i valori...
    """
    def __balance(self) -> None:
        orientation = None

        diff = self.us_left - self.us_right

        if diff < 0:
            orientation = Action.ROTATE_RIGHT
        else:
            orientation = Action.ROTATE_LEFT

        diff = abs(diff)

        self.action = orientation
        self.balance_target = diff

    def __restore(self) -> None:
        if self.action == Action.ROTATE_RIGHT:
            self.action = Action.ROTATE_LEFT
        elif self.action == Action.ROTATE_LEFT:
            self.action = Action.ROTATE_RIGHT




