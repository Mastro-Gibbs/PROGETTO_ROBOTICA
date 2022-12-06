import json
import random
import time

from math import pi

from lib.libctrl.remote_serial import RemoteController
from lib.librd.redisdata import ControllerData, RemoteControllerData

from lib.libctrl.utility import Compass, Clockwise
from lib.libctrl.utility import f_r_l_b_to_compass
from lib.libctrl.utility import negate_compass, detect_target
from lib.libctrl.utility import Logger, CFG
from lib.libctrl.utility import round_v, normalize_angle

from lib.libctrl.tree import Tree, Node, Type, DIRECTION

from lib.libctrl.enums import Command, Position, Mode, State
from lib.libctrl.enums import Color, STDOUTDecor

from lib.exit_codes import NOACTIONS, TREEUPDATEERROR

from redis import Redis
from redis.client import PubSubWorkerThread
from redis.exceptions import ConnectionError as RedisConnError


class ControllerException(Exception):
    pass


class test:
    def __init__(self):
        self.__robot_speed = 600

        try:
            self.__redis_message_handler = None
            self.__redis = Redis(host=ControllerData.Connection.Host, port=ControllerData.Connection.Port,
                                 decode_responses=True)
            self.__redis.flushall()
            self.__pubsub = self.__redis.pubsub()
            self.__pubsub.psubscribe(**{ControllerData.Topic.Body: self.__on_message})
            self.__pubsub.psubscribe(**{ControllerData.Topic.Remote: self.__on_remote})
        except RedisConnError or OSError or ConnectionRefusedError:
            raise ControllerException(f'Unable to connect to redis server at: '
                                      f'{ControllerData.Connection.Host}:{ControllerData.Connection.Port}')

        self.__remote = RemoteController() if RemoteControllerData.is_enabled else None

    def virtual_destructor(self) -> None:
        self.__redis_message_handler.stop()
        self.__redis.close()

        if self.__remote is not None:
            self.__remote.stop()

    def begin(self):
        self.__new_motor_values(0, 0, 0, 0, emit=True)
        self.__redis_message_handler: PubSubWorkerThread = self.__pubsub.run_in_thread(sleep_time=0.01)

        if self.__remote is not None:
            self.__remote.begin()

    def __on_remote(self, msg):
        _key = msg['data']
        _value = self.__redis.get(_key)

        if _key == RemoteControllerData.Key.RC:
            data = json.loads(_value)
            RemoteControllerData.on_values(data['rc_cmd'], data['rc_spd'])

    # callback receiver
    def __on_message(self, msg):
        _key = msg['data']
        ControllerData.Machine.on_values(self.__redis.get(_key))

    # sender method
    def __send_command(self, _cmd) -> None:  # RESET

        if _cmd == ControllerData.Command.Motor and ControllerData.Motor.changed:
            self.__redis.set(ControllerData.Key.Motor, ControllerData.Motor.values)
            self.__redis.publish(ControllerData.Topic.Controller, ControllerData.Key.Motor)

        elif _cmd == ControllerData.Command.Led and ControllerData.Led.changed:
            self.__redis.set(ControllerData.Key.Led, ControllerData.Led.leds())
            self.__redis.publish(ControllerData.Topic.Controller, ControllerData.Key.Led)

        elif _cmd == ControllerData.Command.Buzzer and ControllerData.Buzzer.changed:
            self.__redis.set(ControllerData.Key.Buzzer, ControllerData.Buzzer.status())
            self.__redis.publish(ControllerData.Topic.Controller, ControllerData.Key.Buzzer)

    def __execute_motor(self, cmd):
        if cmd == Command.STOP:
            self.__new_motor_values(0, 0, 0, 0, True)

        elif cmd == Command.RUN:
            self.__new_motor_values(-self.__robot_speed,
                                    -self.__robot_speed,
                                    -self.__robot_speed,
                                    -self.__robot_speed,
                                    True)

    def __new_motor_values(self, rum, lum, rlm, llm, emit: bool):
        ControllerData.Motor.on_values(rum, lum, rlm, llm)

        if emit:
            self.__send_command(ControllerData.Command.Motor)

    def __new_led(self, status: bool, arrow: bool = False, clockwise: Clockwise = None, emit: bool = False):
        ControllerData.Led.on_arrow(arrow, clockwise)
        ControllerData.Led.set(status)

        if emit:
            self.__send_command(ControllerData.Command.Led)

    def __new_buzzer(self, status: bool, emit: bool = False):
        ControllerData.Buzzer.set(status)

        if emit:
            self.__send_command(ControllerData.Command.Buzzer)

    def test(self):
        time.sleep(1)

        self.__execute_motor(Command.RUN)
        while ControllerData.Machine.front() > 10:
            print(f"\rFront prox test [1/4] => {ControllerData.Machine.front()} cm", end='')

        self.__execute_motor(Command.STOP)

        time.sleep(1)
        print()
        self.__execute_motor(Command.RUN)
        while ControllerData.Machine.left() > 10:
            print(f"\rLeft prox test [2/4] => {ControllerData.Machine.left()} cm", end='')

        self.__execute_motor(Command.STOP)

        time.sleep(1)
        print()
        self.__execute_motor(Command.RUN)
        while ControllerData.Machine.right() > 10:
            print(f"\rRight prox test [3/4] => {ControllerData.Machine.right()} cm", end='')

        self.__execute_motor(Command.STOP)
        print()

        print(f"\rGoal test [4/5] => {ControllerData.Machine.goal()} ", end='')
        print()

        time.sleep(5)
        print("RC test [5/5]")
        time.sleep(1)
        c = Clockwise(random.randint(0, 1))

        if self.__remote is not None:
            self.__remote.allow()
            self.__new_led(status=True, arrow=True, clockwise=c, emit=True)

            self.__new_buzzer(status=True, emit=True)
            time.sleep(0.1)
            self.__new_buzzer(status=False, emit=True)

            while not RemoteControllerData.is_rotation_done:
                time.sleep(0.01)

            self.__remote.dismiss()

            self.__new_led(status=False, emit=True)

        self.virtual_destructor()

if __name__ == "__main__":
    t = test()
    t.begin()

    try:
        t.test()
    except KeyboardInterrupt:
        t.virtual_destructor()
