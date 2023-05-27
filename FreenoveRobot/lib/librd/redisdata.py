import json

from lib.libctrl.utility import __REDIS_CFG__, __SENSOR_CFG__, FRLB, CFG

def maxify(value) -> int:
    if value > 150:
        return 150
    else:
        return value

class __RedisData:
    class Connection:
        Host = __REDIS_CFG__['HOST']
        Port = __REDIS_CFG__["PORT"]

    class Topic:
        Body = __REDIS_CFG__["BODY_TOPIC"]
        Remote = __REDIS_CFG__["REMOTE_CONTROLLER_TOPIC"]
        Controller = __REDIS_CFG__["CTRL_TOPIC"]

    class Key:
        Btn = __REDIS_CFG__["BTN_KEY"]
        SELF = __REDIS_CFG__["SELF_KEY"]
        RC = __REDIS_CFG__["RC_KEY"]
        MPU = __REDIS_CFG__["MPU_KEY"]
        Led = __REDIS_CFG__["LED_KEY"]
        Motor = __REDIS_CFG__["MOTORS_KEY"]
        Buzzer = __REDIS_CFG__["BUZZER_KEY"]
        Infrared = __REDIS_CFG__["INFRARED_KEY"]
        Ultrasonic = __REDIS_CFG__["ULTRASONIC_KEY"]

    class Command:
        Led = 'Led'
        Motor = 'Motor'
        Buzzer = 'Buzzer'


class BodyData(__RedisData):
    class __Value:
        __status = False
        __value = False

        @classmethod
        def changed(cls):
            return cls.__status

        @classmethod
        def change(cls, _b: bool):
            cls.__status = _b

        @classmethod
        def set(cls, _b: bool):
            if cls.__value != _b:
                cls.__value = _b
                cls.__status = True

        @classmethod
        def status(cls) -> int:
            cls.__status = False
            return int(cls.__value)

    class Motor(__Value):
        __rum = None
        __lum = None
        __rlm = None
        __llm = None

        @classmethod
        def on_values(cls, jdata):
            data: dict = json.loads(jdata)

            if cls.__rum == data['rum'] and cls.__lum == data['lum'] and \
                    cls.__rlm == data['rlm'] and cls.__llm == data['llm']:
                super().change(False)
            else:
                cls.__rum = data['rum']
                cls.__lum = data['lum']
                cls.__rlm = data['rlm']
                cls.__llm = data['llm']

                super().change(True)

        @classmethod
        def values(cls):
            super().change(False)

            return cls.__rum, cls.__rum, cls.__rum, cls.__rum

    class Ultrasonic(__Value):
        __proxL = None
        __proxF = None
        __proxR = None

        @classmethod
        def on_values(cls, proxL, proxF, proxR):
            if cls.__proxL == proxL and cls.__proxF == proxF and cls.__proxR == proxR:
                super().change(False)
            else:
                cls.__proxL = maxify(proxL)
                cls.__proxF = maxify(proxF)
                cls.__proxR = maxify(proxR)
                super().change(True)

        @classmethod
        def values(cls):
            data = dict()
            data['proxL'] = cls.__proxL
            data['proxF'] = cls.__proxF
            data['proxR'] = cls.__proxR

            super().change(False)

            return json.dumps(data, indent=0)

    class Infrared(__Value):
        __irL = None
        __irM = None
        __irR = None

        @classmethod
        def on_values(cls, irL, irM, irR):
            if cls.__irL == irL and cls.__irM == irM and cls.__irR == irR:
                super().change(False)
            else:
                cls.__irL = irL
                cls.__irM = irM
                cls.__irR = irR
                super().change(True)

        @classmethod
        def values(cls):
            data = dict()
            data['irL'] = cls.__irL
            data['irM'] = cls.__irM
            data['irR'] = cls.__irR

            super().change(False)

            return json.dumps(data, indent=0)

    class Yaw(__Value):
        __enabled: bool = __SENSOR_CFG__["YAW_ENABLED"]
        __yaw = None

        @classmethod
        def is_enabled(cls):
            return cls.__enabled

        @classmethod
        def on_value(cls, yaw):
            if cls.__yaw == yaw:
                super().change(False)
            else:
                cls.__yaw = yaw
                super().change(True)

        @classmethod
        def value(cls):
            data = dict()
            data['Zaxis'] = cls.__yaw

            super().change(False)

            return json.dumps(data, indent=0)

    class Led(__Value):
        __arrow = 0
        __frlb = None

        @classmethod
        def on_arrow(cls, arrow: int, frlb):
            cls.__arrow = arrow
            cls.__frlb = frlb

        @classmethod
        def leds(cls):
            data: dict = dict()
            data['status'] = cls.status()
            data['arrow']  = cls.__arrow
            data['frlb'] = cls.__frlb

            cls.__arrow = False
            cls.__frlb = None

            return json.dumps(data, indent=0)

        @classmethod
        def arrow(cls):
            return int(cls.__arrow)

        @classmethod
        def direction(cls):
            return cls.__frlb


class ControllerData(__RedisData):
    class __Value:
        __status = False
        __value = False

        @classmethod
        @property
        def changed(cls):
            return cls.__status

        @classmethod
        def change(cls, _b: bool):
            cls.__status = _b

        @classmethod
        def set(cls, _b: bool):
            if cls.__value != _b:
                cls.__value = _b
                cls.__status = True

        @classmethod
        def status(cls) -> int:
            cls.__status = False
            return int(cls.__value)

    class Machine:
        __data: dict = dict()
        __data['irL'] = 0
        __data['irM'] = 0
        __data['irR'] = 0
        __data['proxL'] = 0
        __data['proxF'] = 0
        __data['proxR'] = 0
        __data['Zaxis'] = None
        __data['virtB'] = 0
        __data['ready'] = 0
        __goal = False

        __max = 17
        __min = 5

        @classmethod
        def on_values(cls, data):
            try:
                data = json.loads(data)
                cls.__data.update(data)

                if int(cls.__data['irL']) and int(cls.__data['irM']) and int(cls.__data['irR']):
                    cls.__goal = True
            except TypeError:
                pass

        @classmethod
        def front(cls):
            return int(cls.__data['proxF']) if int(cls.__data['proxF']) <= cls.__max else None

        @classmethod
        def left(cls):
            return int(cls.__data['proxL']) if int(cls.__data['proxL']) <= cls.__max else None

        @classmethod
        def right(cls):
            return int(cls.__data['proxR']) if int(cls.__data['proxR']) <= cls.__max else None

        @classmethod
        def goal(cls):
            return cls.__goal

        @classmethod
        def orientation(cls):
            return float(cls.__data['Zaxis']) if cls.__data['Zaxis'] else None

        @classmethod
        def left_ir(cls):
            return int(cls.__data['irL'])

        @classmethod
        def mid_ir(cls):
            return int(cls.__data['irM'])

        @classmethod
        def right_ir(cls):
            return int(cls.__data['irR'])

        @classmethod
        def connection(cls) -> bool:
            return bool(cls.__data['virtB'])

        @classmethod
        def ready(cls) -> bool:
            return bool(cls.__data['ready'])

        @classmethod
        def set_ready(cls, value: int):
            cls.__data['ready'] = value


    class Motor(__Value):
        __rum = None
        __lum = None
        __rlm = None
        __llm = None

        @classmethod
        def on_values(cls, rum: int, lum: int, rlm: int, llm: int):
            if cls.__rum == rum and cls.__lum == lum and cls.__rlm == rlm and cls.__llm == llm:
                super().change(False)
            else:
                cls.__rum = rum
                cls.__lum = lum
                cls.__rlm = rlm
                cls.__llm = llm

                super().change(True)

        @classmethod
        @property
        def values(cls):
            data = dict()
            data['rum'] = cls.__rum
            data['lum'] = cls.__lum
            data['rlm'] = cls.__rlm
            data['llm'] = cls.__llm

            super().change(False)

            return json.dumps(data, indent=0)

    class Led(__Value):
        __arrow = 0
        __frlb = None

        @classmethod
        def on_arrow(cls, arrow: bool, frlb: FRLB):
            cls.__arrow = int(arrow)
            cls.__frlb = frlb.value if frlb is not None else None

        @classmethod
        def leds(cls):
            data: dict = dict()
            data['status'] = cls.status()
            data['arrow']  = cls.__arrow
            data['frlb']   = cls.__frlb

            cls.__arrow = 0
            cls.__frlb = None

            return json.dumps(data, indent=0)

        @classmethod
        @property
        def arrow(cls):
            return int(cls.__arrow)

    class Buzzer(__Value):
        pass


class RemoteControllerData(__RedisData):
    __enabled = __REDIS_CFG__['RC_ENABLED']

    __speed = 600
    __command = None

    __engaged = False

    __fw = 'FORWARD'
    __bw = 'BACKWARD'

    __lf = 'LEFT'
    __rg = 'RIGHT'

    __st = 'STOP'

    __dn = 'DONE'

    @classmethod
    @property
    def is_enabled(cls):
        return cls.__enabled

    @classmethod
    @property
    def is_engaged(cls):
        return cls.__engaged

    @classmethod
    def engaged(cls, b: bool):
        cls.__engaged = b

    @classmethod
    @property
    def is_done(cls):
        return True if cls.__command == cls.__dn else False

    @classmethod
    def on_values(cls, command, speed):
        if speed is not None:
            cls.__speed = speed

        if command is not None:
            cls.__command = command

    @classmethod
    @property
    def values(cls):
        data = dict()
        data['rc_spd'] = cls.__speed
        data['rc_cmd'] = cls.__command

        return json.dumps(data, indent=0)

    @classmethod
    def is_valid(cls):
        return True if cls.__command is not None and cls.__command != cls.__dn else False

    @classmethod
    @property
    def is_rotation_done(cls):
        return True if cls.__command == cls.__dn else False

    @classmethod
    def get_motor(cls):
        if cls.__command == cls.__st:
            return 0, 0, 0, 0
        elif cls.__command == cls.__fw:
            return -cls.__speed, -cls.__speed, -cls.__speed, -cls.__speed
        elif cls.__command == cls.__bw:
            return cls.__speed, cls.__speed, cls.__speed, cls.__speed
        elif cls.__command == cls.__lf:
            return -cls.__speed, cls.__speed, -cls.__speed, cls.__speed
        elif cls.__command == cls.__rg:
            return cls.__speed, -cls.__speed, cls.__speed, -cls.__speed

