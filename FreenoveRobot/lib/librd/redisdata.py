import json

from lib.ctrllib.utility import __CFG__



class __RedisData:
    class Connection:
        Host = __CFG__['HOST']
        Port = __CFG__["PORT"]

    class Topic:
        Body = __CFG__["BODY_TOPIC"]
        Remote = __CFG__["REMOTE_CONTROLLER_TOPIC"]
        Controller = __CFG__["CTRL_TOPIC"]

    class Key:
        RC = __CFG__["RC_KEY"]
        MPU = __CFG__["MPU_KEY"]
        Led = __CFG__["LED_KEY"]
        Motor = __CFG__["MOTORS_KEY"]
        Buzzer = __CFG__["BUZZER_KEY"]
        Infrared = __CFG__["INFRARED_KEY"]
        Ultrasonic = __CFG__["ULTRASONIC_KEY"]

    class Command:
        Led = 'Led'
        Motor = 'Motor'
        Buzzer = 'Buzzer'


class BodyData(__RedisData):
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
        @property
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
                cls.__proxL = proxL
                cls.__proxF = proxF
                cls.__proxR = proxR
                super().change(True)

        @classmethod
        @property
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
        @property
        def values(cls):
            data = dict()
            data['irL'] = cls.__irL
            data['irM'] = cls.__irM
            data['irR'] = cls.__irR

            super().change(False)

            return json.dumps(data, indent=0)

    class Yaw(__Value):
        __yaw = None

        @classmethod
        def on_value(cls, yaw):
            if cls.__yaw == yaw:
                super().change(False)
            else:
                cls.__yaw = yaw
                super().change(True)

        @classmethod
        @property
        def value(cls):
            data = dict()
            data['Zaxis'] = cls.__yaw

            super().change(False)

            return json.dumps(data, indent=0)


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
        __goal = False

        @classmethod
        def on_values(cls, data):
            data = json.loads(data)

            cls.__data.update(data)

            if int(cls.__data['irL']) and int(cls.__data['irM']) and int(cls.__data['irR']):
                cls.__goal = True

            print(cls.__data)

        @classmethod
        def front(cls):
            return int(cls.__data['proxF']) if cls.__data['proxF'] != 'None' else None

        @classmethod
        def left(cls):
            return int(cls.__data['proxL']) if cls.__data['proxL'] != 'None' else None

        @classmethod
        def right(cls):
            return int(cls.__data['proxR']) if cls.__data['proxR'] != 'None' else None

        @classmethod
        def back(cls):
            return int(cls.__data['proxB']) if cls.__data['proxB'] != 'None' else None

        @classmethod
        def goal(cls):
            return cls.__goal

        @classmethod
        def z_axis(cls):
            return float(cls.__data['Zaxis']) if cls.__data['Zaxis'] != 'None' else None

        @classmethod
        def left_ir(cls):
            return int(cls.__data['irL']) if cls.__data['irL'] != 'None' else None

        @classmethod
        def mid_ir(cls):
            return int(cls.__data['irM']) if cls.__data['irM'] != 'None' else None

        @classmethod
        def right_ir(cls):
            return int(cls.__data['irR']) if cls.__data['irR'] != 'None' else None

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
        pass

    class Buzzer(__Value):
        pass


class RemoteControllerData(__RedisData):
    __enabled = __CFG__['RC_ENABLED']

    __speed = 600
    __command = None

    __fw = 'FORWARD'
    __bw = 'BACKWARD'

    __lf = 'LEFT'
    __rg = 'RIGHT'

    __st = 'STOP'

    @classmethod
    @property
    def is_enabled(cls):
        return cls.__enabled

    @classmethod
    def on_values(cls, command, speed):
        if speed is not None:
            cls.__speed = speed

        cls.__command = command

    @classmethod
    @property
    def values(cls):
        data = dict()
        data['rc_spd'] = cls.__speed
        data['rc_cmd'] = cls.__command

        return json.dumps(data, indent=0)

    @classmethod
    @property
    def is_valid(cls):
        return True if cls.__command is not None and cls.__command != 'DONE' else False

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

