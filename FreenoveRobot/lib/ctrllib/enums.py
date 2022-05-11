from enum import Enum
import configparser

def local_CFG_parser() -> dict:
    psr = configparser.ConfigParser()
    psr.read('data/config.conf')
    return {
        "HOST": psr["REDIS"]["host"],
        "PORT": psr["REDIS"]["port"],
        "BODY_TOPIC": psr["REDIS"]["body_topic"],
        "CTRL_TOPIC": psr["REDIS"]["ctrl_topic"],
        "LED": psr["REDIS"]["led_key"],
        "BUZZER": psr["REDIS"]["buzzer_key"],
        "INFRARED": psr["REDIS"]["infrared_key"],
        "ULTRASONIC": psr["REDIS"]["ultrasonic_key"],
        "MPU": psr["REDIS"]["mpu_key"],
        "MOTORS": psr["REDIS"]["motors_key"]
    }


class State(Enum):
    STARTING = -1
    STOPPED = 0
    RUNNING = 1
    ROTATING = 2
    SENSING = 3


class Position(Enum):
    INITIAL = 0
    CORRIDOR = 1
    JUNCTION = 2


class Command(Enum):
    START = -1
    STOP = 0
    RUN = 1
    ROTATE = 2  # ?
    GO_TO_JUNCTION = 3


class Mode(Enum):
    EXPLORING = 0
    ESCAPING = 1


class Compass(float, Enum):
    NORD = 90.0
    SUD = -90.0
    EST = 0.0
    OVEST = 180.0


class Clockwise(Enum):
    RIGHT = 0
    LEFT = 1


class WAY(Enum):
    LEFT = 1
    MID = 2
    RIGHT = 3


class Type(Enum):
    OBSERVED = "OBSERVED"
    EXPLORED = "EXPLORED"
    DEAD_END = "DEAD_END"
    FINAL = "FINAL"


class RedisCOMMAND(str, Enum):
    RUN = 'MOTORS_RUN',
    STOP = 'MOTORS_STOP',
    ROTATEL = 'MOTORS_ROTATEL',
    ROTATER = 'MOTORS_ROTATER',

    LEDEMIT = 'LEDEMIT',
    LEDINTERRUPT = 'LEDINT',

    BZZEMIT = 'BZZEMIT',
    BZZINTERRUPT = 'BZZINT'



class RedisKEYS(str, Enum):
    LED = local_CFG_parser()["LED"],
    BUZZER = local_CFG_parser()["BUZZER"],
    INFRARED = local_CFG_parser()["INFRARED"],
    ULTRASONIC = local_CFG_parser()["ULTRASONIC"],
    MPU = local_CFG_parser()["MPU"],
    MOTORS = local_CFG_parser()["MOTORS"]


class RedisTOPICS(str, Enum):
    BODY_TOPIC = local_CFG_parser()["BODY_TOPIC"],
    CTRL_TOPIC = local_CFG_parser()["CTRL_TOPIC"]

class RedisCONNECTION(str, Enum):
    HOST = local_CFG_parser()["HOST"],
    PORT = local_CFG_parser()["PORT"]
