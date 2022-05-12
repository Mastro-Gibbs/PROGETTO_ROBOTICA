from enum import Enum
import configparser

def lacal_CFG_parser() -> dict:
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


REDISDICT: dict = lacal_CFG_parser()

class Color(str, Enum):
    DARKGREEN = '\033[32m',
    DARKRED = '\033[31m',
    YELLOW = '\033[93m',
    PURPLE = '\033[95m',
    WHITE = '\033[97m',
    GREEN = '\033[92m',
    CYAN = '\033[96m',
    GRAY = '\033[37m',
    RED = '\033[91m'


class STDOUTDecor(str, Enum):
    UNDERLINE = '\033[04m',
    SLOWBLINK = '\033[05m',
    FASTBLINK = '\033[06m',
    STOPBLINK = '\033[25m',
    DEFAULT = '\033[00m',
    ITALIC = '\033[03m'    


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
    LED = REDISDICT["LED"],
    BUZZER = REDISDICT["BUZZER"],
    INFRARED = REDISDICT["INFRARED"],
    ULTRASONIC = REDISDICT["ULTRASONIC"],
    MPU = REDISDICT["MPU"],
    MOTORS = REDISDICT["MOTORS"]


class RedisTOPICS(str, Enum):
    BODY_TOPIC = REDISDICT["BODY_TOPIC"],
    CTRL_TOPIC = REDISDICT["CTRL_TOPIC"]

class RedisCONNECTION(str, Enum):
    HOST = REDISDICT["HOST"],
    PORT = REDISDICT["PORT"]
