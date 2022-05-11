from enum import Enum
from utility import CFG


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


class LOGLEVEL(Enum):
    CRITICAL = 5,
    ERROR = 4,
    WARNING = 3,
    INFO = 2,
    DEBUG = 1


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
    LED = CFG.redis_data()["LED"],
    BUZZER = CFG.redis_data()["BUZZER"],
    INFRARED = CFG.redis_data()["INFRARED"],
    ULTRASONIC = CFG.redis_data()["ULTRASONIC"],
    MPU = CFG.redis_data()["MPU"],
    MOTORS = CFG.redis_data()["LMOTORSED"]


class RedisTOPICS(str, Enum):
    BODY_TOPIC = CFG.redis_data()["BODY_TOPIC"],
    CTRL_TOPIC = CFG.redis_data()["CTRL_TOPIC"]

class RedisCONNECTION(str, Enum):
    HOST = CFG.redis_data()["HOST"],
    PORT = CFG.redis_data()["PORT"]
