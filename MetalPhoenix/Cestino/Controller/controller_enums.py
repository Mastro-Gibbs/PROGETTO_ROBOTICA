from enum import Enum
from math import pi


class Key(str, Enum):
    USONIC = 'US'
    IRED = 'IR'
    POS = 'OR'
    MOTOR = 'MT'
    BUZZER = 'BU'


class Topic(str, Enum):
    BODY = 'BODY'
    CONTROLLER = 'CTRL'


class Compass(str, Enum):
    NORD = 'N'
    EST = 'E'
    OVEST = 'O'
    SUD = 'S'


class Semaphore(float, Enum):
    GREEN = 0.40
    YELLOW = 0.30
    RED = 0.15


class Action(int, Enum):
    GO_FORWARD = 1
    GO_BACKWARD = 2
    ROTATE_LEFT = 3
    ROTATE_RIGHT = 4
    STOP = 5


class MotorsSpeed(Enum):
    LOW = 0.5
    MEDIUM = 45 * pi / 180
    HIGH = 2
    MAX = 5
