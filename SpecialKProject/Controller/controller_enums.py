from enum import Enum


class Key(Enum):
    USONIC = 'US'
    IRED = 'IR'
    POS = 'OR'
    MOTOR = 'MT'
    BUZZER = 'BU'


class Topic(Enum):
    BODY = 'BODY'
    CONTROLLER = 'CTRL'


class Compass(Enum):
    NORD = 'N'
    EST = 'E'
    OVEST = 'O'
    SUD = 'S'


class Action(Enum):
    GO_FORWARD = 1
    GO_BACKWARD = 2
    ROTATE_LEFT = 3
    ROTATE_RIGHT = 4
    STOP = 5
