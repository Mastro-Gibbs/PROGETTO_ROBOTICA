from enum import Enum

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
