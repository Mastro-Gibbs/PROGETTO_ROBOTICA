from enum import Enum


class Color(str, Enum):
    DARKGREEN = '\033[32m',
    DARKRED   = '\033[31m',
    YELLOW    = '\033[93m',
    PURPLE    = '\033[95m',
    WHITE     = '\033[97m',
    GREEN     = '\033[92m',
    CYAN      = '\033[96m',
    GRAY      = '\033[37m',
    RED       = '\033[91m'


class STDOUTDecor(str, Enum):
    UNDERLINE = '\033[04m',
    SLOWBLINK = '\033[05m',
    FASTBLINK = '\033[06m',
    STOPBLINK = '\033[25m',
    DEFAULT   = '\033[00m',
    ITALIC    = '\033[03m'


class State(Enum):
    STARTING = -1
    STOPPED  = 0
    RUNNING  = 1
    ROTATING = 2
    SENSING  = 3


class Position(Enum):
    UNKNOWN  = 0
    INITIAL  = 1
    CORRIDOR = 2
    JUNCTION = 3
    END      = 4


class Command(Enum):
    START  = -1
    STOP   = 0
    RUN    = 1
    ROTATE = 2
    GO_TO_JUNCTION = 3


class Mode(Enum):
    EXPLORING = 0
    ESCAPING  = 1


