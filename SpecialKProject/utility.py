from math import pi
from enum import Enum

FREENOVE = "Freenove4wd"


class Clockwise(Enum):
    RIGHT = 0
    LEFT = 1

def radians_to_degree(rad):
    return rad * 180 / pi


def degree_to_radians(deg):
    return deg * pi / 180
