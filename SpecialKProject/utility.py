from math import pi
from enum import Enum
from io import StringIO

FREENOVE = "Freenove4wd"
ROUND_DIGITS = 4

CRITICAL_SIDE_DISTANCE = 0.070


class Clockwise(Enum):
    RIGHT = 0
    LEFT = 1


def round_v(value):
    return round(value, ROUND_DIGITS)


def radians_to_degrees(rad):
    return rad * 180 / pi


def degrees_to_radians(deg):
    return deg * pi / 180


def normalize_angle(ang, type):
    """
    Normalizes any angle in degrees to be in the interval [0.,360.) or
    [-180.,180.).
    """
    bang = ang
    if type == 0:
        while bang < 0.:
            bang = bang + 360.
        while bang >= 360.:
            bang = bang - 360.
    else:
        while bang < -180.:
            bang = bang + 360.
        while bang >= 180.:
            bang = bang - 360.
    return bang



class StringBuilder:
    _file_str = None

    def __init__(self):
        self._file_str = StringIO()

    def concat(self, str, end=""):
        self._file_str.write(str)
        self._file_str.write(end)

    def erase(self):
        del self._file_str
        self._file_str = StringIO()

    def __str__(self):
        return self._file_str.getvalue()


"""angles = [90, -90, 270, 360, 340, 21, 0, 200, 180, -180]
# angles = [-90]
init_g_180 = -130
init_g_360 = normalize_angle(init_g_180, 0)
print("init_g_360: ", init_g_360)
rot = 90
#print(normalize_angle(init_g_180-rot, 1))
final_g_360 = init_g_360 + rot
print("final_g_360: ", final_g_360)
final_g_180 = normalize_angle(final_g_360, 1)
print(final_g_180)

print(angles)
g = []
for i in angles:
    g.append(normalize_angle(i, 1))
print(g)"""
