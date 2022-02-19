from io import StringIO
from enum import Enum


class Compass(float, Enum):
    NORD = 90.0
    SUD = -90.0
    EST = 0.0
    OVEST = 180.0


def normalize_angle(ang: float, type_t: int):
    """
    Normalizes any angle in degrees to be in the interval [0.,360.) or
    [-180.,180.).
    """
    bang = ang
    if type_t == 0:
        while bang < 0.0:
            bang = bang + 360.0
        while bang >= 360.0:
            bang = bang - 360.0
    else:
        while bang < -180.0:
            bang = bang + 360.0
        while bang >= 180.0:
            bang = bang - 360.0
    return bang


def short_way(init_g: float, final_g: float) -> bool:
    """
    Calculate shorted path from param 'init_g' aka current orientation
    to 'final_g' aka target orientation.

    #PARAMS -> init_g: float.  Current angle.
            -> final_g: float. Target angle to reach.

    #RETURN: bool. True mean rotate to right, False mean rotate to left.
    """
    init_g_360 = normalize_angle(init_g, 0)
    final_g_360 = normalize_angle(final_g, 0)
    # diff
    first_angle = final_g_360 - init_g_360
    # smallest
    second_angle = -1 * first_angle / abs(first_angle) * (360 - abs(first_angle))
    smallest = first_angle

    if abs(first_angle) > 180:
        smallest = second_angle

    if smallest < 0:
        return True  # to right
    else:
        return False  # to left


def detect_target(begin: float) -> float | None:
    """
    Detect nearest angle [0, 90, -90, 180] from 'begin' aka current angle.

    #WARNING: May not locate the correct angle.

    #PARAM: -> begin: float. Current angle.

    #RETURN: float. Nearest angle.
    """
    if begin is None:
        return None

    if -45.0 < begin <= 45.0:
        target = Compass.EST
    elif 45.0 < begin <= 135.0:
        target = Compass.NORD
    elif 135.0 < begin <= 180 or -180 <= begin <= -135.0:
        target = Compass.OVEST
    else:
        target = Compass.SUD

    return target


def normalize_compass(curr_pos: float, compass: Compass) -> float:
    if detect_target(curr_pos) == 0:
        if compass == Compass.EST:
            return Compass.SUD
        elif compass == Compass.OVEST:
            return Compass.NORD
    elif detect_target(curr_pos) == 90:
        return compass
    elif detect_target(curr_pos) == -90:
        if compass == Compass.EST:
            return Compass.OVEST
        elif compass == Compass.OVEST:
            return Compass.EST
    elif detect_target(curr_pos) == 180:
        if compass == Compass.EST:
            return Compass.NORD
        elif compass == Compass.OVEST:
            return Compass.SUD


class StringBuilder:
    """C++ style StringStream class."""
    _file_str = None

    def __init__(self):
        """Constructor"""
        self._file_str = StringIO()

    def concat(self, string: str, end: str = ''):
        """Build string (append param: string)
        #PARAMS: -> string: str. Entity to append.
                 -> end: str. End char or string.

        #WARNING: to set '\n' must pass it with 'end' param.
        """
        self._file_str.write(string)
        self._file_str.write(end)

    def erase(self):
        """#WARNING: Must be called to destroy previous built string

        Erase current StringBuilder buffer.
        """
        del self._file_str
        self._file_str = StringIO()

    def __str__(self):
        return self._file_str.getvalue()


class StdoutLogger:
    """
    Class to menage stdout log colors.
    """

    def __init__(self, class_name: str, color: str):
        """Constructor

        #PARAMS -> class_name: str. Name of the class.
                -> color: str. Color to set to the class name.

        [AVAILABLE COLORS] = {purple, cyan}
        """
        if color == "purple":
            self.__class = "\033[95m[{0}]\033[00m".format(class_name)
        elif color == "cyan":
            self.__class = "\033[96m[{0}]\033[00m".format(class_name)

    def log(self, msg, severity: int = 0, italic: bool = False):
        """Print on stdout he message with selected color.

        #PARAMS: -> msg: any. Message to print.
                 -> severity: int. [-1 to 4] refer color.
                 -> italic: bool. Italic font
        """
        out: str = str()

        if italic:
            out = "\033[03m"

        if severity == 4:
            out = out + "\033[31m{0}\033[00m".format(msg)  # dk red
        elif severity == 3:
            out = out + "\033[91m{0}\033[00m".format(msg)  # red
        elif severity == 2:
            out = out + "\033[93m{0}\033[00m".format(msg)  # yellow
        elif severity == 1:
            out = out + "\033[32m{0}\033[00m".format(msg)  # dk green
        elif severity == 0:
            out = out + "\033[92m{0}\033[00m".format(msg)  # green
        else:
            out = out + "\033[37m{0}\033[00m".format(msg)  # lite gray

        print(self.__class, end=' \033[97m---> \033[00m')
        print(out)


class Stack:
    """Stack class based on list"""

    def __init__(self):
        self.stack = list()
        self.index = -1

    def push(self, elem: float):
        self.stack.append(elem)
        self.index += 1

    def pop(self) -> float:
        if not self.is_empty():
            self.index -= 1
            return self.stack.pop()
        else:
            raise IndexError("Stack is empty!")

    def peek(self) -> float:
        elem = self.pop()
        self.push(elem)
        return elem

    def is_empty(self) -> bool:
        if self.index == -1:
            return True
        return False

    def erase(self):
        self.stack.clear()
        self.index = -1
