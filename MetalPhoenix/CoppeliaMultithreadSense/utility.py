from io import StringIO


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


def detect_target(begin: float) -> float:
    """
    Detect nearest angle [0, 90, -90, 180] from 'begin' aka current angle.

    #PARAM: -> begin: float. Current angle.

    #RETURN: float. Nearest angle.
    """
    if -45.0 < begin <= 45.0:
        target = 0.0
    elif 45.0 < begin <= 135.0:
        target = 90.0
    elif 135.0 < begin <= 180 or -180 <= begin <= -135.0:
        target = 180.0
    else:
        target = -90.0

    return target


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
            self.__class = "\033[96m[{0}]\033[00m" .format(class_name)

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
