from io import StringIO
from enum import Enum
import configparser


class Compass(float, Enum):
    NORD = 90.0
    SUD = -90.0
    EST = 0.0
    OVEST = 180.0


class Clockwise(Enum):
    RIGHT = 0
    LEFT = 1


def round_v(value):
    return round(value, 4)


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


def detect_target(begin: float) -> Compass | None:
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


# Front Right Left Back to compass
def f_r_l_b_to_compass(curr_ori: float) -> {}:
    if detect_target(curr_ori) == 0:  # Muso robot ad EST
        return {"FRONT": Compass.EST, "RIGHT": Compass.SUD, "LEFT": Compass.NORD, "BACK": Compass.OVEST}
    elif detect_target(curr_ori) == 90:  # Muso robot a NORD
        return {"FRONT": Compass.NORD, "RIGHT": Compass.EST, "LEFT": Compass.OVEST, "BACK": Compass.SUD}
    elif detect_target(curr_ori) == 180:  # Muso robot ad OVEST
        return {"FRONT": Compass.OVEST, "RIGHT": Compass.NORD, "LEFT": Compass.SUD, "BACK": Compass.EST}
    else:  # -90
        return {"FRONT": Compass.SUD, "RIGHT": Compass.OVEST, "LEFT": Compass.EST, "BACK": Compass.NORD}


def negate_compass(compass: float) -> Compass:
    if compass == Compass.NORD:
        return Compass.SUD
    elif compass == Compass.SUD:
        return Compass.NORD
    elif compass == Compass.EST:
        return Compass.OVEST
    elif compass == Compass.OVEST:
        return Compass.EST


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


class CFG:

    def __init__(self):
        ...

    @staticmethod
    def controller_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('../resources/data/config.conf')
        return {
                "SPEED": float(psr["ROBOT"]["speed"]),
                "ROT_SPEED": float(psr["ROBOT"]["rot_speed"]),
                "SAFE_DIST": float(psr["ROBOT"]["safe_dist"]),
                "MAX_ATTEMPTS": int(psr["ROBOT"]["max_attempts"])
                }

    @staticmethod
    def physical_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('../resources/data/config.conf')
        return {
                "IP": psr["COPPELIA"]["ip"],
                "PORT": int(psr["COPPELIA"]["port"])
                }
