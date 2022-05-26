from io import StringIO
from enum import Enum
import configparser
import datetime

""" This python file contains all the static methods and classes that are useful for the maze algorithm """

date = datetime.datetime.now()
sign = "[" + str(date.year) + "-" + str(date.month) + "-" + str(date.day) + "_" + str(date.hour) \
       + "_" + str(date.minute) + "_" + str(date.second) + "]"


class Compass(float, Enum):
    NORD = 90.0
    SUD = -90.0
    EST = 0.0
    OVEST = 180.0

    def test_and_set(cmp: str):

        if 'NORD' == cmp:
            return Compass.NORD
        elif 'SUD' == cmp:
            return Compass.SUD
        elif 'EST' == cmp:
            return Compass.EST
        elif 'OVEST' == cmp:
            return Compass.OVEST


class Clockwise(Enum):
    """ Used to understand how the robot has to rotate: to the right or to the left of the robot """

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


def f_r_l_b_to_compass(curr_ori: float) -> {}:
    """
    Front Right Left Back to compass.
    It returns a dict according to the robot orientation where:
    key: Front or Right or Left or Back
    value: Est or Sud or Nord or West
    """
    if detect_target(curr_ori) == 0:  # EAST front
        return {"FRONT": Compass.EST, "RIGHT": Compass.SUD, "LEFT": Compass.NORD, "BACK": Compass.OVEST}
    elif detect_target(curr_ori) == 90:  # NORD front
        return {"FRONT": Compass.NORD, "RIGHT": Compass.EST, "LEFT": Compass.OVEST, "BACK": Compass.SUD}
    elif detect_target(curr_ori) == 180:  # WEST front
        return {"FRONT": Compass.OVEST, "RIGHT": Compass.NORD, "LEFT": Compass.SUD, "BACK": Compass.EST}
    else:  # SUD front
        return {"FRONT": Compass.SUD, "RIGHT": Compass.OVEST, "LEFT": Compass.EST, "BACK": Compass.NORD}


def negate_compass(compass: float) -> Compass:
    """ Given a compass value it returns the negate value """
    if compass == Compass.NORD:
        return Compass.SUD
    elif compass == Compass.SUD:
        return Compass.NORD
    elif compass == Compass.EST:
        return Compass.OVEST
    elif compass == Compass.OVEST:
        return Compass.EST


class StringBuilder:
    """ C++ style StringStream class. """
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


class Logger:
    """
    Class to menage stdout log colors && log files.
    """

    class LOGLEVEL(Enum):
        CRITICAL = 5,
        ERROR = 4,
        WARNING = 3,
        INFO = 2,
        DEBUG = 1

    def __init__(self, class_name: str, color: str = "gray"):
        """Constructor

        #PARAMS -> class_name: str. Name of the class.
                -> color: str. Color to set to the class name.

        [AVAILABLE COLORS] = {purple, cyan, yellow}
        """
        if color == "purple":
            self.__class = "\033[95m[{0}]\033[00m".format(class_name)
        elif color == "cyan":
            self.__class = "\033[96m[{0}]\033[00m".format(class_name)
        elif color == "yellow":
            self.__class = "\033[93m[{0}]\033[00m".format(class_name)
        else:
            self.__class = "\033[37m[{0}]\033[00m".format(class_name)

        self.__class_name = class_name

        self.__file = None

    def set_logfile(self, path: str):
        self.__file = path + "_" + self.__class_name + sign + "." + CFG.logger_data()["EXT"]

    def log(self, msg, color: str = "green", newline: bool = False, italic: bool = False,
            noheader: bool = False):
        """Print on stdout the message with selected color.

        #PARAMS: -> msg: any. Message to print.
                 -> severity: int. [-1 to 4] refer color.
                 -> italic: bool. Italic font
        """

        if self.__file is not None:
            with open(self.__file, "a") as file:
                time = datetime.datetime.now()
                time = "[" + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) + "]"

                if noheader:
                    data_to_write = time + "[NOHEADER] -> " + msg + "\n"
                else:
                    if color == "dkred":
                        data_to_write = time + " [" + self.__class_name + "]" + "[CRITICAL]"
                    elif color == "red":
                        data_to_write = time + " [" + self.__class_name + "]" + "[ERROR]"
                    elif color == "yellow":
                        data_to_write = time + " [" + self.__class_name + "]" + "[WARNING]"
                    elif color == "yellow+":
                        data_to_write = time + " [" + self.__class_name + "]" + "[WARNING][DEBUG]"
                    elif color == "dkgreen" or color == "green":
                        data_to_write = time + " [" + self.__class_name + "]" + "[INFO]"
                    elif color == "gray":
                        data_to_write = time + " [" + self.__class_name + "]" + "[DEBUG]"

                    data_to_write += " -> " + msg + "\n"

                if newline:
                    file.write("\n")
                file.write(data_to_write)

        out: str = str()

        if italic:
            out = "\033[03m"

        if color == "dkred":
            out = out + "\033[31m{0}\033[00m".format(msg)  # dk red
        elif color == "red":
            out = out + "\033[91m{0}\033[00m".format(msg)  # red
        elif color == "yellow":
            out = out + "\033[93m{0}\033[00m".format(msg)  # yellow
        elif color == "yellow+":
            out = out + "[DEBUG]\033[93m{0}\033[00m".format(msg)  # yellow+
        elif color == "dkgreen":
            out = out + "\033[32m{0}\033[00m".format(msg)  # dk green
        elif color == "green":
            out = out + "\033[92m{0}\033[00m".format(msg)  # green
        elif color == "gray":
            out = out + "\033[37m{0}\033[00m".format(msg)  # lite gray

        if newline:
            print()

        if noheader:
            print(out)
        else:
            print(self.__class, end=' \033[97m---> \033[00m')
            print(out)

    @staticmethod
    def is_loggable(severity: str, comparator: str) -> bool:
        if severity == "none":
            return False

        if severity == "high":
            return True
        elif severity == "mid" and comparator != "high":
            return True
        elif severity == "low" and (comparator == "low" or comparator == "none"):
            return True

        return False


class CFG:

    def __init__(self):
        ...

    @staticmethod
    def controller_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('../resources/data/config.conf')

        pref = psr["ROBOT"]["priority_list"]
        pref = pref.split(', ')

        i = 0
        for elem in pref:
            pref[i] = Compass.test_and_set(elem)
            i += 1

        return {
            "SPEED": float(psr["ROBOT"]["speed"]),
            "ROT_SPEED": float(psr["ROBOT"]["rot_speed"]),
            "SAFE_DIST": float(psr["ROBOT"]["safe_dist"]),
            "MAX_ATTEMPTS": int(psr["ROBOT"]["max_attempts"]),
            "PRIORITY_LIST": pref,
            "INTELLIGENCE": psr["ROBOT"]["intelligence"]
        }


    @staticmethod
    def physical_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('../resources/data/config.conf')
        return {
            "IP": psr["COPPELIA"]["ip"],
            "PORT": int(psr["COPPELIA"]["port"])
        }

    @staticmethod
    def logger_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('../resources/data/config.conf')
        return {
            "CLOGFILE": psr["UTILITY"]["controllerlog"],
            "BLOGFILE": psr["UTILITY"]["bodylog"],
            "ALOGFILE": psr["UTILITY"]["agentlog"],
            "EXT": psr["UTILITY"]["ext"],
            "SEVERITY": psr["UTILITY"]["severity"]
        }
