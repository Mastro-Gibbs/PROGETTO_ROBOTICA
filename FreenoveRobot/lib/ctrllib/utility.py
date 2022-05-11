import configparser
import datetime
from enums import Compass


date = datetime.datetime.now()
sign = "[" + str(date.year) + "-" + str(date.month) + "-" + str(date.day) + "_" + str(date.hour) \
        + "_" + str(date.minute) + "_" + str(date.second) + "]"


NODE_ID = "n"
NODE_COUNT = 0


def generate_node_id() -> str:
    global NODE_COUNT
    NODE_COUNT += 1
    return NODE_ID + str(NODE_COUNT)


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
def f_r_l_b_to_compass(curr_ori: float) -> dict:
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


class Logger:
    """
    Class to menage stdout log colors && log files.
    """

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
        psr.read('lib/data/config.conf')
        return {
                "SPEED": float(psr["ROBOT"]["speed"]),
                "ROT_SPEED": float(psr["ROBOT"]["rot_speed"]),
                "SAFE_DIST": float(psr["ROBOT"]["safe_dist"]),
                "MAX_ATTEMPTS": int(psr["ROBOT"]["max_attempts"])
                }

    @staticmethod
    def logger_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('lib/data/config.conf')
        return {
                "CLOGFILE": psr["UTILITY"]["controllerlog"],
                "BLOGFILE": psr["UTILITY"]["bodylog"],
                "ALOGFILE": psr["UTILITY"]["agentlog"],
                "EXT": psr["UTILITY"]["ext"],
                "SEVERITY": psr["UTILITY"]["severity"]
                }

    @staticmethod
    def redis_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('lib/data/config.conf')
        return {
            "HOST": psr["REDIS"]["host"],
            "PORT": psr["REDIS"]["port"],
            "B_TOPIC": psr["REDIS"]["b_topic"],
            "C_TOPIC": psr["REDIS"]["c_topic"],
            "SENSORS_KEY": psr["REDIS"]["sensors_k"],
            "MOTORS_KEY": psr["REDIS"]["motors_k"]
        }
