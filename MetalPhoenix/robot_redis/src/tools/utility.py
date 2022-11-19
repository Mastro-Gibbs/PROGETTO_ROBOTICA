from io import StringIO
import os
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

    def string_to_compass(cmp: str):
        if 'NORD' == cmp or 'N' == cmp:
            return Compass.NORD
        elif 'SUD' == cmp or 'S' == cmp:
            return Compass.SUD
        elif 'EST' == cmp or 'E' == cmp:
            return Compass.EST
        elif 'OVEST' == cmp or 'O' == cmp:
            return Compass.OVEST

    def compass_to_string(cmp):
        if cmp == Compass.NORD:
            return "NORD"
        if cmp == Compass.OVEST:
            return "OVEST"
        if cmp == Compass.EST:
            return "EST"
        if cmp == Compass.SUD:
            return "SUD"

    def compass_list_to_string_comma_sep(compass_list):
        string_list = Compass.compass_to_string(compass_list[0]) + ", " +\
            Compass.compass_to_string(compass_list[1]) + ", " + \
            Compass.compass_to_string(compass_list[2]) + ", " + \
            Compass.compass_to_string(compass_list[3])
        return string_list

    def compass_list_to_concat_string(compass_list: list) -> str:
        compass_string = ""
        for cmp in compass_list:
            if cmp == Compass.NORD:
                compass_string += "N"
            if cmp == Compass.SUD:
                compass_string += "S"
            if cmp == Compass.EST:
                compass_string += "E"
            if cmp == Compass.OVEST:
                compass_string += "O"
        return compass_string


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
    def robot_conf_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('../resources/data/config.conf')

        priority_list = psr["ROBOT"]["priority_list"]
        priority_list = priority_list.split(', ')

        i = 0
        for elem in priority_list:
            priority_list[i] = Compass.string_to_compass(elem)
            i += 1

        return {
            "SPEED": float(psr["ROBOT"]["speed"]),
            "ROT_SPEED": float(psr["ROBOT"]["rot_speed"]),
            "SAFE_DIST": float(psr["ROBOT"]["safe_dist"]),
            "MAX_ATTEMPTS": int(psr["ROBOT"]["max_attempts"]),
            "PRIORITY_LIST": priority_list,
            "INTELLIGENCE": psr["ROBOT"]["intelligence"]
        }

    @staticmethod
    def read_data_analysis():
        ...

    @staticmethod
    def write_data_analysis(maze_name, time_to_solve, tree_dict,
                            number_of_nodes, number_of_dead_end, performed_commands,
                            trajectory, intelligence, priority_list):
        config = configparser.ConfigParser()
        path = "../resources/data/"
        file_name = "data_analysis.conf"
        conf_file = path + file_name

        # Verifica se data_analysis.conf esiste altrimenti lo crea
        if os.path.isfile(conf_file):
            print(f"\nFile {file_name} loaded")
            config.read(conf_file)
        else:
            with open(conf_file, "w") as configfile:
                config.write(configfile)

        new_section_name_found = False
        trial = 0
        while not new_section_name_found:
            section_name = maze_name + "_" + str(trial)
            if section_name in config:
                trial += 1
            else:
                new_section_name_found = True

        config.add_section(section_name)
        config[section_name] = {
            "time_to_solve_sec": time_to_solve,
            "intelligence": intelligence,
            "priority_list": priority_list,
            "number_of_nodes": number_of_nodes,
            "number_of_dead_end": number_of_dead_end,
            "tree_dict": tree_dict,
            "performed_commands": performed_commands,
            "trajectory": trajectory
            }

        """
            print("\nSezioni del file di config:")
            print(config.sections())
        """

        with open(conf_file, "w") as configfile:
            config.write(configfile)

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

    @staticmethod
    def redis_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('../resources/data/config.conf')
        return {
            "HOST": psr["REDIS"]["host"],
            "PORT": psr["REDIS"]["port"],
            "B_TOPIC": psr["REDIS"]["b_topic"],
            "C_TOPIC": psr["REDIS"]["c_topic"],
            "SENSORS_KEY": psr["REDIS"]["sensors_k"],
            "MOTORS_KEY": psr["REDIS"]["motors_k"]
        }
