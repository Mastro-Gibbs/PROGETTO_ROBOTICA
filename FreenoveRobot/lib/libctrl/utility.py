import os
import datetime
import configparser
from io import StringIO

from sys import stdout
from enum import Enum

from lib.libctrl.enums import Command, Color, STDOUTDecor

""" This python file contains all the static methods and classes that are useful for the maze algorithm """

date = datetime.datetime.now()
sign = "[" + str(date.year) + "-" + str(date.month) + "-" + str(date.day) + "_" + str(date.hour) \
       + "_" + str(date.minute) + "_" + str(date.second) + "]"


class make:
    @staticmethod
    def tuple(*args) -> tuple:
        return tuple(args)

    @staticmethod
    def dict(*args) -> dict:
        length = len(args)

        if length % 2 != 0:
            length -= 1

        d: dict = dict()
        i = 0
        while length != 0:
            d[args[i]] = args[i+1]
            i += 2
            length -= 2

        return d


class Compass(float, Enum):
    NORTH = 0.0
    SOUTH = 180.0
    EAST  = 90.0
    WEST  = -90.0

    @staticmethod
    def string_to_compass(cmp: str):
        if 'NORTH' == cmp or 'N' == cmp:
            return Compass.NORTH
        elif 'SOUTH' == cmp or 'S' == cmp:
            return Compass.SOUTH
        elif 'EAST' == cmp or 'E' == cmp:
            return Compass.EAST
        elif 'WEST' == cmp or 'W' == cmp:
            return Compass.WEST

    @staticmethod
    def compass_to_string(cmp):
        if cmp == Compass.NORTH:
            return "NORTH"
        if cmp == Compass.WEST:
            return "WEST"
        if cmp == Compass.EAST:
            return "EAST"
        if cmp == Compass.SOUTH:
            return "SOUTH"

    @staticmethod
    def compass_list_to_string_comma_sep(compass_list):
        string_list = Compass.compass_to_string(compass_list[0]) + ", " + \
                      Compass.compass_to_string(compass_list[1]) + ", " + \
                      Compass.compass_to_string(compass_list[2]) + ", " + \
                      Compass.compass_to_string(compass_list[3])
        return string_list

    @staticmethod
    def compass_list_to_concat_string(compass_list: list) -> str:
        compass_string = ""
        for cmp in compass_list:
            if cmp == Compass.NORTH:
                compass_string += "N"
            if cmp == Compass.SOUTH:
                compass_string += "S"
            if cmp == Compass.EAST:
                compass_string += "E"
            if cmp == Compass.WEST:
                compass_string += "W"
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


def detect_target(begin: float) -> Compass:
    """
    Detect nearest angle [0, 90, -90, 180] from 'begin' aka current angle.

    #WARNING: May not locate the correct angle.

    #PARAM: -> begin: float. Current angle.

    #RETURN: float. Nearest angle.
    """

    if -45.0 < begin <= 45.0:
        target = Compass.NORTH
    elif 45.0 < begin <= 135.0:
        target = Compass.EAST
    elif 135.0 < begin <= 180 or -180 <= begin <= -135.0:
        target = Compass.SOUTH
    else:
        target = Compass.WEST

    return target


class FRLB(str, Enum):
    FRONT = 'FRONT'
    LEFT = 'LEFT'
    RIGHT = 'RIGHT'
    BACK = 'BACK'


def f_r_l_b_to_compass(curr_ori: float) -> dict:
    """
    Front Right Left Back to compass.
    It returns a dict according to the robot orientation where:
    key: Front or Right or Left or Back
    value: Est or Sud or Nord or West
    """
    if detect_target(curr_ori) == 0.0:  # EAST front
        return {FRLB.FRONT: Compass.NORTH, FRLB.RIGHT: Compass.EAST, FRLB.LEFT: Compass.WEST, FRLB.BACK: Compass.SOUTH}
    elif detect_target(curr_ori) == 90.0:  # NORTH front
        return {FRLB.FRONT: Compass.EAST, FRLB.RIGHT: Compass.SOUTH, FRLB.LEFT: Compass.NORTH, FRLB.BACK: Compass.WEST}
    elif detect_target(curr_ori) == 180.0:  # WEST front
        return {FRLB.FRONT: Compass.SOUTH, FRLB.RIGHT: Compass.WEST, FRLB.LEFT: Compass.EAST, FRLB.BACK: Compass.NORTH}
    else:  # SOUTH front
        return {FRLB.FRONT: Compass.WEST, FRLB.RIGHT: Compass.NORTH, FRLB.LEFT: Compass.SOUTH, FRLB.BACK: Compass.EAST}


def negate_compass(compass: float) -> Compass:
    """ Given a compass value it returns the negate value """
    if compass == Compass.NORTH:
        return Compass.SOUTH
    elif compass == Compass.SOUTH:
        return Compass.NORTH
    elif compass == Compass.EAST:
        return Compass.WEST
    elif compass == Compass.WEST:
        return Compass.EAST


class Severity(Enum):
    HIGH = 4,
    MID = 3,
    LOW = 2,
    NONE = 1


class Logger:
    """
    Class to menage stdout log colors && log files.
    """

    def __init__(self, class_name: str, severity: Severity, color: Color = Color.GRAY):
        """Constructor

        #PARAMS -> class_name: str. Name of the class.
                -> color: str. Color to set to the class name.

        [AVAILABLE COLORS] => Color enum values
        """
        self.__color = color

        self.__class = color.value + f"[{class_name}]" + STDOUTDecor.DEFAULT.value

        self.__class_name = class_name

        self.__file: str = ''

        self.__severity = severity

        self.__context: str = ''

    def switch_context(self, context) -> None:
        self.__context = self.__class_name
        self.__class_name = context
        self.__class = self.__color.value + f"[{context}]" + STDOUTDecor.DEFAULT.value

    def reset_context(self) -> None:
        self.__class_name = self.__context
        self.__class = self.__color.value + f"[{self.__context}]" + STDOUTDecor.DEFAULT.value

    def set_severity(self, severity: Severity):
        self.__severity = severity

    def set_logfile(self, path: str):
        self.__file = path + "_" + self.__class_name + sign + "." + CFG.logger_data()["EXT"]

    def log(self, msg, color: Color = Color.GREEN, newline: bool = False, italic: bool = False,
            underline: bool = False, blink: bool = False, noheader: bool = False, _stdout: bool = True,
            rewritable: bool = False):

        """
            Log on file, if it was set; Log on stdout everytime! 
        """

        if self.__file is not None:
            with open(self.__file, "a") as file:
                time = datetime.datetime.now()
                time = "[" + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) + "]"

                if noheader:
                    data_to_write = msg + "\n"
                else:
                    data_to_write = time + " [" + self.__class_name + "] -> " + msg + "\n"

                if newline:
                    file.write("\n")
                file.write(data_to_write)

        if _stdout:
            out: str = str()

            if newline:
                print()

            if italic:
                out += STDOUTDecor.ITALIC.value

            if blink:
                out += STDOUTDecor.SLOWBLINK.value

            if underline:
                out += STDOUTDecor.UNDERLINE.value

            out += color.value + f"{msg}" + STDOUTDecor.DEFAULT.value

            if noheader:
                if rewritable:
                    print(out, end='\r')
                else:
                    print(out)

            else:
                if rewritable:
                    print(self.__class + Color.WHITE.value + ' ---> ' + STDOUTDecor.DEFAULT.value + out, end='\r')
                else:
                    print(self.__class, end=Color.WHITE.value + ' ---> ' + STDOUTDecor.DEFAULT.value)
                    print(out)

            stdout.flush()

    def is_loggable(self, comparator: Severity) -> bool:
        if comparator == Severity.NONE:
            return False

        if self.__severity == Severity.HIGH:
            return True
        elif self.__severity == Severity.MID and comparator != Severity.HIGH:
            return True
        elif self.__severity == Severity.LOW and (comparator == Severity.LOW or comparator == Severity.NONE):
            return True

        return False


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

    @property
    def string(self) -> str:
        return self._file_str.getvalue()

    def __str__(self):
        return self._file_str.getvalue()


class CFG:

    def __init__(self):
        ...

    @staticmethod
    def robot_conf_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('data/config.conf')

        priority_list = psr["ROBOT"]["priority_list"]
        priority_list = priority_list.split(', ')

        i = 0
        for elem in priority_list:
            priority_list[i] = Compass.string_to_compass(elem)
            i += 1
        return {
            "SPEED": int(psr["ROBOT"]["speed"]),
            "ROT_SPEED": int(psr["ROBOT"]["rot_speed"]),
            "FRONT_SAFE_DIST": int(psr["ROBOT"]["front_safe_dist"]),
            "SIDE_SAFE_DIST": int(psr["ROBOT"]["side_safe_dist"]),
            "MAX_ATTEMPTS": int(psr["ROBOT"]["max_attempts"]),
            "AUTO_PRIORITY_LIST": int(psr["ROBOT"]["AUTO_PRIORITY_LIST"]),
            "PRIORITY_LIST": priority_list,
            "JUNCTION_TIME": float(psr["ROBOT"]["junction_time"])
        }


    @staticmethod
    def write_data_analysis(maze_name, maze_solved, execution_time, tree_dict,
                            number_of_nodes, number_of_dead_end, performed_commands,
                            trajectory, priority_list):
        config = configparser.ConfigParser()
        path = "data/"
        file_name = "data_analysis.conf"
        conf_file = path + file_name

        # Verifica se data_analysis.conf esiste altrimenti lo crea
        if os.path.isfile(conf_file):
            print(f"\nFile {file_name} loaded")
            config.read(conf_file)
        else:
            with open(conf_file, "w") as configfile:
                config.write(configfile)

        section_name = ""
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
            "maze_solved": maze_solved,
            "execution_time_sec": execution_time,
            "priority_list": priority_list,
            "number_of_nodes": number_of_nodes,
            "number_of_dead_end": number_of_dead_end,
            "tree_dict": tree_dict,
            "performed_commands": performed_commands,
            "trajectory": trajectory
        }

        with open(conf_file, "w") as configfile:
            config.write(configfile)

    @staticmethod
    def physical_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('data/config.conf')
        return {
            "IP": psr["COPPELIA"]["ip"],
            "PORT": int(psr["COPPELIA"]["port"])
        }

    @staticmethod
    def logger_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('data/config.conf')
        return {
            "CLOGFILE": psr["LOGGER"]["controllerlog"],
            "BLOGFILE": psr["LOGGER"]["bodylog"],
            "ALOGFILE": psr["LOGGER"]["agentlog"],
            "EXT": psr["LOGGER"]["ext"],
            "SEVERITY": psr["LOGGER"]["severity"]
        }

    @staticmethod
    def cfg_redis_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('data/config.conf')
        return {
            "HOST": psr["REDIS"]["host"],
            "PORT": psr["REDIS"]["port"],

            "BODY_TOPIC": psr["REDIS"]["body_topic"],
            "CTRL_TOPIC": psr["REDIS"]["ctrl_topic"],
            "REMOTE_CONTROLLER_TOPIC": psr['REDIS']['rc_topic'],

            "SELF_KEY": psr["REDIS"]["self_key"],
            "BTN_KEY": psr["REDIS"]["btn_key"],
            "RC_KEY": psr["REDIS"]["rc_key"],
            "LED_KEY": psr["REDIS"]["led_key"],
            "MPU_KEY": psr["REDIS"]["mpu_key"],
            "MOTORS_KEY": psr["REDIS"]["motors_key"],
            "BUZZER_KEY": psr["REDIS"]["buzzer_key"],
            "INFRARED_KEY": psr["REDIS"]["infrared_key"],
            "ULTRASONIC_KEY": psr["REDIS"]["ultrasonic_key"],

            "RC_ENABLED": bool(int(psr["REDIS"]["rc_enabled"]))
        }

    @staticmethod
    def cfg_sensors() -> dict:
        psr = configparser.ConfigParser()
        psr.read('data/config.conf')
        return {
            "YAW_ENABLED": bool(int(psr["SENSORS"]["yaw_enabled"]))
        }

    @staticmethod
    def maze_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('data/config.conf')
        return {
            "MAZE_NUMBER": psr["MAZE"]["maze_number"]
        }


__REDIS_CFG__: dict = CFG.cfg_redis_data()
__SENSOR_CFG__: dict = CFG.cfg_sensors()
