import os
import json
import datetime
import configparser

from sys import stdout
from enum import Enum

from lib.ctrllib.enums import Command, Color, STDOUTDecor

""" This python file contains all the static methods and classes that are useful for the maze algorithm """

date = datetime.datetime.now()
sign = "[" + str(date.year) + "-" + str(date.month) + "-" + str(date.day) + "_" + str(date.hour) \
       + "_" + str(date.minute) + "_" + str(date.second) + "]"


class Compass(float, Enum):
    NORD = 90.0
    SUD = -90.0
    EST = 0.0
    OVEST = 180.0

    @staticmethod
    def string_to_compass(cmp: str):
        if 'NORD' == cmp or 'N' == cmp:
            return Compass.NORD
        elif 'SUD' == cmp or 'S' == cmp:
            return Compass.SUD
        elif 'EST' == cmp or 'E' == cmp:
            return Compass.EST
        elif 'OVEST' == cmp or 'O' == cmp:
            return Compass.OVEST

    @staticmethod
    def compass_to_string(cmp):
        if cmp == Compass.NORD:
            return "NORD"
        if cmp == Compass.OVEST:
            return "OVEST"
        if cmp == Compass.EST:
            return "EST"
        if cmp == Compass.SUD:
            return "SUD"

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


'''
def decision_making_policy(priority: list, actions: list) -> Compass:
        if not actions:
            return None

        if isinstance(actions[0], Command):
            return actions[0]

        for direction in priority:  # [ S, N, O, E ]
            for action in actions:  # [ E, O, N ]
                if direction == action:
                    return action


def compute_performed_degrees(c, init_g, curr_g):
        """Calculates the angle between init_g and curr_g that the robot performed based on the direction of rotation"""

        if init_g == curr_g:
            return 0

        init_g_360 = normalize_angle(init_g, 0)
        curr_g_360 = normalize_angle(curr_g, 0)

        first_angle = curr_g_360 - init_g_360
        second_angle = -1 * first_angle / \
            abs(first_angle) * (360 - abs(first_angle))

        if c == Clockwise.RIGHT:
            if first_angle < 0:
                performed_degrees = abs(first_angle)
            else:
                performed_degrees = abs(second_angle)
        else:
            if first_angle > 0:
                performed_degrees = abs(first_angle)
            else:
                performed_degrees = abs(second_angle)
        return performed_degrees


def best_angle_and_rotation_way(init_g, final_g):
        """Calculate the best (minimum) angle between init_g and final_g and how you need to rotate"""
        if init_g == final_g:
            return 0

        init_g_360 = normalize_angle(init_g, 0)
        final_g_360 = normalize_angle(final_g, 0)

        first_angle = final_g_360 - init_g_360

        second_angle = -1 * first_angle / \
            abs(first_angle) * (360 - abs(first_angle))
        smallest = first_angle

        if abs(first_angle) > 180:
            smallest = second_angle

        if smallest < 0:
            c = Clockwise.RIGHT
        else:
            c = Clockwise.LEFT

        return smallest, c
'''


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

    def __init__(self, class_name: str, color: Color = Color.GRAY):
        """Constructor

        #PARAMS -> class_name: str. Name of the class.
                -> color: str. Color to set to the class name.

        [AVAILABLE COLORS] => Color enum values
        """
        self.__class = color.value + f"[{class_name}]" + STDOUTDecor.DEFAULT.value

        self.__class_name = class_name

        self.__file: str = None

    def set_logfile(self, path: str):
        self.__file = path + "_" + self.__class_name + sign + "." + CFG.logger_data()["EXT"]

    def log(self, msg, color: Color = Color.GREEN, newline: bool = False, italic: bool = False,
            underline: bool = False, blink: bool = False, noheader: bool = False, _stdout: bool = True):

        """
            Log on file, if it was set; Log on stdout everytime! 
        """

        if self.__file is not None:
            with open(self.__file, "a") as file:
                time = datetime.datetime.now()
                time = "[" + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) + "]"

                if noheader:
                    data_to_write = time + "[NOHEADER] -> " + msg + "\n"
                else:
                    if color == Color.DARKRED:
                        data_to_write = time + " [" + self.__class_name + "]" + "[CRITICAL]"
                    elif color == Color.RED:
                        data_to_write = time + " [" + self.__class_name + "]" + "[ERROR]"
                    elif color == Color.YELLOW:
                        data_to_write = time + " [" + self.__class_name + "]" + "[WARNING]"
                    elif color == Color.DARKGREEN or color == Color.GREEN:
                        data_to_write = time + " [" + self.__class_name + "]" + "[INFO]"
                    elif color == Color.GRAY:
                        data_to_write = time + " [" + self.__class_name + "]" + "[DEBUG]"

                    data_to_write += " -> " + msg + "\n"

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
                print(out)

            else:
                print(self.__class, end=Color.WHITE.value + ' ---> ' + STDOUTDecor.DEFAULT.value)
                print(out)

            stdout.flush()

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
        psr.read('data/config.conf')
        return {
            "HOST": psr["REDIS"]["host"],
            "PORT": psr["REDIS"]["port"],
            "BODY_TOPIC": psr["REDIS"]["body_topic"],
            "CTRL_TOPIC": psr["REDIS"]["ctrl_topic"],
            "LED": psr["REDIS"]["led_key"],
            "BUZZER": psr["REDIS"]["buzzer_key"],
            "INFRARED": psr["REDIS"]["infrared_key"],
            "ULTRASONIC": psr["REDIS"]["ultrasonic_key"],
            "MPU": psr["REDIS"]["mpu_key"],
            "MOTORS": psr["REDIS"]["motors_key"]
        }


__CFG__: dict = CFG.redis_data()


class RedisData:

    class Connection:
        Host = __CFG__['HOST']
        Port = __CFG__["PORT"]

    class Topic:
        Body = __CFG__["BODY_TOPIC"]
        Remote = __CFG__["REMOTE_CONTROLLER_TOPIC"]
        Controller = __CFG__["CTRL_TOPIC"]

    class Key:
        MPU = __CFG__["MPU_KEY"]
        Led = __CFG__["LED_KEY"]
        Motor = __CFG__["MOTORS_KEY"]
        Buzzer = __CFG__["BUZZER_KEY"]
        Infrared = __CFG__["INFRARED_KEY"]
        Ultrasonic = __CFG__["ULTRASONIC_KEY"]

    class Command:
        Led = 'Led'
        Motor = 'Motor'
        Buzzer = 'Buzzer'

    class Value:
        class __Value:
            __status = False

            @classmethod
            @property
            def changed(cls):
                return cls.__status

            @classmethod
            def toggle(cls):
                cls.__status = not cls.__status

            @classmethod
            def set(cls, _b: bool):
                cls.__status = _b

            @classmethod
            def status(cls) -> bool:
                return cls.__status

        class Machine:
            __data: dict = dict()
            __goal = False

            @classmethod
            def on_values(cls, data):
                cls.__data.update(data)

                if int(cls.__data['irL']) and int(cls.__data['irM']) and int(cls.__data['irR']):
                    cls.__goal = True

            @classmethod
            def front(cls):
                return float(cls.__data['proxF']) if cls.__data['proxF'] != 'None' else None

            @classmethod
            def left(cls):
                return float(cls.__data['proxL']) if cls.__data['proxL'] != 'None' else None

            @classmethod
            def right(cls):
                return float(cls.__data['proxR']) if cls.__data['proxR'] != 'None' else None

            @classmethod
            def back(cls):
                return float(cls.__data['proxB']) if cls.__data['proxB'] != 'None' else None

            @classmethod
            def goal(cls):
                return cls.__goal

            @classmethod
            def z_axis(cls):
                return float(cls.__data['Zaxis']) if cls.__data['Zaxis'] != 'None' else None

            @classmethod
            def left_ir(cls):
                return int(cls.__data['irL']) if cls.__data['irL'] != 'None' else None

            @classmethod
            def mid_ir(cls):
                return int(cls.__data['irM']) if cls.__data['irM'] != 'None' else None

            @classmethod
            def right_ir(cls):
                return int(cls.__data['irR']) if cls.__data['irR'] != 'None' else None

        class Motor(__Value):
            __rum = None
            __lum = None
            __rlm = None
            __llm = None

            @classmethod
            def on_values(cls, rum: int, lum: int, rlm: int, llm: int):
                if cls.__rum == rum and cls.__lum == lum and cls.__rlm == rlm and cls.__llm == llm:
                    super().set(False)
                else:
                    cls.__rum = rum
                    cls.__lum = lum
                    cls.__rlm = rlm
                    cls.__llm = llm

                    super().set(True)

            @classmethod
            @property
            def values(cls):
                data = dict()
                data['rum'] = cls.__rum
                data['lum'] = cls.__lum
                data['rlm'] = cls.__rlm
                data['llm'] = cls.__llm

                super().set(False)

                return json.dumps(data, indent=0)

        class Led(__Value):
            pass

        class Buzzer(__Value):
            pass

