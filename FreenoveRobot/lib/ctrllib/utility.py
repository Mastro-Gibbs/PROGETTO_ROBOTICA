import configparser
import datetime
from lib.ctrllib.enums import Compass, Command, Clockwise, Color, STDOUTDecor


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


def detect_target(begin: float) -> Compass:
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


class Logger:
    """
    Class to menage stdout log colors && log files.
    """

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
            underline: bool = False, blink: bool = False, noheader: bool = False):

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


class CFG:

    def __init__(self):
        ...

    @staticmethod
    def controller_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('data/config.conf')

        pref = psr["ROBOT"]["robot_preference_choice"]
        pref = pref.split(', ')

        i = 0
        for elem in pref:
            pref[i] = Compass.test_and_set(elem)
            i += 1

        return {
                "SPEED": int(psr["ROBOT"]["speed"]),
                "ROT_SPEED": int(psr["ROBOT"]["rot_speed"]),
                "SAFE_DIST": int(psr["ROBOT"]["safe_dist"]),
                "MAX_ATTEMPTS": int(psr["ROBOT"]["max_attempts"]),
                "PREFERENCE": pref
                }

    @staticmethod
    def logger_data() -> dict:
        psr = configparser.ConfigParser()
        psr.read('data/config.conf')
        return {
                "LOGPATH": psr["LOGGER"]["logpath"],
                "EXT": psr["LOGGER"]["ext"]
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
