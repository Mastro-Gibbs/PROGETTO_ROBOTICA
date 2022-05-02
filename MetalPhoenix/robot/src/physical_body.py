from math import pi

from tools.coppeliaAPI.simConst import *
import tools.coppeliaAPI.sim as sim
from tools.coppelia import *
from tools.utility import Logger, CFG


LOGSEVERITY = CFG.logger_data()["SEVERITY"]


class PhysicalBody:

    def __init__(self):
        """Constructor for PhysicalBody

        -Start connection with coppelia
        -Init sensors

        Can catch Coppelia and generals exceptions,
        if connection was made, it will be destroyed
        """
        print()  # \n
        self.__class_logger = Logger(class_name="PhysicalBody", color="purple")
        self.__class_logger.set_logfile(CFG.logger_data()["BLOGFILE"])
        self.__class_logger.log(f"LOG SEVERYTY: {str.upper(LOGSEVERITY)}\n", color="yellow")
        self.__class_logger.log("PHYSICAL BODY LAUNCHED", color="dkgreen", italic=True)

        self.__sim = SimConnection(ip=CFG.physical_data()["IP"], port=CFG.physical_data()["PORT"])

        try:
            self.__sim.begin_connection()
            self.__sim.start_simulation()

            self.__class_logger.log("COPPELIA CONNECTION INITIALIZED", italic=True)

            # SENSORS
            self.__cam_handler = self.__sim.get_object_handle("/Freenove4wd/robot_cam")
            sim.simxReadVisionSensor(self.__sim.id(), self.__cam_handler, simx_opmode_oneshot_wait)

            self.__front_prox_handler = self.__sim.get_object_handle("/Freenove4wd/fps")
            self.__back_prox_handler = self.__sim.get_object_handle("/Freenove4wd/bps")
            self.__left_prox_handler = self.__sim.get_object_handle("/Freenove4wd/lps")
            self.__right_prox_handler = self.__sim.get_object_handle("/Freenove4wd/rps")
            self.__gate_handler = self.__sim.get_object_handle("/Maze/GateCounter55cmX40cm/Sensor")
            sim.simxReadProximitySensor(self.__sim.id(), self.__front_prox_handler, simx_opmode_oneshot_wait)
            sim.simxReadProximitySensor(self.__sim.id(), self.__back_prox_handler, simx_opmode_oneshot_wait)
            sim.simxReadProximitySensor(self.__sim.id(), self.__left_prox_handler, simx_opmode_oneshot_wait)
            sim.simxReadProximitySensor(self.__sim.id(), self.__right_prox_handler, simx_opmode_oneshot_wait)
            sim.simxReadProximitySensor(self.__sim.id(), self.__gate_handler, simx_opmode_oneshot_wait)

            # MOTORS
            self.__fl_motor_handler = self.__sim.get_object_handle("/Freenove4wd/joint_front_left_wheel")
            self.__fr_motor_handler = self.__sim.get_object_handle("/Freenove4wd/joint_front_right_wheel")
            self.__rl_motor_handler = self.__sim.get_object_handle("/Freenove4wd/joint_rear_left_wheel")
            self.__rr_motor_handler = self.__sim.get_object_handle("/Freenove4wd/joint_rear_right_wheel")

            # READING INITIAL ORIENTATION OF THE ROBOT
            self.__robot_handler = self.__sim.get_object_handle("/Freenove4wd")
            self.__parent_handler = sim_handle_parent
            sim.simxGetObjectOrientation(self.__sim.id(), self.__robot_handler, self.__parent_handler,
                                         simx_opmode_oneshot_wait)

        except SimConnectionException as sce:
            self.__class_logger.log("[ERR] -> {0}\n".format(sce), "red", True, True)
            exit(-1)

        except SimHandleException as she:
            self.__class_logger.log("[ERR] -> {0}\n".format(she), "red", True, True)
            self.__sim.stop_simulation()
            self.__sim.end_connection()
            exit(-1)

        except Exception as e:
            self.__class_logger.log("Something went wrong:\n[ERR] -> {0}".format(e), "red", True, True)
            self.__sim.stop_simulation()
            self.__sim.end_connection()
            exit(-1)

    def __del__(self):
        """Destroyer

        -Log on stdout
        -Close coppelia connection (if it will be called, the connection to coppelia exists)
        """

        self.__sim.stop_simulation()
        self.__sim.end_connection()

    def virtual_destructor(self):
        self.__class_logger.log("COPPELIA CONNECTION STOPPED", "yellow", italic=True)
        self.__class_logger.log("PHYSICAL BODY STOPPED", "yellow", italic=True)

    def move_forward(self, vel) -> None:
        """Move forward robot wheels
        #PARAM:
            -vel -> velocity to set
        """
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log(f"Moving forward, vel = {vel}", color="gray")

        self.__set_motor_velocity(vel, vel, vel, vel)

    def move_backward(self, vel) -> None:
        """Move backward robot wheels
        #PARAM:
            -vel -> velocity to set
        """
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log(f"Moving backward, vel = {vel}", color="gray")

        self.__set_motor_velocity(-vel, -vel, -vel, -vel)

    def turn(self, vel_rail_l, vel_rail_r) -> None:
        """Rotate robot.
        #PARAM:
            -vel_rail_l -> velocity to set at left rail
            -vel_rail_r -> velocity to set at right rail
        """
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log(f"Turning: vel_rail_l = {vel_rail_l}, vel_rail_r = {vel_rail_r}", color="gray")

        self.__set_motor_velocity(vel_rail_l, vel_rail_r, vel_rail_l, vel_rail_r)

    def stop(self) -> None:
        """Stop robot wheels"""
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log("Stopping", color="gray")

        self.__set_motor_velocity(0, 0, 0, 0)

    def __set_motor_velocity(self, vel_FL, vel_FR, vel_RL, vel_RR) -> None:
        """Call simx API to set wheels speed
        #PARAM:
            -vel_FL -> Left upper wheel
            -vel_FR -> right upper wheel
            -vel_RL -> Left lower wheel
            -vel_RR -> right lower wheel
        """
        _id = self.__sim.id()
        _op_mode = simx_opmode_oneshot

        sim.simxSetJointTargetVelocity(_id, self.__rl_motor_handler, vel_RL, _op_mode)
        sim.simxSetJointTargetVelocity(_id, self.__fr_motor_handler, vel_FR, _op_mode)
        sim.simxSetJointTargetVelocity(_id, self.__rr_motor_handler, vel_RR, _op_mode)
        sim.simxSetJointTargetVelocity(_id, self.__fl_motor_handler, vel_FL, _op_mode)

    def get_proxF(self) -> float | None:
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log("Getting frontal proximity", color="gray")

        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__front_prox_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return _val if 0.03 <= _val <= 0.40 else None

    def get_proxL(self) -> float | None:
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log("Getting left proximity", color="gray")

        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__left_prox_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return _val if 0.03 <= _val <= 0.40 else None

    def get_proxR(self) -> float | None:
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log("Getting right proximity", color="gray")

        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__right_prox_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return _val if 0.03 <= _val <= 0.40 else None

    def get_proxB(self) -> float | None:
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log("Getting back proximity", color="gray")

        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__back_prox_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return _val if 0.03 <= _val <= 0.40 else None

    def get_gate(self) -> bool:
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log("Reading gate", color="gray")

        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__gate_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return True if 0.25 < _val <= 0.30 else False

    def get_orientation(self) -> float:
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log("Getting car orientation", color="gray")

        return sim.simxGetObjectOrientation(self.__sim.id(), self.__robot_handler, self.__parent_handler,
                                            simx_opmode_oneshot_wait)[1][2]

    def get_orientation_deg(self) -> float:
        return self.get_orientation() * 180 / pi

    @staticmethod
    def update_cfg():
        global LOGSEVERITY

        LOGSEVERITY = CFG.logger_data()["SEVERITY"]
