from math import pi

from robot.src.utility.coppelia import *
from robot.src.utility.utility import StdoutLogger, CFG


class PhysicalBody:

    def __init__(self):
        """Constructor for PhysicalBody

        -Start connection with coppelia
        -Init sensors

        Can catch Coppelia and generals exceptions,
        if connection was made, it will be destroyed
        """
        print()  # \n
        self.__class_logger = StdoutLogger(class_name="PhysicalBody", color="purple")

        self.__sim = SimConnection(ip=CFG.physical_data()["IP"], port=CFG.physical_data()["PORT"])

        try:
            self.__sim.begin_connection()
            self.__sim.start_simulation()

            self.__class_logger.log("Coppelia connection started!", italic=True)

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
            self.__class_logger.log("[ERR] -> {0}".format(sce), 4)
            exit(-1)

        except SimHandleException as she:
            self.__class_logger.log("[ERR] -> {0}".format(she), 4)
            self.__sim.stop_simulation()
            self.__sim.end_connection()
            exit(-1)

        except Exception as e:
            self.__class_logger.log("Something went wrong:\n[ERR] -> {0}".format(e), 4)
            self.__sim.stop_simulation()
            self.__sim.end_connection()
            exit(-1)

    def __del__(self):
        """Destroyer

        -Log on stdout
        -Close coppelia connection (if it will be called, the connection to coppelia exists)
        """
        self.__class_logger.log("Coppelia connection stopped!\n", severity=4, italic=True)
        self.__sim.stop_simulation()
        self.__sim.end_connection()

    def move_forward(self, vel) -> None:
        """Move forward robot wheels
        #PARAM:
            -vel -> velocity to set
        """
        self.__set_motor_velocity(vel, vel, vel, vel)

    def move_backward(self, vel) -> None:
        """Move backward robot wheels
        #PARAM:
            -vel -> velocity to set
        """
        self.__set_motor_velocity(-vel, -vel, -vel, -vel)

    def turn(self, vel_rail_l, vel_rail_r) -> None:
        """Rotate robot.
        #PARAM:
            -vel_rail_l -> velocity to set at left rail
            -vel_rail_r -> velocity to set at right rail
        """
        self.__set_motor_velocity(vel_rail_l, vel_rail_r, vel_rail_l, vel_rail_r)

    def stop(self) -> None:
        """Stop robot wheels"""
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
        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__front_prox_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return _val if 0.03 <= _val <= 0.40 else None

    def get_proxL(self) -> float | None:
        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__left_prox_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return _val if 0.03 <= _val <= 0.40 else None

    def get_proxR(self) -> float | None:
        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__right_prox_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return _val if 0.03 <= _val <= 0.40 else None

    def get_proxB(self) -> float | None:
        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__back_prox_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return _val if 0.03 <= _val <= 0.40 else None

    def get_gate(self) -> bool:
        _, _, point, _, _ = sim.simxReadProximitySensor(self.__sim.id(), self.__gate_handler,
                                                        simx_opmode_oneshot_wait)
        _val = point[2]
        return True if 0.25 < _val <= 0.30 else False

    def get_orientation(self) -> float:
        return sim.simxGetObjectOrientation(self.__sim.id(), self.__robot_handler, self.__parent_handler,
                                            simx_opmode_oneshot_wait)[1][2]

    def get_orientation_deg(self) -> float:
        return self.get_orientation() * 180 / pi
