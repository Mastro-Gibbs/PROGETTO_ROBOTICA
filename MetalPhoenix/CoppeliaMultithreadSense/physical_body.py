import inspect
import ctypes as ct

from threading import Thread, stack_size
from time import sleep
from enum import Enum
from math import pi

from coppelia import *

from utility import StdoutLogger


class ThreadType(str, Enum):
    """Specify thread type

    return thread name
    """
    th_prox = "prox"
    th_orientation = "orientation"


class PhysicalBody:

    def __init__(self):
        """Constructor for PhysicalBody

        -Start connection with coppelia
        -Init sensors
        -Init threads
        -Init shared vars

        Can catch Coppelia and generals exceptions,
        if connection was made, it will be destroyed
        """
        print()  # \n
        self.__class_logger = StdoutLogger(class_name="PhysicalBody", color="purple")

        self.__sim = SimConnection(ip="127.0.0.1", port=19997)

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
            sim.simxGetObjectOrientation(self.__sim.id(), self.__robot_handler, self.__parent_handler, simx_opmode_oneshot_wait)

            # THREADS
            self.__thread_prox = Thread(target=self.__get_distance, args=(), name="prox")

            self.__thread_orientation = Thread(target=self.__get_orientation, args=(), name="orientation")

            stack_size(524288)  # 512KB stack size

            # SHARED VARS
            self._proxF = None
            self._proxL = None
            self._proxR = None
            self._proxB = None
            self._gate = None

            self._orientation = None

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

    def thread_begin(self, th: ThreadType, sample_delay=0.0) -> None:
        """Start specified thread

        # PARAMS:
            -th -> ThreadType
            -sample_delay -> time to sleep over thread loops

        If the new thread already exists, it will be killed
        """
        sample_delay = abs(sample_delay)
        if th == ThreadType.th_prox:
            if self.__thread_prox.is_alive():
                self.__async_raise(self.__thread_prox, SystemExit)
            self.__thread_prox = Thread(target=self.__get_distance, args=(sample_delay,), name="prox")
            self.__thread_prox.start()
        elif th == ThreadType.th_orientation:
            if self.__thread_orientation.is_alive():
                self.__async_raise(self.__thread_orientation, SystemExit)
            self.__thread_orientation = Thread(target=self.__get_orientation, args=(sample_delay,), name="orientation")
            self.__thread_orientation.start()

        self.__class_logger.log("[Thread] '{0}' reveals itself!".format(th), 1)

    def thread_kill(self, th: ThreadType) -> None:
        """Stop specified thread

        # PARAMS:
            -th -> ThreadType

        If the thread already exists, it will be killed
        else it has no effect
        """
        if th == ThreadType.th_prox:
            if self.__thread_prox.is_alive():
                self.__async_raise(self.__thread_prox, SystemExit)
        elif th == ThreadType.th_orientation:
            if self.__thread_orientation.is_alive():
                self.__async_raise(self.__thread_orientation, SystemExit)

    def safe_exit(self) -> None:
        """Stop all threads, if they are alive

        Must be called to be sure to kill all thread.

        #EXAMPLE: (no kill thread in time)
        try:
            thread_begin(ThreadType.th_prox)
            ...
        except KeyboardInterrupt:
            safe_kill()
        """
        if self.__thread_prox.is_alive():
            self.__sync_raise(self.__thread_prox, SystemExit)
        if self.__thread_orientation.is_alive():
            self.__sync_raise(self.__thread_orientation, SystemExit)

    def __async_raise(self, thread: Thread, exctype) -> None:
        """Raise exc into passed thread, it's asyncr"""
        tid = ct.c_long(thread.ident)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ct.pythonapi.PyThreadState_SetAsyncExc(tid, ct.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            ct.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

        self.__class_logger.log("[Thread] '{0}' is gone!".format(thread.name), 3)

    def __sync_raise(self, thread: Thread, exctype) -> None:
        """Raise exc into passed thread, it's syncr (wait for thread 'is_alive' property becomes False)"""
        tid = ct.c_long(thread.ident)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ct.pythonapi.PyThreadState_SetAsyncExc(tid, ct.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            ct.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")
        while thread.is_alive():
            sleep(0.001)

        self.__class_logger.log("[Thread] '{0}' is gone!".format(thread.name), 3)

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

    def __get_distance(self, sample_delay):
        self._proxF = None
        self._proxR = None
        self._proxL = None
        self._proxB = None
        self._gate = None
        _id = self.__sim.id()
        while True:
            try:
                _, _, point, _, _ = sim.simxReadProximitySensor(_id, self.__front_prox_handler, simx_opmode_oneshot_wait)
                _val = point[2]
                self._proxF = _val if 0.03 <= _val <= 0.40 else None
                del point

                _, _, point, _, _ = sim.simxReadProximitySensor(_id, self.__back_prox_handler, simx_opmode_oneshot_wait)
                _val = point[2]
                self._proxB = _val if 0.03 <= _val <= 0.40 else None
                del point

                _, _, point, _, _ = sim.simxReadProximitySensor(_id, self.__left_prox_handler, simx_opmode_oneshot_wait)
                _val = point[2]
                self._proxL = _val if 0.03 <= _val <= 0.40 else None
                del point

                _, _, point, _, _ = sim.simxReadProximitySensor(_id, self.__right_prox_handler, simx_opmode_oneshot_wait)
                _val = point[2]
                self._proxR = _val if 0.03 <= _val <= 0.40 else None
                del point

                _, _, point, _, _ = sim.simxReadProximitySensor(_id, self.__gate_handler,
                                                                simx_opmode_oneshot_wait)
                _val = point[2]
                self._gate = True if 0.25 < _val <= 0.30 or self._gate else False
                del point
            except Exception as e:
                self.__class_logger.log("[THREAD prox][ERR] --> {0}".format(e), 4)
                break

            if sample_delay > 0:
                sleep(sample_delay)

    def __get_orientation(self, sample_delay):
        self._orientation = None
        _id = self.__sim.id()
        sleep(0.1)
        while True:
            try:
                self._orientation = \
                    sim.simxGetObjectOrientation(_id, self.__robot_handler, self.__parent_handler,
                                                 simx_opmode_oneshot_wait)[1][2]
            except Exception as e:
                self.__class_logger.log("[THREAD orientation][ERR] --> {0}".format(e), 4)
                break
            if sample_delay > 0:
                sleep(sample_delay)

    def get_proxF(self) -> float | None:
        """return last thread's sampled value of proximity sensor in front"""
        return self._proxF

    def get_proxL(self) -> float | None:
        """return last thread's sampled value of proximity sensor in left"""
        return self._proxL

    def get_proxR(self) -> float | None:
        """return last thread's sampled value of proximity sensor in right"""
        return self._proxR

    def get_proxB(self) -> float | None:
        """return last thread's sampled value of proximity sensor in back"""
        return self._proxB

    def get_gate(self) -> bool:
        """return last thread's sampled value of proximity sensor in back"""
        return self._gate

    def get_orientation(self) -> float | None:
        """return last thread's sampled value of robot Z axis (radiant)"""
        return self._orientation

    def get_orientation_deg(self) -> float | None:
        """return last thread's sampled value of robot Z axis (degrees)"""
        if self._orientation is not None:
            ret = self._orientation * 180 / pi
            return ret if ret > 0.3 or ret < -0.3 else None
        else:
            return None
