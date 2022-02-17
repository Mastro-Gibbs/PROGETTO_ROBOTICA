import inspect
import ctypes as ct

from threading import Thread, stack_size
from time import sleep
from enum import Enum
from math import pi

from pycsim import CSim, common
import RemoteApiPython.sim as sim

from utility import StdoutLogger


class ThreadType(str, Enum):
    """Specify thread type

    return thread name
    """
    th_proxF = "proxF"
    th_proxL = "proxL"
    th_proxR = "proxR"
    th_proxB = "proxB"
    th_visL = "visL"
    th_visC = "visC"
    th_visR = "visR"
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

        self.__api = CSim.connect("127.0.0.1", 19997)
        self.__api.simulation.start()

        self.__class_logger.log("Coppelia connection started!", italic=True)
        try:
            self.__clientID = self.__api._id

            # SENSORS
            self.__front_camera = self.__api.sensor.vision("cam1")
            self.__centre_vision_sensor = self.__api.sensor.vision("cvs")
            self.__right_vision_sensor = self.__api.sensor.vision("rvs")
            self.__left_vision_sensor = self.__api.sensor.vision("lvs")
            self.__front_proximity_sensor = self.__api.sensor.proximity('fps')
            self.__back_proximity_sensor = self.__api.sensor.proximity('bps')
            self.__left_proximity_sensor = self.__api.sensor.proximity('lps')
            self.__right_proximity_sensor = self.__api.sensor.proximity('rps')

            # IR SENSOR HANDLERS
            self.__lvs_handle = sim.simxGetObjectHandle(self.__clientID, 'lvs', sim.simx_opmode_blocking)[1]
            sim.simxReadVisionSensor(self.__clientID, self.__lvs_handle, sim.simx_opmode_streaming)
            self.__cvs_handle = sim.simxGetObjectHandle(self.__clientID, 'cvs', sim.simx_opmode_blocking)[1]
            sim.simxReadVisionSensor(self.__clientID, self.__cvs_handle, sim.simx_opmode_streaming)
            self.__rvs_handle = sim.simxGetObjectHandle(self.__clientID, 'rvs', sim.simx_opmode_blocking)[1]
            sim.simxReadVisionSensor(self.__clientID, self.__rvs_handle, sim.simx_opmode_streaming)

            # MOTORS
            self.__front_left_motor = self.__api.joint.with_velocity_control("joint_front_left_wheel")
            self.__front_right_motor = self.__api.joint.with_velocity_control("joint_front_right_wheel")
            self.__rear_left_motor = self.__api.joint.with_velocity_control("joint_rear_left_wheel")
            self.__rear_right_motor = self.__api.joint.with_velocity_control("joint_rear_right_wheel")

            # MOTOR HANDLERS
            self.__flmotor_handle = \
                sim.simxGetObjectHandle(self.__clientID, 'joint_front_left_wheel', sim.simx_opmode_blocking)[1]
            self.__frmotor_handle = \
                sim.simxGetObjectHandle(self.__clientID, 'joint_front_right_wheel', sim.simx_opmode_blocking)[1]
            self.__rlmotor_handle = \
                sim.simxGetObjectHandle(self.__clientID, 'joint_rear_left_wheel', sim.simx_opmode_blocking)[1]
            self.__rrmotor_handle = \
                sim.simxGetObjectHandle(self.__clientID, 'joint_rear_right_wheel', sim.simx_opmode_blocking)[1]

            # READING INITIAL ORIENTATION OF THE ROBOT
            self.__code_robot, self.__handle_robot = sim.simxGetObjectHandle(self.__clientID, "Freenove4wd",
                                                                             sim.simx_opmode_blocking)
            self.__handle_parent = sim.sim_handle_parent

            sim.simxGetObjectOrientation(self.__code_robot, self.__handle_robot, self.__handle_parent,
                                         sim.simx_opmode_streaming)

            # THREADS
            self.__thread_proxF = Thread(target=self.__get_front_distance, args=(), name="proxF")
            self.__thread_proxL = Thread(target=self.__get_left_distance, args=(), name="proxL")
            self.__thread_proxR = Thread(target=self.__get_right_distance, args=(), name="proxR")
            self.__thread_proxB = Thread(target=self.__get_back_distance, args=(), name="proxB")

            self.__thread_visL = Thread(target=self.__black_color_detected_left, args=(), name="visL")
            self.__thread_visC = Thread(target=self.__black_color_detected_centre, args=(), name="visC")
            self.__thread_visR = Thread(target=self.__black_color_detected_centre, args=(), name="visR")

            self.__thread_orientation = Thread(target=self.__get_orientation, args=(), name="orientation")

            stack_size(524288)  # 512KB stack size

            # SHARED VARS
            self._proxF = None
            self._proxL = None
            self._proxR = None
            self._proxB = None

            self._visL = None
            self._visC = None
            self._visR = None

            self._orientation = None

        except common.NotFoundComponentError as e:
            self.__class_logger.log("Coppelia Sim Scene Error: missing component:\n[ERR] -> {0}".format(e), 4)
            self.__api.simulation.stop()
            self.__api.close_connection()
            exit(-1)
        except Exception as e:
            self.__class_logger.log("Something went wrong:\n[ERR] -> {0}".format(e), 4)
            self.__api.simulation.stop()
            self.__api.close_connection()
            exit(-1)

    def __del__(self):
        """Destroyer

        -Log on stdout
        -Close coppelia connection (if it will be called, the connection to coppelia exists)
        """
        self.__class_logger.log("Coppelia connection stopped!\n", severity=4, italic=True)
        self.__api.simulation.stop()
        self.__api.close_connection()

    def thread_begin(self, th: ThreadType, sample_delay=0.01) -> None:
        """Start specified thread

        # PARAMS:
            -th -> ThreadType
            -sample_delay -> time to sleep over thread loops

        If the new thread already exists, it will be killed
        """
        sample_delay = abs(sample_delay)
        if th == ThreadType.th_proxF:
            if self.__thread_proxF.is_alive():
                self.__async_raise(self.__thread_proxF, SystemExit)
            self.__thread_proxF = Thread(target=self.__get_front_distance, args=(sample_delay,), name="proxF")
            self.__thread_proxF.start()
        elif th == ThreadType.th_proxL:
            if self.__thread_proxL.is_alive():
                self.__async_raise(self.__thread_proxL, SystemExit)
            self.__thread_proxL = Thread(target=self.__get_left_distance, args=(sample_delay,), name="proxL")
            self.__thread_proxL.start()
        elif th == ThreadType.th_proxR:
            if self.__thread_proxR.is_alive():
                self.__async_raise(self.__thread_proxR, SystemExit)
            self.__thread_proxR = Thread(target=self.__get_right_distance, args=(sample_delay,), name="proxR")
            self.__thread_proxR.start()
        elif th == ThreadType.th_proxB:
            if self.__thread_proxB.is_alive():
                self.__async_raise(self.__thread_proxB, SystemExit)
            self.__thread_proxB = Thread(target=self.__get_back_distance, args=(sample_delay,), name="proxB")
            self.__thread_proxB.start()
        elif th == ThreadType.th_visL:
            if self.__thread_visL.is_alive():
                self.__async_raise(self.__thread_visL, SystemExit)
            self.__thread_visL = Thread(target=self.__black_color_detected_left, args=(sample_delay,), name="visL")
            self.__thread_visL.start()
        elif th == ThreadType.th_visC:
            if self.__thread_visC.is_alive():
                self.__async_raise(self.__thread_visC, SystemExit)
            self.__thread_visC = Thread(target=self.__black_color_detected_centre, args=(sample_delay,), name="visC")
            self.__thread_visC.start()
        elif th == ThreadType.th_visR:
            if self.__thread_visR.is_alive():
                self.__async_raise(self.__thread_visR, SystemExit)
            self.__thread_visR = Thread(target=self.__black_color_detected_right, args=(sample_delay,), name="visR")
            self.__thread_visR.start()
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
        if th == ThreadType.th_proxF:
            if self.__thread_proxF.is_alive():
                self.__async_raise(self.__thread_proxF, SystemExit)
        elif th == ThreadType.th_proxL:
            if self.__thread_proxL.is_alive():
                self.__async_raise(self.__thread_proxL, SystemExit)
        elif th == ThreadType.th_proxR:
            if self.__thread_proxR.is_alive():
                self.__async_raise(self.__thread_proxR, SystemExit)
        elif th == ThreadType.th_proxB:
            if self.__thread_proxB.is_alive():
                self.__async_raise(self.__thread_proxB, SystemExit)
        elif th == ThreadType.th_visL:
            if self.__thread_visL.is_alive():
                self.__async_raise(self.__thread_visL, SystemExit)
        elif th == ThreadType.th_visC:
            if self.__thread_visC.is_alive():
                self.__async_raise(self.__thread_visC, SystemExit)
        elif th == ThreadType.th_visR:
            if self.__thread_visR.is_alive():
                self.__async_raise(self.__thread_visR, SystemExit)
        elif th == ThreadType.th_orientation:
            if self.__thread_orientation.is_alive():
                self.__async_raise(self.__thread_orientation, SystemExit)

        self.__class_logger.log("[Thread] '{0}' is gone!".format(th), 3)

    def safe_exit(self) -> None:
        """Stop all threads, if they are alive

        Must be called to be sure to kill all thread.

        #EXAMPLE: (no kill thread in time)
        try:
            thread_begin(ThreadType.th_proxF)
            ...
        except KeyboardInterrupt:
            safe_kill()
        """
        if self.__thread_proxF.is_alive():
            self.__sync_raise(self.__thread_proxF, SystemExit)
        if self.__thread_proxL.is_alive():
            self.__sync_raise(self.__thread_proxL, SystemExit)
        if self.__thread_proxR.is_alive():
            self.__sync_raise(self.__thread_proxR, SystemExit)
        if self.__thread_proxB.is_alive():
            self.__sync_raise(self.__thread_proxB, SystemExit)
        if self.__thread_visL.is_alive():
            self.__sync_raise(self.__thread_visL, SystemExit)
        if self.__thread_visC.is_alive():
            self.__sync_raise(self.__thread_visC, SystemExit)
        if self.__thread_visR.is_alive():
            self.__sync_raise(self.__thread_visR, SystemExit)
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
        _id = self.__clientID
        _op_mode = sim.simx_opmode_streaming

        sim.simxSetJointTargetVelocity(_id, self.__flmotor_handle, vel_FL, _op_mode)
        sim.simxSetJointTargetVelocity(_id, self.__frmotor_handle, vel_FR, _op_mode)
        sim.simxSetJointTargetVelocity(_id, self.__rlmotor_handle, vel_RL, _op_mode)
        sim.simxSetJointTargetVelocity(_id, self.__rrmotor_handle, vel_RR, _op_mode)

    def __get_front_distance(self, sample_delay):
        self._proxF = None
        while True:
            try:
                _, vec3 = self.__front_proximity_sensor.read()
                self._proxF = vec3.distance()
            except Exception as e:
                self.__class_logger.log("[THREAD proxF][ERR] --> {0}".format(e), 4)
            sleep(sample_delay)

    def __get_back_distance(self, sample_delay):
        self._proxB = None
        while True:
            try:
                _, vec3 = self.__back_proximity_sensor.read()
                self._proxB = vec3.distance()
            except Exception as e:
                self.__class_logger.log("[THREAD proxB][ERR] --> {0}".format(e), 4)
            sleep(sample_delay)

    def __get_left_distance(self, sample_delay):
        self._proxL = None
        while True:
            try:
                _, vec3 = self.__left_proximity_sensor.read()
                self._proxL = vec3.distance()
            except Exception as e:
                self.__class_logger.log("[THREAD proxL][ERR] --> {0}".format(e), 4)
            sleep(sample_delay)

    def __get_right_distance(self, sample_delay):
        self._proxR = None
        while True:
            try:
                _, vec3 = self.__right_proximity_sensor.read()
                self._proxR = vec3.distance()
            except Exception as e:
                self.__class_logger.log("[THREAD proxR][ERR] --> {0}".format(e), 4)
            sleep(sample_delay)

    def __black_color_detected_left(self, sample_delay):
        self._visL = None
        while True:
            try:
                arr = sim.simxReadVisionSensor(self.__clientID, self.__lvs_handle, sim.simx_opmode_buffer)[2]
                if arr:
                    self._visL = arr[0][11] < 0.5
                del arr
            except IndexError:
                self._visL = None
            except Exception as e:
                self.__class_logger.log("[THREAD visL][ERR] --> {0}".format(e), 4)
            sleep(sample_delay + 0.01)

    def __black_color_detected_centre(self, sample_delay):
        self._visC = None
        while True:
            try:
                arr = sim.simxReadVisionSensor(self.__clientID, self.__cvs_handle, sim.simx_opmode_buffer)[2]
                if arr:
                    self._visC = arr[0][11] < 0.5
                del arr
            except IndexError:
                self._visC = None
            except Exception as e:
                self.__class_logger.log("[THREAD visC][ERR] --> {0}".format(e), 4)
            sleep(sample_delay + 0.01)

    def __black_color_detected_right(self, sample_delay):
        self._visR = None
        while True:
            try:
                arr = sim.simxReadVisionSensor(self.__clientID, self.__rvs_handle, sim.simx_opmode_buffer)[2]
                if arr:
                    self._visR = arr[0][11] < 0.5
                del arr
            except IndexError:
                self._visR = None
            except Exception as e:
                self.__class_logger.log("[THREAD visR][ERR] --> {0}".format(e), 4)
            sleep(sample_delay + 0.01)

    def __get_orientation(self, sample_delay):
        self._orientation = None
        sleep(0.1)
        while True:
            try:
                self._orientation = \
                    sim.simxGetObjectOrientation(self.__code_robot, self.__handle_robot, self.__handle_parent,
                                                 sim.simx_opmode_buffer)[1][2]
            except Exception as e:
                self.__class_logger.log("[THREAD orientation][ERR] --> {0}".format(e), 4)
            sleep(sample_delay)

    def get_proxF(self)-> float | None:
        """return last thread's sampled value of proximity sensor in front"""
        return self._proxF if self._proxF is None or self._proxF > 0 else None

    def get_proxL(self) -> float | None:
        """return last thread's sampled value of proximity sensor in left"""
        return self._proxL if self._proxF is None or self._proxL > 0 else None

    def get_proxR(self) -> float | None:
        """return last thread's sampled value of proximity sensor in right"""
        return self._proxR if self._proxF is None or self._proxR > 0 else None

    def get_proxB(self) -> float | None:
        """return last thread's sampled value of proximity sensor in back"""
        return self._proxB if self._proxF is None or self._proxB > 0 else None

    def get_visL(self) -> bool:
        """return last thread's sampled value of vision sensor in left (black line)"""
        return self._visL

    def get_visC(self) -> bool:
        """return last thread's sampled value of vision sensor in centre (black line)"""
        return self._visC

    def get_visR(self) -> bool:
        """return last thread's sampled value of vision sensor in right (black line)"""
        return self._visR

    def get_orientation(self) -> float | None:
        """return last thread's sampled value of robot Z axis (radiant)"""
        return self._orientation

    def get_orientation_deg(self) -> float | None:
        """return last thread's sampled value of robot Z axis (degrees)"""
        if self._orientation is not None:
            return self._orientation * 180 / pi
        else:
            return None
