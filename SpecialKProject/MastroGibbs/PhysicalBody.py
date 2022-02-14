from pycsim import CSim, common
import RemoteApiPython.sim as s
from utility import *

from threading import Thread
import inspect
import ctypes
from time import sleep as zzz


class PhysicalBody:

    def __init__(self):
        self.__api = CSim.connect("127.0.0.1", 19997)
        self.__api.simulation.start()
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

            # ACCELEROMETRO
            self.__accelerometer = s.simxGetObjectHandle(self.__clientID, 'Accelerometer_forceSensor',
                                                         s.simx_opmode_blocking)
            self.__mass_object = s.simxGetObjectHandle(self.__clientID, 'Accelerometer_mass', s.simx_opmode_streaming)
            self.__mass = s.simxGetObjectFloatParam(self.__clientID, self.__mass_object[1], s.sim_shapefloatparam_mass,
                                                    s.simx_opmode_streaming)
            s.simxReadForceSensor(self.__clientID, self.__accelerometer[1], s.simx_opmode_streaming)

            # IR SENSORS
            self.__lvs_handle = s.simxGetObjectHandle(self.__clientID, 'lvs', s.simx_opmode_oneshot_wait)[1]
            s.simxReadVisionSensor(self.__clientID, self.__lvs_handle, s.simx_opmode_streaming)
            self.__cvs_handle = s.simxGetObjectHandle(self.__clientID, 'cvs', s.simx_opmode_oneshot_wait)[1]
            s.simxReadVisionSensor(self.__clientID, self.__cvs_handle, s.simx_opmode_streaming)
            self.__rvs_handle = s.simxGetObjectHandle(self.__clientID, 'rvs', s.simx_opmode_oneshot_wait)[1]
            s.simxReadVisionSensor(self.__clientID, self.__rvs_handle, s.simx_opmode_streaming)

            # MOTORS
            self.__front_left_motor = self.__api.joint.with_velocity_control("joint_front_left_wheel")
            self.__front_right_motor = self.__api.joint.with_velocity_control("joint_front_right_wheel")
            self.__rear_left_motor = self.__api.joint.with_velocity_control("joint_rear_left_wheel")
            self.__rear_right_motor = self.__api.joint.with_velocity_control("joint_rear_right_wheel")

            self.__flmotor_handle = \
                s.simxGetObjectHandle(self.__clientID, 'joint_front_left_wheel', s.simx_opmode_oneshot_wait)[1]
            self.__frmotor_handle = \
                s.simxGetObjectHandle(self.__clientID, 'joint_front_right_wheel', s.simx_opmode_oneshot_wait)[1]
            self.__rlmotor_handle = \
                s.simxGetObjectHandle(self.__clientID, 'joint_rear_left_wheel', s.simx_opmode_oneshot_wait)[1]
            self.__rrmotor_handle = \
                s.simxGetObjectHandle(self.__clientID, 'joint_rear_right_wheel', s.simx_opmode_oneshot_wait)[1]

            # READING INITIAL ORIENTATION OF THE ROBOT
            self.__code_robot, self.__handle_robot = s.simxGetObjectHandle(self.__clientID, FREENOVE,
                                                                           s.simx_opmode_oneshot_wait)
            self.__handle_parent = s.sim_handle_parent

            s.simxGetObjectOrientation(self.__code_robot, self.__handle_robot, self.__handle_parent,
                                       s.simx_opmode_streaming)

            # THREADS
            self.__thread_proxF = None
            self.__thread_proxL = None
            self.__thread_proxR = None
            self.__thread_proxB = None

            self.__thread_visL = None
            self.__thread_visC = None
            self.__thread_visR = None

            self.__thread_orientation = None
            self.__thread_orientation_deg = None

            self.__sample_delay = 0.1

            # SHARED VARS
            self._proxF = None
            self._proxL = None
            self._proxR = None
            self._proxB = None

            self._visL = None
            self._visC = None
            self._visR = None

            self._orientation = None
            self._orientation_deg = None

        except common.NotFoundComponentError as e:
            print("[PhysicalBody] Coppelia Sim Scene Error: missing component!")
            print("------------->", e)
            self.__api.simulation.stop()
            self.__api.close_connection()
            exit(-1)
        except Exception:
            print("[PhysicalBody] PhysicalBody ERRORR!")
            self.__api.simulation.stop()
            self.__api.close_connection()
            exit(-1)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.__api.simulation.stop()
        self.__api.close_connection()

    def __del__(self):
        self.__api.simulation.stop()
        self.__api.close_connection()

    def thread_heap_begin(self, sample_delay=0.1):
        self.__sample_delay = abs(sample_delay)
        self.__thread_proxF = Thread(target=self.__get_front_distance, args=(self.__sample_delay,))
        self.__thread_proxL = Thread(target=self.__get_left_distance, args=(self.__sample_delay,))
        self.__thread_proxR = Thread(target=self.__get_right_distance, args=(self.__sample_delay,))
        self.__thread_proxB = Thread(target=self.__get_back_distance, args=(self.__sample_delay,))

        self.__thread_visL = Thread(target=self.__black_color_detected_left, args=(self.__sample_delay,))
        self.__thread_visC = Thread(target=self.__black_color_detected_centre, args=(self.__sample_delay,))
        self.__thread_visR = Thread(target=self.__black_color_detected_right, args=(self.__sample_delay,))

        self.__thread_orientation = Thread(target=self.__get_orientation, args=(self.__sample_delay,))
        self.__thread_orientation_deg = Thread(target=self.__get_orientation_degrees, args=(self.__sample_delay,))

        print("[PhysicalBody] Threads just born!")

    def thread_heap_start(self):
        self.__thread_proxF.start()
        self.__thread_proxL.start()
        self.__thread_proxR.start()
        self.__thread_proxB.start()

        self.__thread_visL.start()
        self.__thread_visC.start()
        self.__thread_visR.start()

        self.__thread_orientation.start()
        self.__thread_orientation_deg.start()

        print("[PhysicalBody] Threads just start!")

    def thread_heap_revive(self):
        revived = False
        if not self.__thread_proxF.is_alive():
            self.__thread_proxF = Thread(target=self.__get_front_distance, args=(self.__sample_delay,))
            self.__thread_proxF.start()
            revived = True
            print("[PhysicalBody] Thread 'proxF' rised!")

        if not self.__thread_proxL.is_alive():
            self.__thread_proxL = Thread(target=self.__get_left_distance, args=(self.__sample_delay,))
            self.__thread_proxL.start()
            revived = True
            print("[PhysicalBody] Thread 'proxL' rised!")

        if not self.__thread_proxR.is_alive():
            self.__thread_proxR = Thread(target=self.__get_right_distance, args=(self.__sample_delay,))
            self.__thread_proxR.start()
            revived = True
            print("[PhysicalBody] Thread 'proxR' rised!")

        if not self.__thread_proxB.is_alive():
            self.__thread_proxB = Thread(target=self.__get_back_distance, args=(self.__sample_delay,))
            self.__thread_proxB.start()
            revived = True
            print("[PhysicalBody] Thread 'proxB' rised!")

        if not self.__thread_visL.is_alive():
            self.__thread_visL = Thread(target=self.__black_color_detected_left, args=(self.__sample_delay,))
            self.__thread_visL.start()
            revived = True
            print("[PhysicalBody] Thread 'visL' rised!")

        if not self.__thread_visC.is_alive():
            self.__thread_visC = Thread(target=self.__black_color_detected_centre, args=(self.__sample_delay,))
            self.__thread_visC.start()
            revived = True
            print("[PhysicalBody] Thread 'visC' rised!")

        if not self.__thread_visR.is_alive():
            self.__thread_visR = Thread(target=self.__black_color_detected_right, args=(self.__sample_delay,))
            self.__thread_visR.start()
            revived = True
            print("[PhysicalBody] Thread 'visR' rised!")

        if not self.__thread_orientation.is_alive():
            self.__thread_orientation = Thread(target=self.__get_orientation, args=(self.__sample_delay,))
            self.__thread_orientation.start()
            revived = True
            print("[PhysicalBody] Thread 'orientation' rised!")

        if not self.__thread_orientation_deg.is_alive():
            self.__thread_orientation_deg = Thread(target=self.__get_orientation_degrees, args=(self.__sample_delay,))
            self.__thread_orientation_deg.start()
            revived = True
            print("[PhysicalBody] Thread 'orientation_deg' rised!")

        if not revived:
            print("[PhysicalBody] No thread rised!")

    def thread_heap_kill(self, th: int):
        killer = Thread(target=self.__suppress_thread, args=(th,))
        killer.start()

    def __suppress_thread(self, th):
        if th == 1:
            self.__async_raise(self.__thread_proxF.ident, SystemExit)
            while self.__thread_proxF.is_alive():
                pass
            print("[PhysicalBody] Thread 'proxF' died!")
        elif th == 2:
            self.__async_raise(self.__thread_proxL.ident, SystemExit)
            while self.__thread_proxL.is_alive():
                pass
            print("[PhysicalBody] Thread 'proxL' died!")
        elif th == 3:
            self.__async_raise(self.__thread_proxR.ident, SystemExit)
            while self.__thread_proxR.is_alive():
                pass
            print("[PhysicalBody] Thread 'proxR' died!")
        elif th == 4:
            self.__async_raise(self.__thread_proxB.ident, SystemExit)
            while self.__thread_proxB.is_alive():
                pass
            print("[PhysicalBody] Thread 'proxB' died!")
        elif th == 5:
            self.__async_raise(self.__thread_visL.ident, SystemExit)
            while self.__thread_visL.is_alive():
                pass
            print("[PhysicalBody] Thread 'visL' died!")
        elif th == 6:
            self.__async_raise(self.__thread_visC.ident, SystemExit)
            while self.__thread_visC.is_alive():
                pass
            print("[PhysicalBody] Thread 'visC' died!")
        elif th == 7:
            self.__async_raise(self.__thread_visR.ident, SystemExit)
            while self.__thread_visR.is_alive():
                pass
            print("[PhysicalBody] Thread 'visR' died!")
        elif th == 8:
            self.__async_raise(self.__thread_orientation.ident, SystemExit)
            while self.__thread_orientation.is_alive():
                pass
            print("[PhysicalBody] Thread 'orientation' died!")
        elif th == 9:
            self.__async_raise(self.__thread_orientation_deg.ident, SystemExit)
            while self.__thread_orientation_deg.is_alive():
                pass
            print("[PhysicalBody] Thread 'orientation_deg' died!")
        else:
            print("[PhysicalBody] No thread found by id!")


    def thread_heap_killall(self):
        if self.__thread_proxF.is_alive():
            self.__async_raise(self.__thread_proxF.ident, SystemExit)
        if self.__thread_proxL.is_alive():
            self.__async_raise(self.__thread_proxL.ident, SystemExit)
        if self.__thread_proxR.is_alive():
            self.__async_raise(self.__thread_proxR.ident, SystemExit)
        if self.__thread_proxB.is_alive():
            self.__async_raise(self.__thread_proxB.ident, SystemExit)

        if self.__thread_visL.is_alive():
            self.__async_raise(self.__thread_visL.ident, SystemExit)
        if self.__thread_visC.is_alive():
            self.__async_raise(self.__thread_visC.ident, SystemExit)
        if self.__thread_visR.is_alive():
            self.__async_raise(self.__thread_visR.ident, SystemExit)

        if self.__thread_orientation.is_alive():
            self.__async_raise(self.__thread_orientation.ident, SystemExit)
        if self.__thread_orientation_deg.is_alive():
            self.__async_raise(self.__thread_orientation_deg.ident, SystemExit)

        print("\n[PhysicalBody] All thread killed!")

    def move_forward(self, vel):
        self.__set_motor_velocity(vel, vel, vel, vel)

    def move_backward(self, vel):
        self.__set_motor_velocity(-vel, -vel, -vel, -vel)

    def turn(self, vel_rail_l, vel_rail_r):
        self.__set_motor_velocity(vel_rail_l, vel_rail_r, vel_rail_l, vel_rail_r)

    def stop(self):
        self.__set_motor_velocity(0, 0, 0, 0)

    def __set_motor_velocity(self, vel_FL, vel_FR, vel_RL, vel_RR):
        _id = self.__clientID
        _op_mode = s.simx_opmode_streaming

        s.simxSetJointTargetVelocity(_id, self.__flmotor_handle, vel_FL, _op_mode)
        s.simxSetJointTargetVelocity(_id, self.__frmotor_handle, vel_FR, _op_mode)
        s.simxSetJointTargetVelocity(_id, self.__rlmotor_handle, vel_RL, _op_mode)
        s.simxSetJointTargetVelocity(_id, self.__rrmotor_handle, vel_RR, _op_mode)

    def __get_front_distance(self, sample_delay):
        while True:
            _, vec3 = self.__front_proximity_sensor.read()
            self._proxF = vec3.distance()
            if sample_delay:
                zzz(sample_delay)

    def __get_back_distance(self, sample_delay):
        while True:
            _, vec3 = self.__back_proximity_sensor.read()
            self._proxB = vec3.distance()
            if sample_delay:
                zzz(sample_delay)

    def __get_left_distance(self, sample_delay):
        while True:
            _, vec3 = self.__left_proximity_sensor.read()
            self._proxL = vec3.distance()
            if sample_delay:
                zzz(sample_delay)

    def __get_right_distance(self, sample_delay):
        while True:
            _, vec3 = self.__right_proximity_sensor.read()
            self._proxR = vec3.distance()
            if sample_delay:
                zzz(sample_delay)

    def __black_color_detected_left(self, sample_delay):
        while True:
            try:
                arr = s.simxReadVisionSensor(self.__clientID, self.__lvs_handle, s.simx_opmode_buffer)[2]
                if arr:
                    self._visL = arr[0][11] < 0.5
                del arr
            except IndexError:
                self._visL = None

            if sample_delay:
                zzz(sample_delay)

    def __black_color_detected_centre(self, sample_delay):
        while True:
            try:
                arr = s.simxReadVisionSensor(self.__clientID, self.__cvs_handle, s.simx_opmode_buffer)[2]
                if arr:
                    self._visC = arr[0][11] < 0.5
                del arr
            except IndexError:
                self._visC = None

            if sample_delay:
                zzz(sample_delay)

    def __black_color_detected_right(self, sample_delay):
        while True:
            try:
                arr = s.simxReadVisionSensor(self.__clientID, self.__rvs_handle, s.simx_opmode_buffer)[2]
                if arr:
                    self._visR = arr[0][11] < 0.5
                del arr
            except IndexError:
                self._visR = None

            if sample_delay:
                zzz(sample_delay)

    def __get_orientation(self, sample_delay):
        while True:
            self._orientation = \
                s.simxGetObjectOrientation(self.__code_robot, self.__handle_robot, self.__handle_parent,
                                           s.simx_opmode_buffer)[1][2]

            if sample_delay:
                zzz(sample_delay)

    def __get_orientation_degrees(self, sample_delay):
        while True:
            if self._orientation is not None:
                self._orientation_deg = radians_to_degrees(self._orientation)

            if sample_delay:
                zzz(sample_delay)

    def __async_raise(self, tid, exctype):
        """raises the exception, performs cleanup if needed"""
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

    def get_proxF(self):
        return self._proxF if self._proxF is None or self._proxF > 0 else None

    def get_proxL(self):
        return self._proxL if self._proxF is None or self._proxL > 0 else None

    def get_proxR(self):
        return self._proxR if self._proxF is None or self._proxR > 0 else None

    def get_proxB(self):
        return self._proxB if self._proxF is None or self._proxB > 0 else None

    def get_visL(self):
        return self._visL

    def get_visC(self):
        return self._visC

    def get_visR(self):
        return self._visR

    def get_orientation(self):
        return self._orientation

    def get_orientation_deg(self):
        return self._orientation_deg
