from pycsim import CSim
import RemoteApiPython.sim as s
import time
from utility import *

DEBUG = False


class PhysicalBody:

    def __init__(self, api: CSim):
        self._api = api
        self.clientID = api._id
        # print(self.clientID)

        # SENSORS
        self._front_camera = api.sensor.vision("cam1")
        self._front_vision_sensor = api.sensor.vision("fvs")
        self._right_vision_sensor = api.sensor.vision("rvs")
        self._left_vision_sensor = api.sensor.vision("lvs")
        self._front_proximity_sensor = api.sensor.proximity('fps')
        self._back_proximity_sensor = api.sensor.proximity('bps')
        self._left_proximity_sensor = api.sensor.proximity('lps')
        self._right_proximity_sensor = api.sensor.proximity('rps')

        # ACCELEROMETRO
        self.accelerometer = \
            s.simxGetObjectHandle(self.clientID, 'Accelerometer_forceSensor', s.simx_opmode_blocking)
        self.mass_object = \
            s.simxGetObjectHandle(self.clientID, 'Accelerometer_mass', s.simx_opmode_streaming)
        # print(type(self.mass_object))
        self.mass = \
            s.simxGetObjectFloatParam(self.clientID, self.mass_object[1], s.sim_shapefloatparam_mass,
                                      s.simx_opmode_streaming)
        ret, value, arr1, arr2 = \
            s.simxReadForceSensor(self.clientID, self.accelerometer[1], s.simx_opmode_streaming)

        # MOTORS
        self._front_left_motor = api.joint.with_velocity_control("joint_front_left_wheel")
        self._front_right_motor = api.joint.with_velocity_control("joint_front_right_wheel")
        self._rear_left_motor = api.joint.with_velocity_control("joint_rear_left_wheel")
        self._rear_right_motor = api.joint.with_velocity_control("joint_rear_right_wheel")

        # READING INITIAL ORIENTATION OF THE ROBOT
        self.code_robot, self.handle_robot = \
            s.simxGetObjectHandle(self.clientID, FREENOVE, s.simx_opmode_oneshot_wait)
        self.handle_parent = s.sim_handle_parent
        self.c, self.orientation = \
            s.simxGetObjectOrientation(self.code_robot, self.handle_robot, self.handle_parent, s.simx_opmode_streaming)
        if DEBUG:
            print(self.orientation)

        self.stop()
        time.sleep(0.2)

    # ACT
    def move_forward(self, velocity):
        self._front_left_motor.set_target_velocity(velocity)
        self._rear_left_motor.set_target_velocity(velocity)
        self._front_right_motor.set_target_velocity(velocity)
        self._rear_right_motor.set_target_velocity(velocity)

    # ACT
    def move_backward(self, velocity):
        self._front_left_motor.set_target_velocity(-velocity)
        self._rear_left_motor.set_target_velocity(-velocity)
        self._front_right_motor.set_target_velocity(-velocity)
        self._rear_right_motor.set_target_velocity(-velocity)

    # ACT
    def turn(self, left_velocity, right_velocity):
        self._front_left_motor.set_target_velocity(left_velocity)
        self._rear_left_motor.set_target_velocity(left_velocity)
        self._front_right_motor.set_target_velocity(right_velocity)
        self._rear_right_motor.set_target_velocity(right_velocity)

    # ACT
    def stop(self):
        self._front_left_motor.set_target_velocity(0)
        self._rear_left_motor.set_target_velocity(0)
        self._front_right_motor.set_target_velocity(0)
        self._rear_right_motor.set_target_velocity(0)

    # SENSE
    def get_front_distance(self):
        _, vec3 = self._front_proximity_sensor.read()
        # print(vec3)
        return vec3.distance()

    def get_back_distance(self):
        _, vec3 = self._back_proximity_sensor.read()
        # print(vec3)
        return vec3.distance()

    # SENSE
    def get_left_distance(self):
        _, vec3 = self._left_proximity_sensor.read()
        # print(vec3)
        return vec3.distance()

    # SENSE
    def get_right_distance(self):
        _, vec3 = self._right_proximity_sensor.read()
        # print(vec3)
        return vec3.distance()

    # SENSE
    def get_fvs_values(self):
        return self._front_vision_sensor.read()

    # SENSE
    def get_rvs_values(self):
        return self._right_vision_sensor.read()

    # SENSE
    def get_lvs_values(self):
        return self._left_vision_sensor.read()

    def get_accelerometer(self):
        code, state, force, torque = s.simxReadForceSensor(self.clientID, self.accelerometer[1], s.simx_opmode_buffer)
        accel = None
        if state:
            x = force[0] / self.mass[0]
            y = force[1] / self.mass[0]
            z = force[2] / self.mass[0]
            e = pow(10, 3)
            accel = [x * e, y * e, z * e]
        return accel

    def black_color_detected_left(self):
        return self._left_vision_sensor.read()[2][0][11] < 0.5

    def black_color_detected_front(self):
        return self._front_vision_sensor.read()[2][0][11] < 0.5

    def black_color_detected_right(self):
        return self._right_vision_sensor.read()[2][0][11] < 0.5

    def get_orientation(self):
        self.c, self.orientation = \
            s.simxGetObjectOrientation(self.code_robot,
                                       self.handle_robot,
                                       self.handle_parent,
                                       s.simx_opmode_buffer)
        return self.orientation

    def get_orientation_degrees(self):
        self.orientation = self.get_orientation()
        arr = self.orientation
        a = radians_to_degrees(arr[0])
        b = radians_to_degrees(arr[1])
        g = radians_to_degrees(arr[2])
        return [a, b, g]
