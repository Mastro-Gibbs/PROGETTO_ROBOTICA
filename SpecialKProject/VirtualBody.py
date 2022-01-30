from pycsim import CSim
import sim as s
import simConst as sc
import utility
import time
from enum import Enum

FREENOVE = "Freenove4wd"


class Clockwise(Enum):
    RIGHT = 0
    LEFT = 1


class VirtualBody:

    def __init__(self, api: CSim):
        self._api = api

        self._front_camera = api.sensor.vision("cam1")

        self._front_left_motor = api.joint.with_velocity_control("joint_front_left_wheel")
        self._front_right_motor = api.joint.with_velocity_control("joint_front_right_wheel")
        self._rear_left_motor = api.joint.with_velocity_control("joint_rear_left_wheel")
        self._rear_right_motor = api.joint.with_velocity_control("joint_rear_right_wheel")
        self._front_proximity_sensor = api.sensor.proximity('fps')
        self._left_proximity_sensor = api.sensor.proximity('lps')
        self._right_proximity_sensor = api.sensor.proximity('rps')
        self.stop()

        self.clientID = api._id
        print(self.clientID)
        self.def_op_mode = sc.simx_opmode_oneshot_wait
        self.code_r, self.handle_r = s.simxGetObjectHandle(self.clientID, FREENOVE, self.def_op_mode)
        self.handle_parent = s.sim_handle_parent
        self.c, self.orientation = s.simxGetObjectOrientation(self.code_r, self.handle_r, self.handle_parent,
                                                              s.simx_opmode_streaming)
        print(self.orientation)

        self.stop()
        time.sleep(0.2)

    # ACT
    def move_forward(self, velocity):
        self._set_left_motors(velocity)
        self._set_right_motors(velocity)

    # ACT
    def move_backward(self, velocity):
        self._set_left_motors(-velocity)
        self._set_right_motors(-velocity)

    # ACT
    def turn(self, right_velocity, left_velocity):
        self._set_right_motors(right_velocity)
        self._set_left_motors(left_velocity)

    def turn_to_right(self, right_velocity, left_velocity):
        self.turn(- abs(right_velocity), + abs(left_velocity))

    def turn_to_left(self, right_velocity, left_velocity):
        self.turn(+ abs(right_velocity), - abs(left_velocity))

    # ACT
    def _set_left_motors(self, velocity):
        self._front_left_motor.set_target_velocity(velocity)
        self._rear_left_motor.set_target_velocity(velocity)

    # ACT
    def _set_right_motors(self, velocity):
        self._front_right_motor.set_target_velocity(velocity)
        self._rear_right_motor.set_target_velocity(velocity)

    # ACT
    def stop(self):
        self._set_right_motors(0.0)
        self._set_left_motors(0.0)

    # SENSE
    def camera(self):
        return self._front_camera.get_image()

    # SENSE
    def get_front_distance(self):
        _, vec3 = self._front_proximity_sensor.read()
        print(vec3)
        return vec3.distance()

    # SENSE
    def get_left_distance(self):
        _, vec3 = self._left_proximity_sensor.read()
        print(vec3)
        return vec3.distance()

    # SENSE
    def get_right_distance(self):
        _, vec3 = self._right_proximity_sensor.read()
        print(vec3)
        return vec3.distance()

    def get_orientation(self):
        self.c, self.orientation = \
            s.simxGetObjectOrientation(self.code_r,
                                       self.handle_r,
                                       self.handle_parent,
                                       s.simx_opmode_buffer)
        return self.orientation

    def get_degree_orientation(self):
        self.orientation = self.get_orientation()
        arr = self.orientation
        a = utility.radians_to_degree(arr[0])
        b = utility.radians_to_degree(arr[1])
        g = utility.radians_to_degree(arr[2])
        return [a, b, g]

    def rotate(self, vel, c: Clockwise):
        arr1 = self.get_degree_orientation()
        init_g = abs(arr1[2])
        print(init_g)
        stop = False
        while not stop:
            arr = self.get_degree_orientation()
            print(arr)
            curr_g = arr[2]
            print(f"init_g: {init_g}, curr_g: {curr_g}")
            diff = abs(abs(curr_g) - init_g)
            print(f"Difference: {diff}")
            if 87 < diff < 90:
                print("90° reached")
                stop = True
                self.stop()
                time.sleep(1)
            elif c == Clockwise.RIGHT:
                self.turn_to_right(vel, vel)
            elif c == Clockwise.LEFT:
                self.turn_to_left(vel, vel)


