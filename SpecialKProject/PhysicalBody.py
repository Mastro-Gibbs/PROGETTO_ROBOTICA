from pycsim import CSim
import sim as s
import simConst as sc
import utility
import time
import math
import numpy as np

from utility import *


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
        # print(force)
        # print(type(force[0]))
        # print(self.mass)
        # print(type(self.mass))
        accel = None
        if state:
            x = force[0] / self.mass[0]
            y = force[1] / self.mass[0]
            z = force[2] / self.mass[0]
            e = pow(10, 3)
            accel = [x * e, y * e, z * e]
        return accel

    # Funzione da spostare
    def black_color_detected(self):
        # print("Nero" if self._front_vision_sensor.read()[2][0][11] < 0.5 else "bianco")
        detected = False
        if self._front_vision_sensor.read()[2][0][11] < 0.5:
            detected = True
        return detected

    def get_orientation(self):
        self.c, self.orientation = \
            s.simxGetObjectOrientation(self.code_robot,
                                       self.handle_robot,
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

    """Set robot orientation using CoppeliaSimApi"""
    def set_orientation(self, g):
        rad = utility.degree_to_radians(g)
        print(rad)
        # code_w, handle_w = s.simxGetObjectHandle(self.clientID, "Cylinder15", s.simx_opmode_blocking)
        # code = s.simxSetObjectOrientation(clientID=self.clientID,
        #                                   objectHandle=handle_w,
        #                                   relativeToObjectHandle=-1,
        #                                   eulerAngles=[degree_to_radians(90), 0, 0],
        #                                   operationMode=s.simx_opmode_oneshot)
        init_orient = self.get_orientation()
        next_orient = init_orient
        next_orient[2] = rad
        code = s.simxSetObjectOrientation(clientID=self.clientID,
                                          objectHandle=self.handle_robot,
                                          relativeToObjectHandle=self.handle_parent,  # -11
                                          eulerAngles=next_orient,
                                          operationMode=s.simx_opmode_oneshot)
        print(f"Setting orientation to {g}")
        return code

    # FUNZIONE ALTERNATIVA PER LA ROTAZIONE -- DA TERMINARE
    def rotate_test(self, vel, c: Clockwise):
        init_g = self.get_degree_orientation()[2]
        print("Init_g: ", init_g)
        deg_delta = 2
        deg_goal = None
        goal_reached = False
        """
            deg_gol = 180° (oppure -180°)
            if -180<curr_g<=-176 or 176<=curr_g<=180:
                stop 
        """
        # [0, -90, -180,  180,  90]
        # [0,  90,  180, -180, -90]
        if c.RIGHT:
            if -90 < init_g < 180:
                deg_goal = init_g - 90
            elif -180 <= init_g <= -90:
                deg_goal = init_g + 270
            # deg_goal = 180
            limit_range = [deg_goal - 1, deg_goal + 1]
            print("Goal: ", deg_goal)
            print(limit_range)

        time.sleep(4)
        stop = False
        while not stop:
            # self.stop()
            curr_g = self.get_degree_orientation()[2]
            print(curr_g)
            l = min(limit_range)
            if deg_goal - deg_delta <= curr_g <= deg_goal + deg_delta:
                self.stop()
                stop = True
                print("Second check if the goal is reached")
                curr_g = self.get_degree_orientation()[2]
                if deg_goal - deg_delta <= curr_g <= deg_goal + deg_delta:
                    goal_reached = True
                    print("Goal reached")
                else:
                    initial_clockwise = c
                    self.adjust_orientation(deg_goal, initial_clockwise)
            elif np.sign(curr_g) == np.sign(l) and curr_g < l:
                print("Goal not reached accurately")
                self.stop()
                time.sleep(3)
                initial_clockwise = c
                self.adjust_orientation(deg_goal, initial_clockwise)
                stop = True
            else:
                self.turn_to_right(vel, vel)

    def adjust_orientation(self, deg_goal, initial_clockwise: Clockwise):
        print("Trying to adjust the orientation")
        print("Goal: ", deg_goal)
        limit_range = [deg_goal - 3, deg_goal + 3]
        print(limit_range)
        time.sleep(3)
        arr = self.get_degree_orientation()
        curr_g = arr[2]
        print(initial_clockwise)
        c = Clockwise.RIGHT
        if initial_clockwise.RIGHT:
            c = Clockwise.LEFT
        print(c)
        vel = 45 * math.pi / 180
        stop = False
        deg_delta = 2
        while not stop:
            # self.stop()
            curr_g = self.get_degree_orientation()[2]
            print(curr_g)
            print("here")
            if deg_goal - deg_delta <= curr_g <= deg_goal + deg_delta:
                self.stop()
                stop = True
                print("Goal reached")
                goal_reached = True
            elif c == Clockwise.RIGHT:
                self.turn_to_right(vel, vel)
            elif c == Clockwise.LEFT:
                self.turn_to_left(vel, vel)

    """Function that given vel, Clockwise and rotation degrees permits to rotate the Robot around the z axis"""
    def rotate(self, vel, c: Clockwise, degrees):
        if degrees < 4:
            return
        orient = self.get_degree_orientation()
        init_g = orient[2]
        prev_g = init_g
        deg = 0
        delta = 2
        achieved = False
        stop = False
        while not stop:
            orient = self.get_degree_orientation()
            # print(orient)
            curr_g = orient[2]
            print(f"init_g: {init_g}, curr_g: {curr_g}")

            if prev_g < -90 and curr_g > 90:
                delta_sx = 180 + prev_g
                delta_dx = 180 - curr_g
                deg = delta_sx + delta_dx + deg
            elif prev_g > 90 and curr_g < -90:
                delta_dx = 180 - prev_g
                delta_sx = 180 + curr_g
                deg = delta_sx + delta_dx + deg
            else:
                deg = abs(curr_g - prev_g) + deg

            print("Performed deg: ", deg)
            print("Round: ", int(deg / 360))

            prev_g = curr_g

            if degrees - delta < deg < degrees + delta:
                self.stop()
                performed_deg = deg
                achieved = True  # ok
                return achieved, init_g, performed_deg, degrees
            if deg > degrees + delta:
                self.stop()
                print("Error: performed degrees exceeded the target")
                performed_deg = deg
                achieved = False  # error
                return achieved, init_g, performed_deg, degrees
            if c == Clockwise.RIGHT:
                self.turn_to_right(vel, vel)
            elif c == Clockwise.LEFT:
                self.turn_to_left(vel, vel)

    def check_orientation(self, vel, c, degrees, init_g, performed_deg):
        print("Checking the orientation ...")
        orient = self.get_degree_orientation()
        curr_g = orient[2]
        deg_g = 0

    def do_rotation(self, vel, c: Clockwise, degrees):
        self.stop()
        time.sleep(0.1)
        init_g = self.get_degree_orientation()[2]
        final_g = self.compute_final_g(init_g, degrees)
        achieved, init_g, performed_deg, degrees = self.rotate(vel, c, degrees)
        self.check_orientation(vel, c, degrees, init_g, performed_deg)

    def compute_final_g(self, init_g, degrees):
        final_g = 0



        return final_g


"""
 if c == Clockwise.RIGHT:
    if prev_g < -90 and curr_g > 90:
        delta_sx = 180 + prev_g
        delta_dx = 180 - curr_g
        deg = delta_sx + delta_dx + deg
    else:
        deg = abs(curr_g - prev_g) + deg
if c == Clockwise.LEFT:
    if prev_g > 90 and curr_g < -90:
        delta_dx = 180 - prev_g
        delta_sx = 180 + curr_g
        deg = delta_sx + delta_dx + deg
    else:
        deg = abs(curr_g - prev_g) + deg
"""
