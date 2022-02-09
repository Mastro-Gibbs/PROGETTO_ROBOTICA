from pycsim import CSim
import RemoteApiPython.sim as s
import RemoteApiPython.simConst as sc
import utility
import time
import math
import numpy as np
from utility import *

DEBUG = True

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

    def get_orientation_degrees(self):
        self.orientation = self.get_orientation()
        arr = self.orientation
        a = utility.radians_to_degrees(arr[0])
        b = utility.radians_to_degrees(arr[1])
        g = utility.radians_to_degrees(arr[2])
        return [a, b, g]

    """Set robot orientation using CoppeliaSimApi"""

    def set_orientation(self, g):
        rad = utility.degrees_to_radians(g)
        print(rad)
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

    # CONTROLLER
    def rotate_to_final_g(self, vel, final_g):
        """
           Funzione rotate che permette di far ruotare il robot fino a che non raggiunge final_g
        """
        init_g = self.get_orientation_degrees()[2]
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)
        self.do_rotation(vel=vel, c=c, degrees=abs(degrees), final_g=final_g)

    # CONTROLLER
    def rotate_degrees(self, vel, c: Clockwise, degrees):
        """
            Funzione rotate che permette di far ruotare il robot di degrees gradi
        """
        init_g = self.get_orientation_degrees()[2]
        final_g = self.compute_final_g(c, init_g, degrees)
        self.do_rotation(vel=vel, c=c, degrees=abs(degrees), final_g=final_g)

    # CONTROLLER
    def do_rotation(self, vel, c: Clockwise, degrees, final_g):
        self.stop()
        degrees = abs(degrees)
        print("final_g: ", round_v(final_g))
        if DEBUG:
            time.sleep(5)
        self.__rotate(vel, c, degrees)
        ok, curr_g, limit_range = self.check_orientation(final_g)
        if not ok:
            ok, it = self.adjust_orientation(final_g)
        if not ok:
            print("ERROR")
            exit()

    # CONTROLLER
    def __rotate(self, vel, c: Clockwise, degrees):
        """
        Function that given vel, Clockwise and rotation degrees computes
        the rotation of the Robot around the z axis
        """
        degrees = abs(degrees)
        init_g = self.get_orientation_degrees()[2]
        prev_g = init_g
        performed_deg = 0.0
        delta = 1.5
        stop = False
        while not stop:
            curr_g = self.get_orientation_degrees()[2]
            performed_deg = self.compute_performed_degrees(c, prev_g, curr_g) + performed_deg
            prev_g = curr_g
            print(f"[init_g, curr_g, degrees] = [{round_v(init_g)}, {round_v(curr_g)}, {round_v(degrees)}]")
            print(f"[Performed deg, Round] = [{round_v(performed_deg)}, {round_v(int(performed_deg / 360))}]")
            if degrees - delta < performed_deg < degrees + delta:
                self.stop()
                print("Maybe the orientation is correct ...")
                last_sampled_g = prev_g
                achieved = True  # Problema: se il robot ha slittato i gradi raggiunti
                # non sono giusti e quindi serve sempre il check dell'orientation
                return achieved, init_g, last_sampled_g, performed_deg, degrees
            if performed_deg > degrees + delta:
                self.stop()
                print("Error: performed degrees exceeded the target")
                last_sampled_g = prev_g
                achieved = False  # error, serve il check dell'orientation
                return achieved, init_g, last_sampled_g, performed_deg, degrees
            if c == Clockwise.RIGHT:
                self.turn_to_right(vel, vel)
            elif c == Clockwise.LEFT:
                self.turn_to_left(vel, vel)

    # CONTROLLER
    def check_orientation(self, final_g):
        print("Checking if the orientation is correct ...")
        time.sleep(3)
        curr_g = self.get_orientation_degrees()[2]
        delta = 2
        ok = False
        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if curr_g < limit_g_sx or curr_g > limit_g_dx:
                print("Perfect orientation")
                ok = True
            else:
                print("Bad orientation")
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= curr_g <= limit_g_sx:
                print("Perfect orientation")
                ok = True
            else:
                print("Bad orientation")

        print(f"Limit range:[{round_v(limit_g_sx)}, {round_v(limit_g_dx)}], curr_g: {round_v(curr_g)}")
        limit_range = [limit_g_sx, limit_g_dx]

        if DEBUG:
            time.sleep(4)
        return ok, curr_g, limit_range

    # CONTROLLER
    def adjust_orientation(self, final_g):
        ok = False
        it = 0
        max_attempts = 4
        while not ok and it < max_attempts:
            curr_g = self.get_orientation_degrees()[2]
            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)
            print(f"Adjusting orientation, attempts: {it + 1} / {max_attempts}")
            print(f"[Degrees_to_do, curr_g, final_g] = [{round_v(degrees)}, {round_v(curr_g)}, {round_v(final_g)}]")
            if DEBUG:
                time.sleep(8)
            if abs(degrees) < 4:
                self.__rotate(0.5, c, abs(degrees))
            else:
                self.__rotate(45 * math.pi / 180, c, abs(degrees))
            ok, curr_g, limit_range = self.check_orientation(final_g)
            it += 1
        return ok, it

    # CONTROLLER
    def compute_final_g(self, c: Clockwise, init_g, degrees):
        print("Computing final_g ...")
        if c == Clockwise.RIGHT:
            degrees = -1 * degrees
        init_g_360 = normalize_angle(init_g, 0)
        final_g_360 = init_g_360 + degrees
        final_g = normalize_angle(final_g_360, 1)
        print(f"[init_g, final_g, degrees, clockwise] = [{round_v(init_g)}, {round_v(final_g)}, {abs(degrees)}, {c}]")
        return final_g

    # CONTROLLER
    def compute_performed_degrees(self, c, init_g, curr_g):
        """Calcola l'angolo tra init_g e curr_g che il robot ha eseguito in base al senso di rotazione"""

        if init_g == curr_g:
            return 0
        # Trasformo gli angoli compresi tra [-180,180] ai corrispondenti angoli tra [0,360]
        init_g_360 = normalize_angle(init_g, 0)
        curr_g_360 = normalize_angle(curr_g, 0)
        # Calcolo la differenza
        first_angle = curr_g_360 - init_g_360
        second_angle = -1 * first_angle / abs(first_angle) * (360 - abs(first_angle))
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

    # CONTROLLER
    def best_angle_and_rotation_way(self, init_g, final_g):
        """Calcola l'angolo migliore (minimo) tra init_g e final_g e il modo in cui bisogna ruotare"""
        if init_g == final_g:
            return 0
        # Trasformo gli angoli compresi tra [-180,180] ai corrispondenti angoli tra [0,360]
        init_g_360 = normalize_angle(init_g, 0)
        final_g_360 = normalize_angle(final_g, 0)
        # Calcolo la differenza
        first_angle = final_g_360 - init_g_360
        # Calcolo l'angolo più piccolo
        second_angle = -1 * first_angle / abs(first_angle) * (360 - abs(first_angle))
        smallest = first_angle
        if abs(first_angle) > 180:
            smallest = second_angle
        if smallest < 0:
            print(f"Ruotare in senso orario (RIGHT) di {abs(round_v(smallest))} gradi")
            c = Clockwise.RIGHT
        else:
            print(f"Ruotare in senso antirario (LEFT) di {abs(round_v(smallest))} gradi")
            c = Clockwise.LEFT

        return smallest, c
