from cestino.pycsim import CSim
import RemoteApiPython.sim as s
from utility import *
import utility
import time
import math

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

        self.target = float()
        self.curr_speed = float()

        self.stop()
        time.sleep(0.2)

    def setup_reference_system(self):
        init_g = self.get_orientation_degrees()[2]
        if 45 < init_g < 135:
            NORD = 90
        elif -45 <= init_g <= 45:
            NORD = 0
        elif -135 < init_g < -45:
            NORD = -90
        else:
            NORD = 180
        EST = NORD - 90
        OVEST = NORD + 90
        SUD = NORD - 180

        print(f"[N,E,S,O] = [{NORD}, {EST}, {SUD}, {OVEST}]")

    def detect_target(self):
        g = self.get_orientation_degrees()[2]

        if 65.0 < g < 115.0:
            self.target = 90.0
        elif -25.0 < g < 25.0:
            self.target = 0.0
        elif -65.0 > g > -115.0:
            self.target = -90.0
        else:
            self.target = -180.0

    # ACT
    def move_forward(self, velocity):
        self.detect_target()
        self.curr_speed = velocity
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

    def black_color_detected_right(self):
        detected = False
        if self._right_vision_sensor.read()[2][0][11] < 0.5:
            detected = True
        return detected

    def black_color_detected_left(self):
        detected = False
        if self._left_vision_sensor.read()[2][0][11] < 0.5:
            detected = True
        return detected

    def black_color_detected_front(self):
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
        debug = StringBuilder()

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
            debug.concat(f"[init_g, curr_g, degrees] = [{round_v(init_g)}, {round_v(curr_g)}, {round_v(degrees)}]",
                         end="\n")
            debug.concat(f"[Performed deg, Round] = [{round_v(performed_deg)}, {round_v(int(performed_deg / 360))}]",
                         end="\n")
            if degrees - delta < performed_deg < degrees + delta:
                self.stop()
                debug.concat("Maybe the orientation is correct ...")
                last_sampled_g = prev_g
                achieved = True  # Problema: se il robot ha slittato i gradi raggiunti
                # non sono giusti e quindi serve sempre il check dell'orientation
                return achieved, init_g, last_sampled_g, performed_deg, degrees

            if performed_deg > degrees + delta:
                self.stop()
                debug.concat("Error: performed degrees exceeded the target")
                last_sampled_g = prev_g
                achieved = False  # error, serve il check dell'orientation
                return achieved, init_g, last_sampled_g, performed_deg, degrees

            if c == Clockwise.RIGHT:
                self.turn_to_right(vel, vel)
            elif c == Clockwise.LEFT:
                self.turn_to_left(vel, vel)

            if DEBUG:
                print(debug)
                debug.erase()

    # CONTROLLER
    def check_orientation(self, final_g, delta=2):
        debug = StringBuilder()
        debug.concat("Checking if the orientation is correct ...", end="\n")

        time.sleep(3)
        curr_g = self.get_orientation_degrees()[2]
        # delta = 2
        ok = False
        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if curr_g < limit_g_sx or curr_g > limit_g_dx:
                debug.concat("Perfect orientation", end="\n")
                ok = True
            else:
                debug.concat("Bad orientation", end="\n")
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= curr_g <= limit_g_sx:
                debug.concat("Perfect orientation", end="\n")
                ok = True
            else:
                debug.concat("Bad orientation", end="\n")

        debug.concat(f"Limit range:[{round_v(limit_g_sx)}, {round_v(limit_g_dx)}], curr_g: {round_v(curr_g)}")

        limit_range = [limit_g_sx, limit_g_dx]

        if DEBUG:
            print(debug)
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

    def balance(self, direction):
        ok, curr_g, limit_range = self.check_orientation(direction, delta=2)
        if not ok:
            ok, it = self.adjust_orientation(direction)
        if not ok:
            print("ERROR")
            exit()
        critical, side, distance = self.check_side_distances()
        if critical:
            self.adjust_position(side, distance, direction)

    def check_side_distances(self):
        ld = self.get_left_distance()
        rd = self.get_right_distance()
        critical = False
        if ld < CRITICAL_SIDE_DISTANCE:
            critical = True
            return critical, "LEFT", ld
        elif rd < CRITICAL_SIDE_DISTANCE:
            critical = True
            return critical, "RIGHT", rd
        else:
            return critical, None, None

    def adjust_position(self, side, distance, direction):
        print("--------------START: Adjusting position------------")
        if side == "RIGHT":
            new_g = direction + 90
        else:
            new_g = direction - 90
        self.rotate_to_final_g(vel=45 * math.pi / 180, final_g=new_g)
        # sarebbe meglio usare i metri o usare il sensore di dietro
        while self.get_front_distance() > 0.125:
            self.move_forward(0.4)
        self.stop()
        self.rotate_to_final_g(vel=45 * math.pi / 180, final_g=direction)
        print("--------------END: Adjusting position------------")

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

        debug = StringBuilder()
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
            debug.concat(f"Ruotare in senso orario (RIGHT) di {abs(round_v(smallest))} gradi")
            c = Clockwise.RIGHT
        else:
            debug.concat(f"Ruotare in senso antirario (LEFT) di {abs(round_v(smallest))} gradi")
            c = Clockwise.LEFT

        if DEBUG:
            print(debug)
        return smallest, c

    # ################# METHODS FOR LINE FOLLOWING #####################

    def balance_line(self):
        curr_g = self.get_orientation_degrees()[2]

        if abs(curr_g) > abs(self.target + 2) or abs(curr_g) < abs(self.target - 2):
            c = self.adjust_orientation_line(curr_g)

            self.go_to_line(c)

            curr_g = self.get_orientation_degrees()[2]
            self.adjust_orientation_line(curr_g, prev_overflow=True)

    def adjust_orientation_line(self, curr_g, prev_overflow=False):
        overflow = abs(abs(self.target) - abs(curr_g))
        c = Clockwise

        if self.target == 90:                # case target = 90
            if abs(curr_g) > abs(self.target):
                c = Clockwise.RIGHT
            else:
                c = Clockwise.LEFT

        elif self.target == 0:             # case target = 0
            if curr_g > self.target:
                c = Clockwise.RIGHT
            else:
                c = Clockwise.LEFT

        elif self.target == -90:           # case target = 0
            if abs(curr_g) > abs(self.target):
                c = Clockwise.LEFT
            else:
                c = Clockwise.RIGHT

        else:                              # case target = -180/180
            if curr_g > 0:
                c = Clockwise.LEFT
            else:
                c = Clockwise.RIGHT

        if not prev_overflow:
            self.rotate_without_blocking(self.curr_speed, c, overflow * 2)
        else:
            self.rotate_without_blocking(self.curr_speed, c, overflow)

        return c

    def go_to_line(self, c):
        while True:
            self.move_forward(self.curr_speed)

            if c == Clockwise.LEFT and self.black_color_detected_right():
                break

            elif c == Clockwise.RIGHT and self.black_color_detected_left():
                break

    def rotate_without_blocking(self, vel, c: Clockwise, degrees):
        init_g = self.get_orientation_degrees()[2]
        prev_g = init_g

        performed_deg = 0.0
        delta = 1.5

        vel_railL = vel
        vel_railR = vel
        delta_vel = (vel * ((pi * 3) / 7)) - vel

        if c == Clockwise.RIGHT:
            vel_railL += delta_vel
            vel_railR -= delta_vel
        elif c == Clockwise.LEFT:
            vel_railL -= delta_vel
            vel_railR += delta_vel

        while True:
            curr_g = self.get_orientation_degrees()[2]

            performed_deg = self.compute_performed_degrees(c, prev_g, curr_g) + performed_deg
            prev_g = curr_g

            if degrees - delta < performed_deg < degrees + delta:
                achieved = True
                break
            elif performed_deg > degrees + delta:
                achieved = False
                break

            self.turn(vel_railR, vel_railL)

        last_sampled_g = prev_g
        return achieved, init_g, last_sampled_g, performed_deg, degrees

    # ##################### END METHODS FOR LINE FOLLOWING#################
