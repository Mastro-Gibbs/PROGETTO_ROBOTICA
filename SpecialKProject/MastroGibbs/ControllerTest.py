import time

from PhysicalBody import PhysicalBody, ThreadHeap


class ControllerTest:

    def __init__(self):
        self.pb = PhysicalBody()

    def sense_discover(self):
        self.pb.move_forward(2)
        self.pb.thread_heap_begin(sample_delay=0.4)
        self.pb.thread_heap_start()

        i = 10
        while i:
            if i == 8:
                self.pb.thread_heap_kill(ThreadHeap.th_orientation_deg)

            print(self.pb.get_orientation_deg())

            time.sleep(1)

            if i == 5:
                self.pb.thread_heap_revive()

            i -= 1

        self.pb.thread_heap_killall()


try:
    c = ControllerTest()
    c.sense_discover()
except KeyboardInterrupt:
    c.pb.thread_heap_killall()



'''

from PhysicalBody import PhysicalBody
# from utility import Clockwise, StringBuilder
from Controller.controller_enums import Action, Key, Compass, Semaphore
from utility import *
import time
from math import pi


DEBUG = True


class ControllerTest:
    def __init__(self, api):

        self.pb = PhysicalBody(api)
        # physicalbody
        self.target = float()
        self.curr_speed = 10.0
        self.rotation_speed = 45 * pi / 180

    def setup_reference_system(self):
        init_g = self.pb.get_orientation_degrees()[2]
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
        g = self.pb.get_orientation_degrees()[2]

        if 65.0 < g < 115.0:
            self.target = 90.0
        elif -25.0 < g < 25.0:
            self.target = 0.0
        elif -65.0 > g > -115.0:
            self.target = -90.0
        else:
            self.target = -180.0

    # CONTROLLER
    def rotate_to_final_g(self, vel, final_g):
        """
           Funzione rotate che permette di far ruotare il robot fino a che non raggiunge final_g
        """
        init_g = self.pb.get_orientation_degrees()[2]
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)
        self.__do_rotation(vel=vel, c=c, degrees=abs(degrees), final_g=final_g)

    # CONTROLLER
    def rotate_degrees(self, vel, c: Clockwise, degrees):
        """
            Funzione rotate che permette di far ruotare il robot di degrees gradi
        """
        init_g = self.pb.get_orientation_degrees()[2]
        final_g = self.compute_final_g(c, init_g, degrees)
        self.__do_rotation(vel=vel, c=c, degrees=abs(degrees), final_g=final_g)

    # CONTROLLER
    def __do_rotation(self, vel, c: Clockwise, degrees, final_g):
        self.pb.stop()
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
        init_g = self.pb.get_orientation_degrees()[2]
        prev_g = init_g
        performed_deg = 0.0
        delta = 1.5
        stop = False
        it_is_rotating = False  # it is not rotating

        while not stop:
            curr_g = self.pb.get_orientation_degrees()[2]
            if it_is_rotating:
                performed_deg = self.compute_performed_degrees(c, prev_g, curr_g) + performed_deg
            prev_g = curr_g
            debug.concat(f"[init_g, curr_g, degrees] = [{round_v(init_g)}, {round_v(curr_g)}, {round_v(degrees)}]",
                         end="\n")
            debug.concat(f"[Performed deg, Round] = [{round_v(performed_deg)}, {round_v(int(performed_deg / 360))}]",
                         end="\n")
            if degrees - delta < performed_deg < degrees + delta:
                self.pb.stop()
                it_is_rotating = False
                debug.concat("Maybe the orientation is correct ...")
                last_sampled_g = prev_g
                achieved = True  # Problema: se il robot ha slittato i gradi raggiunti
                # non sono giusti e quindi serve sempre il check dell'orientation
                return achieved, init_g, last_sampled_g, performed_deg, degrees

            if performed_deg > degrees + delta:
                self.pb.stop()
                it_is_rotating = False
                debug.concat("Error: performed degrees exceeded the target")
                last_sampled_g = prev_g
                achieved = False  # error, serve il check dell'orientation
                return achieved, init_g, last_sampled_g, performed_deg, degrees

            if c == Clockwise.RIGHT:
                self.pb.turn(vel, -vel)
                it_is_rotating = True
            elif c == Clockwise.LEFT:
                self.pb.turn(-vel, vel)
                it_is_rotating = True

            if DEBUG:
                print(debug)
                debug.erase()

    # CONTROLLER
    def check_orientation(self, final_g, delta=2):
        debug = StringBuilder()
        debug.concat("Checking if the orientation is correct ...", end="\n")

        curr_g = self.pb.get_orientation_degrees()[2]
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
        self.pb.stop()
        ok = False
        it = 0
        max_attempts = 4
        while not ok and it < max_attempts:
            curr_g = self.pb.get_orientation_degrees()[2]
            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)
            print(f"Adjusting orientation, attempts: {it + 1} / {max_attempts}")
            print(f"[Degrees_to_do, curr_g, final_g] = [{round_v(degrees)}, {round_v(curr_g)}, {round_v(final_g)}]")
            if DEBUG:
                time.sleep(8)
            if abs(degrees) < 4:
                self.__rotate(0.5, c, abs(degrees))
            else:
                self.__rotate(45 * pi / 180, c, abs(degrees))
            ok, curr_g, limit_range = self.check_orientation(final_g)
            it += 1
        return ok, it

    def balance(self):
        print("Balance")
        direction = self.target
        ok, curr_g, limit_range = self.check_orientation(direction, delta=2)
        if not ok:
            print("NOT OK")
            self.pb.stop()
            ok, it = self.adjust_orientation(direction)
        if not ok:
            print("ERROR")
            exit()
        critical, side, distance = self.check_side_distances()
        if critical:
            self.adjust_position(side, distance, direction)

    def check_side_distances(self):
        ld = self.pb.get_left_distance()
        rd = self.pb.get_right_distance()
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
        self.rotate_to_final_g(vel=45 * pi / 180, final_g=new_g)
        while self.pb.get_back_distance() < 0.125:
            self.pb.move_forward(0.4)
        self.pb.stop()
        self.rotate_to_final_g(vel=45 * pi / 180, final_g=direction)
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
        self.detect_target()
        curr_g = self.pb.get_orientation_degrees()[2]

        if abs(curr_g) > abs(self.target + 3) or abs(curr_g) < abs(self.target - 3):
            c = self.adjust_orientation_line(curr_g)

            self.go_to_line(c)

            curr_g = self.pb.get_orientation_degrees()[2]
            self.adjust_orientation_line(curr_g, prev_overflow=True)

    def adjust_orientation_line(self, curr_g, prev_overflow=False):
        overflow = abs(abs(self.target) - abs(curr_g))
        c = Clockwise

        if self.target == 90:  # case target = 90
            if abs(curr_g) > abs(self.target):
                c = Clockwise.RIGHT
            else:
                c = Clockwise.LEFT

        elif self.target == 0:  # case target = 0
            if curr_g > self.target:
                c = Clockwise.RIGHT
            else:
                c = Clockwise.LEFT

        elif self.target == -90:  # case target = 0
            if abs(curr_g) > abs(self.target):
                c = Clockwise.LEFT
            else:
                c = Clockwise.RIGHT

        else:  # case target = -180/180
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
            self.pb.move_forward(self.curr_speed)

            if c == Clockwise.LEFT and self.pb.black_color_detected_right():
                break

            elif c == Clockwise.RIGHT and self.pb.black_color_detected_left():
                break

    def rotate_without_blocking(self, vel, c: Clockwise, degrees):
        init_g = self.pb.get_orientation_degrees()[2]
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
            curr_g = self.pb.get_orientation_degrees()[2]

            performed_deg = self.compute_performed_degrees(c, prev_g, curr_g) + performed_deg
            prev_g = curr_g

            if degrees - delta < performed_deg < degrees + delta:
                achieved = True
                break
            elif performed_deg > degrees + delta:
                achieved = False
                break

            self.pb.turn(vel_railL, vel_railR)

        last_sampled_g = prev_g
        return achieved, init_g, last_sampled_g, performed_deg, degrees

    # ##################### END METHODS FOR LINE FOLLOWING###############

    def go_forw(self):
        while self.pb.get_front_distance() > Semaphore.RED:
            if self.pb.get_front_distance() > Semaphore.GREEN:
                self.pb.move_forward(self.curr_speed)
            elif self.pb.get_front_distance() <= Semaphore.YELLOW:
                self.pb.move_forward(self.curr_speed / 2)
            self.balance()
        self.pb.stop()

    def algorithm(self):
        self.target = 90
        self.go_forw()
        self.rotate_to_final_g(self.rotation_speed, 180)
        self.target = 180
        self.go_forw()
        """
        self.rotate_to_final_g(self.rotation_speed, 90)
        self.rotate_to_final_g(self.rotation_speed, 180)
        self.go_forw()
        self.rotate_to_final_g(self.rotation_speed, -90)
        self.go_forw()
        self.rotate_to_final_g(self.rotation_speed, 180)
        self.go_forw()
        self.rotate_to_final_g(self.rotation_speed, 90)
        self.go_forw()"""



    def algorithm_(self):
        actions = [1, 180, 1, 90, 1, 180, 1, -90, 1, 180, 1, 90, 1, 0, 1, 90, 1, 0, 1, 90, 1, 180,
                   1, -90, 1, 180, 1, -90, 1, 180, 1, -90, 1, 180, 1, 90, 1, 0, 1, 90, 1, 180, 1,
                   90, 1, 0, 1, 90, 1, 180, 1, 90, 1]
        vel = 5
        vel_rot = 45 * pi / 180
        i = 0
        direction = 90
        print(Action.GO_FORWARD)
        while len(actions) != i:
            print("f")
            if actions[i] == 1:
                while self.pb.get_front_distance() > 0.15:
                    self.pb.move_forward(vel)
                    self.balance(direction)
                self.pb.stop()
            elif actions[i] != 1:
                self.rotate_to_final_g(vel_rot, actions[i])
                direction = actions[i]
            i += 1

    def fakemain(self):
        GO = True
        # time.sleep(2)
        vel = 45 * pi / 180
        vel = 3
        # algorithm(pb)
        # rotation_test(pb, vel)
        i = 0
        self.curr_speed = vel
        while True:
            if DEBUG:
                print(f"[{self.pb.get_left_distance()},{self.pb.get_front_distance()}, {self.pb.get_right_distance()}]")

            self.pb.move_forward(vel)
            i += 2

            if i % (50 // vel) == 0:
                self.balance_line()

    def fakemain2(self):

        vel = 45 * pi / 180
        vel = 2
        while True:
            if DEBUG:
                print(f"[{self.pb.get_left_distance()},{self.pb.get_front_distance()}, {self.pb.get_right_distance()}]")

            self.pb.move_forward(vel)
            # self.pb.turn(vel, -vel)
            self.balance(90)

'''
