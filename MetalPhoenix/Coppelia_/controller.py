from time import sleep
from math import pi
from physical_body import PhysicalBody
from utility import StdoutLogger, Compass, \
    LIFOStack, normalize_compass, negate_compass, normalize_angle, round_v, Clockwise


OR_MAX_ATTEMPT = 10


class Controller:
    def __init__(self):
        self.__class_logger = StdoutLogger(class_name="Controller", color="cyan")

        self._body = PhysicalBody()
        self._stack = LIFOStack()

        self._speed = 15
        self._rot_speed = 10

        self.target = 0

    def algorithm(self):
        while True:
            self.__go_on()
            self.target = self._stack.pop()
            self.rotate_to_final_g(self._rot_speed, self.target)

    def __go_on(self, _was_insert_l=True, _was_insert_r=True):
        self._body.move_forward(self._speed)

        _front = self._body.get_proxF()

        while _front is None or _front > 0.22:
            if self._body.get_gate():
                self.__class_logger.log("!!!FINISH!!!", 0)
                self._body.stop()
                exit(1)

            _front = self._body.get_proxF()
            _right = self._body.get_proxR()
            _left = self._body.get_proxL()
            _ori = self._body.get_orientation_deg()

            if _left is not None:
                _was_insert_l = False

            if _right is not None:
                _was_insert_r = False

            if _left is None and not _was_insert_l:
                self._stack.push(normalize_compass(_ori, Compass.OVEST))
                _was_insert_l = True

            if _right is None and not _was_insert_r:
                self._stack.push(normalize_compass(_ori, Compass.EST))
                _was_insert_r = True

        self._body.stop()

    def __think(self):
        _right = self._body.get_proxR()
        _left = self._body.get_proxL()

        print("STACK: ", self._stack.stack)

        try:
            if _left is None:
                act = self._stack.pop()
                self._queue.push(negate_compass(act))
                return

            if _right is None:
                act = self._stack.pop()
                self._queue.push(negate_compass(act))
                return

        except IndexError as e:
            self.__class_logger.log("[STACK] empty stack!", 4)
            exit(-1)

    def rotate_to_final_g(self, vel, final_g):
        """
           Funzione rotate che permette di far ruotare il robot fino a che non raggiunge final_g
        """
        self._body.stop()

        init_g = self._body.get_orientation_deg()
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)

        self.__do_rotation(vel=vel, c=c, degrees=degrees, final_g=final_g)

        self._body.stop()

    def __do_rotation(self, vel, c: Clockwise, degrees, final_g):
        degrees = abs(degrees)

        self._body.stop()
        self.__rotate(vel, c, degrees)

        ok, curr_g, limit_range = self.check_orientation(final_g)

        if not ok:
            ok, it = self.adjust_orientation(final_g)

        if it == OR_MAX_ATTEMPT:  # porca vacca!
            print("ERROR")
            #  DA GESTIRE MEGLIO
            exit(-1)

    def __rotate(self, vel, c: Clockwise, degrees):
        """
        Function that given vel, Clockwise and rotation degrees computes
        the rotation of the Robot around the z axis
        """
        degrees = abs(degrees)
        init_g = self._body.get_orientation_deg()

        delta = 0.8
        stop = False
        archived = False
        performed_deg = 0.0

        while not stop:
            curr_g = self._body.get_orientation_deg()

            if c == Clockwise.RIGHT:
                self._body.turn(vel, -vel)
            elif c == Clockwise.LEFT:
                self._body.turn(-vel, vel)

            performed_deg_temp = self.compute_performed_degrees(c, init_g, curr_g)
            if performed_deg_temp > 180:
                continue
            performed_deg = performed_deg_temp

            if degrees - delta < performed_deg < degrees + delta:
                archived = True
                stop = True
            elif performed_deg > degrees + delta:
                archived = False
                stop = True

        self._body.stop()
        return archived, init_g, performed_deg, degrees

    def check_orientation(self, final_g, delta=2):
        self.__class_logger.log("Checking if the orientation is correct ...")

        curr_g = self._body.get_orientation_deg()

        ok = False
        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if curr_g < limit_g_sx or curr_g > limit_g_dx:
                self.__class_logger.log("Perfect orientation")
                ok = True
            else:
                self.__class_logger.log("Bad orientation", )
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= curr_g <= limit_g_sx:
                self.__class_logger.log("Perfect orientation")
                ok = True
            else:
                self.__class_logger.log("Bad orientation")

        self.__class_logger.log(
            f"Limit range:[{round_v(limit_g_sx)}, {round_v(limit_g_dx)}], curr_g: {round_v(curr_g)}")

        limit_range = [limit_g_sx, limit_g_dx]
        return ok, curr_g, limit_range

    def adjust_orientation(self, final_g):
        self._body.stop()

        ok = False
        it = 0

        while not ok and it < OR_MAX_ATTEMPT:
            curr_g = self._body.get_orientation_deg()

            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)

            self.__class_logger.log(f"Adjusting orientation, attempts: {it + 1} / {OR_MAX_ATTEMPT}", 2)
            self.__class_logger.log(f"[Degrees_to_do, curr_g, final_g] = [{round_v(degrees)}, {round_v(curr_g)}, {round_v(final_g)}]", 2)

            if abs(degrees) < 6:
                self.__rotate(0.25, c, abs(degrees))
            else:
                self.__rotate(45 * pi / 180, c, abs(degrees))

            ok, curr_g, limit_range = self.check_orientation(final_g)
            it += 1

        return ok, it

    def compute_performed_degrees(self, c, init_g, curr_g):
        """Calcola l'angolo tra init_g e curr_g che il robot ha eseguito in base al senso di rotazione"""

        if init_g == curr_g:
            return 0

        init_g_360 = normalize_angle(init_g, 0)
        curr_g_360 = normalize_angle(curr_g, 0)

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

    def best_angle_and_rotation_way(self, init_g, final_g):
        """Calcola l'angolo migliore (minimo) tra init_g e final_g e il modo in cui bisogna ruotare"""

        if init_g == final_g:
            return 0

        init_g_360 = normalize_angle(init_g, 0)
        final_g_360 = normalize_angle(final_g, 0)

        first_angle = final_g_360 - init_g_360

        second_angle = -1 * first_angle / abs(first_angle) * (360 - abs(first_angle))
        smallest = first_angle

        if abs(first_angle) > 180:
            smallest = second_angle

        if smallest < 0:
            self.__class_logger.log(f"Ruotare in senso orario (RIGHT) di {abs(round_v(smallest))} gradi")
            c = Clockwise.RIGHT
        else:
            self.__class_logger.log(f"Ruotare in senso antirario (LEFT) di {abs(round_v(smallest))} gradi")
            c = Clockwise.LEFT

        return smallest, c
