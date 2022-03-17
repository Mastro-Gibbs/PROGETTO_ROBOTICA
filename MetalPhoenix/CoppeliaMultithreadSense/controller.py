from time import sleep, time
from math import pi
from physical_body import PhysicalBody, ThreadType
from utility import short_way, detect_target, StdoutLogger, Compass, \
    LIFOStack, FIFOStack, normalize_compass, negate_compass, normalize_angle, round_v, Clockwise

DEBUG = False


class Controller:
    def __init__(self):
        self.__class_logger = StdoutLogger(class_name="Controller", color="cyan")

        self._body = PhysicalBody()
        self._stack = LIFOStack()
        self._queue = LIFOStack()

        self._speed = 5
        self._rot_speed = 0.7

        self.target = 0

        """
        # GET ROBOT INIT ORIENTATION, WAKE UP ORIENTATION THREAD
        self.body.thread_begin(ThreadType.th_orientation)
            
        self._start_orientation = None
        while self._start_orientation is None:
            self._start_orientation = self.body.get_orientation_deg()
            sleep(0.01)

        self.__class_logger.log("Start orientation: {0} degrees".format(self._start_orientation), -1)

        if not self.align_robot(target=Compass.OVEST):
            self.__class_logger.log("Alignment fail!", 4)

        self.body.thread_kill(ThreadType.th_orientation)
        """

    def expire(self):
        self._body.safe_exit()

    def algorithm(self):
        self._body.thread_begin(ThreadType.th_prox)
        self._body.thread_begin(ThreadType.th_orientation)

        self.__class_logger.log("Waiting for threads starting...", 1)
        sleep(2)
        while True:
            self.__go_on()
            self.target = self._stack.pop()
            self.rotate_to_final_g(self._rot_speed, self.target)


        """self.__go_on()
        self.rotate_to_final_g(self._rot_speed, 90)
        self.target = 90
        self.__go_on()
        self.rotate_to_final_g(self._rot_speed, 180)
        self.target = 180
        self.__go_on()
        self.rotate_to_final_g(self._rot_speed, -90)
        self.target = -90
        self.__go_on()
        self.rotate_to_final_g(self._rot_speed, 180)
        self.target = 180
        self.__go_on()
        self.rotate_to_final_g(self._rot_speed, 90)
        self.target = 90
        self.__go_on()"""

    def algorithm_(self):
        self._body.thread_begin(ThreadType.th_prox)
        self._body.thread_begin(ThreadType.th_orientation)

        self.__class_logger.log("Waiting for threads starting...", 1)
        sleep(2)

        self.__go_on()
        while True:
            self.__think()

            self.__go_on()

    def __go_on(self, _was_insert_l=True, _was_insert_r=True):
        self._body.move_forward(self._speed)

        _front = self._body.get_proxF()

        while _front is None or _front > 0.2:
            if self._body.get_gate():
                self.__class_logger.log("!!!FINISH!!!", 0)
                self._body.stop()
                self.expire()
                exit(1)

            _front = self._body.get_proxF()
            _right = self._body.get_proxR()
            _left = self._body.get_proxL()
            _ori = self._body.get_orientation_deg()

            # print("LEFT: {0}, RIGHT: {1}, FRONT: {2}, COMPASS: {3}".format(_left, _right, _front, _ori))

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

    def __go_back(self):
        while not self._queue.is_empty():
            act = self._queue.pop()

            self._body.stop()
            self.align_robot(act)

            print(act)

            self._body.move_forward(self._speed)

            _front = self._body.get_proxF()

            while _front is None or _front > 0.23:
                _front = self._body.get_proxF()

    def __think(self):
        _right = self._body.get_proxR()
        _left = self._body.get_proxL()

        print("STACK: ", self._stack.stack)

        try:
            if _left is None:
                act = self._stack.pop()
                self._queue.push(negate_compass(act))
                if not self.align_robot(act):
                    self.__class_logger.log("ERR", 4)
                return

            if _right is None:
                act = self._stack.pop()
                self._queue.push(negate_compass(act))
                if not self.align_robot(act):
                    self.__class_logger.log("ERR", 4)
                return

        except IndexError as e:
            self.__class_logger.log("[STACK] empty stack!", 4)
            self.expire()
            exit(-1)

    def rotate_to_final_g(self, vel, final_g):
        """
           Funzione rotate che permette di far ruotare il robot fino a che non raggiunge final_g
        """
        self._body.stop()
        init_g = self._body.get_orientation_deg()
        print("FINAL_G: ", final_g)
        print("INIT_G: ", init_g)
        sleep(3)
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)
        self.__do_rotation(vel=vel, c=c, degrees=abs(degrees), final_g=final_g)
        self._body.stop()

    # CONTROLLER
    def rotate_degrees(self, vel, c: Clockwise, degrees):
        """
            Funzione rotate che permette di far ruotare il robot di degrees gradi
        """
        self._body.stop()
        init_g = self._body.get_orientation_deg()
        final_g = self.compute_final_g(c, init_g, degrees)
        self.__do_rotation(vel=vel, c=c, degrees=abs(degrees), final_g=final_g)
        self._body.stop()

    # CONTROLLER
    def __do_rotation(self, vel, c: Clockwise, degrees, final_g):
        self._body.stop()
        degrees = abs(degrees)
        print("final_g: ", round_v(final_g))
        if DEBUG:
            sleep(5)
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
        init_g = self._body.get_orientation_deg()
        self.__class_logger.log(f"init_g: {init_g}", 3)
        prev_g = init_g
        performed_deg = 0.0
        delta = 0.8
        stop = False
        it_is_rotating = False  # it is not rotating

        while not stop:
            curr_g = self._body.get_orientation_deg()
            self.__class_logger.log(f"curr_ggg: {curr_g}", 3)
            if it_is_rotating:
                performed_deg_temp = self.compute_performed_degrees(c, init_g, curr_g)
                if performed_deg_temp > 180:
                    continue
                performed_deg = performed_deg_temp

            self.__class_logger.log(
                f"[init_g, curr_g, degrees] = [{round_v(init_g)}, {round_v(curr_g)}, {round_v(degrees)}]")
            self.__class_logger.log(
                f"[Performed deg, Round] = [{round_v(performed_deg)}, {round_v(int(performed_deg / 360))}]")
            if degrees - delta < performed_deg < degrees + delta:
                self._body.stop()
                it_is_rotating = False
                self.__class_logger.log("Maybe the orientation is correct ...")
                last_sampled_g = prev_g
                achieved = True  # Problema: se il robot ha slittato i gradi raggiunti
                # non sono giusti e quindi serve sempre il check dell'orientation
                return achieved, init_g, last_sampled_g, performed_deg, degrees

            if performed_deg > degrees + delta:
                self._body.stop()
                it_is_rotating = False
                self.__class_logger.log("Error: performed degrees exceeded the target")
                last_sampled_g = prev_g
                achieved = False  # error, serve il check dell'orientation
                return achieved, init_g, last_sampled_g, performed_deg, degrees

            if c == Clockwise.RIGHT:
                self._body.turn(vel, -vel)
                it_is_rotating = True
            elif c == Clockwise.LEFT:
                self._body.turn(-vel, vel)
                it_is_rotating = True

    # CONTROLLER
    def check_orientation(self, final_g, delta=2):

        self.__class_logger.log("Checking if the orientation is correct ...")

        curr_g = self._body.get_orientation_deg()
        # delta = 2
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

    # CONTROLLER
    def adjust_orientation(self, final_g):
        self._body.stop()
        ok = False
        it = 0
        max_attempts = 5
        while not ok and it < max_attempts:
            curr_g = self._body.get_orientation_deg()
            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)
            print(f"Adjusting orientation, attempts: {it + 1} / {max_attempts}")
            print(f"[Degrees_to_do, curr_g, final_g] = [{round_v(degrees)}, {round_v(curr_g)}, {round_v(final_g)}]")
            if DEBUG:
                sleep(8)
            if abs(degrees) < 6:
                self.__rotate(0.25, c, abs(degrees))
            else:
                self.__rotate(45 * pi / 180, c, abs(degrees))
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

    def compute_performed_degrees(self, c, init_g, curr_g):
        """Calcola l'angolo tra init_g e curr_g che il robot ha eseguito in base al senso di rotazione"""

        if init_g == curr_g:
            return 0
        # Trasformo gli angoli compresi tra [-180,180] ai corrispondenti angoli tra [0,360]
        init_g_360 = normalize_angle(init_g, 0)
        curr_g_360 = normalize_angle(curr_g, 0)

        # Calcolo la differenza
        first_angle = curr_g_360 - init_g_360
        print(first_angle)
        second_angle = -1 * first_angle / abs(first_angle) * (360 - abs(first_angle))
        print(second_angle)
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
            self.__class_logger.log(f"Ruotare in senso orario (RIGHT) di {abs(round_v(smallest))} gradi")
            c = Clockwise.RIGHT
        else:
            self.__class_logger.log(f"Ruotare in senso antirario (LEFT) di {abs(round_v(smallest))} gradi")
            c = Clockwise.LEFT

        return smallest, c


'''
    def align_robot(self, target: None | float = None, timeout: float = 70.5) -> bool:
        """Align robot to target.
        #WARNING: To work it needs the 'orientation' thread to be alive.

        #PARAMS -> target: None | float.
                           Target to align robot, if it's None, the '__detect_target()'
                           method will be invoked to calculate the nearest correct alignment.
                           May not locate the correct angle.
                -> timeout: float.
                           Do rotation until timeout is not reached.
                           It's set to 5.5 seconds by default.
                           With rotation speed == 3, the robot need
                           4.5 second to complete 180 degree of rotation.
                           If the rotation speed changes, the timeout must be adjusted.


        #RETURN: bool. True if alignment is reached else False.

        Blocking method until the target or timeout are reached.
        """
        self.__class_logger.log("Alignment to: {0} degrees".format(target), -1)

        ori = self._body.get_orientation_deg()

        if target is None:
            target = detect_target(ori)
        executed = False

        start_time = time()

        while ori > (target + self._rot_speed + 1) or ori < (target - self._rot_speed - 1):
            ori = self._body.get_orientation_deg()
            if not executed:
                if short_way(ori, target):
                    self._body.turn(self._rot_speed, -self._rot_speed)
                    executed = True
                else:
                    self._body.turn(-self._rot_speed, self._rot_speed)
                    executed = True

            if time() > start_time + timeout:
                self._body.stop()
                return False

        self._body.stop()
        return True

'''
