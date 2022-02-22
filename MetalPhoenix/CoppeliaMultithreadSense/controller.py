from time import sleep, time

from physical_body import PhysicalBody, ThreadType
from utility import short_way, detect_target, StdoutLogger, Compass, \
                    LIFOStack, FIFOStack, normalize_compass, negate_compass


class Controller:
    def __init__(self):
        self.__class_logger = StdoutLogger(class_name="Controller", color="cyan")

        self._body = PhysicalBody()
        self._stack = LIFOStack()
        self._queue = LIFOStack()

        self._speed = 5
        self._rot_speed = 1.5

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

    def expire(self):
        self._body.safe_exit()

    def algorithm(self):
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

        while _front is None or _front > 0.23:
            if self._body.get_gate():
                self.__class_logger.log("!!!FINISH!!!", 0)
                self._body.stop()
                self.expire()
                exit(1)

            _front = self._body.get_proxF()
            _right = self._body.get_proxR()
            _left = self._body.get_proxL()
            _ori = self._body.get_orientation_deg()

            print("LEFT: {0}, RIGHT: {1}, FRONT: {2}, COMPASS: {3}".format(_left, _right, _front, _ori))

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

