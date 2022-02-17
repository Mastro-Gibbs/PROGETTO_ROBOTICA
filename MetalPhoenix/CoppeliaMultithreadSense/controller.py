from time import sleep, time

from physical_body import PhysicalBody, ThreadType
from utility import short_way, detect_target, StdoutLogger, Compass, Stack


class Controller:
    def __init__(self):
        self.__class_logger = StdoutLogger(class_name="Controller", color="cyan")

        self.body = PhysicalBody()
        self.stack = Stack()

        self._speed = 8
        self._rot_speed = 10

        # GET ROBOT INIT ORIENTATION, WAKE UP ORIENTATION THREAD
        self.body.thread_begin(ThreadType.th_orientation)

        self._start_orientation = None
        while self._start_orientation is None:
            self._start_orientation = self.body.get_orientation_deg()
            sleep(0.01)

        self.__class_logger.log("Start orientation: {0} degrees".format(self._start_orientation), -1)

        if not self.align_robot(target=Compass.NORD):
            self.__class_logger.log("Alignment fail!", 4)

        self.body.thread_kill(ThreadType.th_orientation)

        sleep(5)

    def align_robot(self, target: None | float = None, timeout: float = 5.5) -> bool:
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

        ori = self.body.get_orientation_deg()

        if target is None:
            target = detect_target(ori)
        executed = False

        start_time = time()

        while ori > (target + self._rot_speed) or ori < (target - self._rot_speed):
            ori = self.body.get_orientation_deg()
            if not executed:
                if short_way(ori, target):
                    self.body.turn(self._rot_speed, -self._rot_speed)
                    executed = True
                else:
                    self.body.turn(-self._rot_speed, self._rot_speed)
                    executed = True

            if time() > start_time + timeout:
                self.body.stop()
                return False

        self.body.stop()
        return True

    def algorithm(self):
        pass

    def __go_on(self):
        _front = None
        _left = None
        _right = None
        _ori = None
        _normalized_ori = None

        self.body.thread_begin(ThreadType.th_proxF)
        self.body.thread_begin(ThreadType.th_proxL)
        self.body.thread_begin(ThreadType.th_proxR)
        self.body.thread_begin(ThreadType.th_orientation)

        while _front is None or _front > 0.15:
            _front = self.body.get_proxF()
            _left = self.body.get_proxL()
            _right = self.body.get_proxR()
            _ori = self.body.get_orientation_deg()
            _normalized_ori = detect_target(_ori)

            if _left > 0.30:
                pass
                # self.stack.push()

        self.body.thread_kill(ThreadType.th_proxF)
        self.body.thread_kill(ThreadType.th_proxL)
        self.body.thread_kill(ThreadType.th_proxR)
        self.body.thread_kill(ThreadType.th_orientation)


