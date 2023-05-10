import json
import random
import time

from math import pi

from lib.libctrl.remote_serial import RemoteController
from lib.librd.redisdata import ControllerData, RemoteControllerData

from lib.libctrl.utility import Clockwise
from lib.libctrl.utility import f_r_l_b_to_compass
from lib.libctrl.utility import negate_compass, detect_target
from lib.libctrl.utility import Logger, CFG
from lib.libctrl.utility import round_v, normalize_angle
from lib.libctrl.utility import make, FRLB

from lib.libctrl.tree import Node, Type, DIRECTION

from lib.libctrl.enums import Command, Position, Mode, State
from lib.libctrl.enums import Color, STDOUTDecor

from cluster.maze import Maze
from cluster.machine import Machine
from cluster.rotation_factory import RotationFactory

from redis import Redis
from redis.client import PubSubWorkerThread
from redis.exceptions import ConnectionError as RedisConnError


class ControllerException(Exception):
    pass


class Controller:
    _ITERATION: int   = 0
    _INIT_TIME: float = time.time()

    _CONTROLLER_DATA = CFG.robot_conf_data()
    _LOGGER_DATA     = CFG.logger_data()

    _INTELLIGENCE  = _CONTROLLER_DATA["INTELLIGENCE"]
    _SAFE_DISTANCE = _CONTROLLER_DATA['SAFE_DIST']
    _GATE_DISTANCE = _CONTROLLER_DATA['GATE_DIST']
    _ROTATION_MAX_ATTEMPTS = _CONTROLLER_DATA['MAX_ATTEMPTS']

    # ***************************** INSTANCE MANAGEMENT SECTION ******************************* #
    #  FILL IT                                                                                  #
    #                                                                                           #
    #                                                                                           #
    # ***************************************************************************************** #

    # done
    def __init__(self):
        self.__logger = Logger('Controller', self._LOGGER_DATA["SEVERITY"], Color.CYAN)
        self.__logger.set_logfile(self._LOGGER_DATA["CLOGFILE"])

        try:
            self.__redis_message_handler = None
            self.__redis = Redis(host=ControllerData.Connection.Host, port=ControllerData.Connection.Port,
                                 decode_responses=True)
            self.__redis.flushall()
            self.__pubsub = self.__redis.pubsub()
            self.__pubsub.psubscribe(**{ControllerData.Topic.Body: self.__on_message})
            self.__pubsub.psubscribe(**{ControllerData.Topic.Remote: self.__on_remote})
        except RedisConnError or OSError or ConnectionRefusedError:
            raise ControllerException(f'Unable to connect to redis server at: '
                                      f'{ControllerData.Connection.Host}:{ControllerData.Connection.Port}')

        self.__remote: RemoteController          = RemoteController()
        self.__rotation_factory: RotationFactory = RotationFactory()

        self.__machine: Machine = Machine()
        self.__maze: Maze       = Maze()

        self.__prev_action      = None

    # done
    def begin(self) -> None:
        self.__logger.log('Initializing controller\n', Color.YELLOW)

        self.__redis_message_handler: PubSubWorkerThread = self.__pubsub.run_in_thread(sleep_time=0.01)
        self.__execute_motor(Command.STOP)

        self.__logger.log(f'Redis thread:         detached', Color.GRAY)

        self.__rotation_factory.attach_rotation_callback(
            self.remote_rotate
            if RemoteControllerData.is_enabled
            else
            self.mpu_rotate
        )

        self.__logger.log(f'Rotation callback:    {"remote_rotate" if RemoteControllerData.is_enabled else "mpu_rotate"}', Color.GRAY)

        self.__rotation_factory.attach_orientation_callback(
            None
            if RemoteControllerData.is_enabled
            else
            ControllerData.Machine.orientation()
        )

        self.__logger.log(f'Orientation callback: {"Built-in" if RemoteControllerData.is_enabled else "ControllerData.Machine.orientation()"}', Color.GRAY)

        if RemoteControllerData.is_enabled:
            self.__remote.begin()
            self.__logger.log(f'Remote controller:    ready', Color.GRAY)

        self.__logger.log('Controller initialized\n', Color.GREEN, newline=True)

        self.__logger.log(f'Declared constants', Color.YELLOW)
        self.__logger.log(f'Safe distance:        {self._SAFE_DISTANCE}', Color.GRAY)
        self.__logger.log(f'Gate distance:        {self._GATE_DISTANCE}', Color.GRAY)
        self.__logger.log(f'Machine speed:        {self.__machine.speed}', Color.GRAY)
        self.__logger.log(f'Junction time:        {self.__machine.junction_time}', Color.GRAY)
        self.__logger.log(f'Machine rot speed:    {self.__machine.rot_speed}', Color.GRAY)
        self.__logger.log(f'Priority list:        {self.__machine.priority}\n', Color.GRAY)

    # done
    def stop(self) -> None:
        print()
        self.__logger.log(f'Arresting controller\n', Color.YELLOW, newline=True)

        if self.__maze.analysis():
            self.__logger.log(f'Maze analysis:        done', Color.GRAY)

        self.__execute_motor(Command.STOP)
        self.__new_led(False, True, None, True)
        self.__new_buzzer(False, True)

        self.__redis_message_handler.stop()
        self.__redis.close()

        self.__logger.log(f'Redis thread:         buried', Color.GRAY)
        self.__logger.log(f'Redis connection:     closed', Color.GRAY)

        if RemoteControllerData.is_enabled:
            msg = self.__remote.stop()
            self.__logger.log(f'{msg}', Color.GRAY)
            self.__logger.log(f'Remote controller:    stopped', Color.GRAY)

        self.__logger.log('Controller arrested!', Color.RED, newline=True)

    #                                                                                           #
    #                                                                                           #
    # **************************** END INSTANCE MANAGEMENT SECTION **************************** #

    # ************************************* MISC SECTION ************************************** #
    #  FILL IT                                                                                  #
    #                                                                                           #
    #                                                                                           #
    # ***************************************************************************************** #

    # done
    # Check if maze was solved.
    def goal_reached(self) -> bool:
        return ControllerData.Machine.goal()

    # done
    # ending animation if maze were solved
    def ending_animation(self) -> None:
        if self.goal_reached():
            self.__logger.log('Maze finished', Color.GREEN, newline=True, italic=True, blink=True)
            self.__new_led(status=True, emit=True)

            for i in range(0, 5, 1):
                self.__new_buzzer(status=True, emit=True)
                time.sleep(0.25)
                self.__new_buzzer(status=False, emit=True)
                time.sleep(0.25)

        self.__new_led(False, True, None, True)

    # done
    # Updater controller configuration
    def update_config(self) -> None:
        self._CONTROLLER_DATA = CFG.robot_conf_data()

        self.__machine.update()

        self._ROTATION_MAX_ATTEMPTS = self._CONTROLLER_DATA["MAX_ATTEMPTS"]
        self._SAFE_DISTANCE         = self._CONTROLLER_DATA["SAFE_DIST"]
        self._GATE_DISTANCE         = self._CONTROLLER_DATA["GATE_DIST"]

        self.__logger.log(f'Updated constants', Color.YELLOW, newline=True)
        self.__logger.log(f'Safe distance:        {self._SAFE_DISTANCE}', Color.GRAY)
        self.__logger.log(f'Gate distance:        {self._GATE_DISTANCE}', Color.GRAY)
        self.__logger.log(f'Machine speed:        {self.__machine.speed}', Color.GRAY)
        self.__logger.log(f'Junction time:        {self.__machine.junction_time}', Color.GRAY)
        self.__logger.log(f'Machine rot speed:    {self.__machine.rot_speed}', Color.GRAY)
        self.__logger.log(f'Priority list:        {self.__machine.priority}\n', Color.GRAY)

    # todo
    # Send main commands
    def __do_action(self, com_action):
        action = com_action[0]

        if action == Command.START:
            self.__execute_motor(Command.STOP)

            self.__new_led(status=True, emit=True)
            self.__new_buzzer(status=True, emit=True)
            time.sleep(0.5)
            self.__new_led(False, True, None, True)
            self.__new_buzzer(status=False, emit=True)

        # Stop
        elif action == Command.STOP:
            self.__execute_motor(Command.STOP)
            self.__machine.state = State.STOPPED

        # Go on
        elif action == Command.RUN:
            self.__execute_motor(Command.RUN)
            self.__machine.state = State.RUNNING

        # Go to Junction
        elif action == Command.GO_TO_JUNCTION:
            while ControllerData.Machine.left() <= self._GATE_DISTANCE and ControllerData.Machine.right() <= self._GATE_DISTANCE:
                self.__execute_motor(Command.RUN)
                time.sleep(0.005)

            time.sleep(self.__machine.junction_time)

            self.__execute_motor(Command.STOP)
            self.__machine.state = State.SENSING
            self.__machine.position = Position.NODE


        elif action == Command.ROTATE:
            self.__machine.state = State.ROTATING
            self.__execute_motor(Command.STOP)
            self.__rotation_factory.rotate(make.tuple(self.__machine.rot_speed, com_action[1]))
            self.__execute_motor(Command.STOP)

    #                                                                                           #
    #                                                                                           #
    # ********************************** END MISC SECTION ************************************* #

    # ************************************* REDIS SECTION ************************************* #
    #                                       DONE                                                #
    # @function                                                                                 #
    # '__on_message' -> redis builtin thread body/scope                                         #
    #                 read value from redis, set up instance vars.                              #
    #                                                                                           #
    # '__send_command' -> write on redis db.                                                    #
    #                 @param _cmd instance of RCMD: depending on param type send key and publish#
    #                 @param _val -> value to send                                              #
    #                                                                                           #
    # ***************************************************************************************** #

    # DONE
    # callback to rc controller
    def __on_remote(self, msg):
        if RemoteControllerData.is_enabled:
            _key = msg['data']
            _value = self.__redis.get(_key)

            if _key == RemoteControllerData.Key.RC:
                data = json.loads(_value)
                RemoteControllerData.on_values(data['rc_cmd'], data['rc_spd'])

            if RemoteControllerData.is_engaged and RemoteControllerData.is_done:
                self.__new_buzzer(True, True)
                time.sleep(0.3)
                self.__new_buzzer(False, True)

                self.__new_led(False, True, None, True)

                self.__remote.dismiss()
                RemoteControllerData.engaged(False)
                ControllerData.Machine.set_ready(1)

    # DONE
    # callback receiver
    def __on_message(self, msg):
        _key = msg['data']
        _message = self.__redis.get(_key)

        ControllerData.Machine.on_values(_message)

    # DONE
    # sender method
    def __send_command(self, _cmd) -> None:
        if _cmd == ControllerData.Command.Motor and ControllerData.Motor.changed:
            self.__redis.set(ControllerData.Key.Motor, ControllerData.Motor.values)
            self.__redis.publish(ControllerData.Topic.Controller, ControllerData.Key.Motor)

        elif _cmd == ControllerData.Command.Led and ControllerData.Led.changed:
            self.__redis.set(ControllerData.Key.Led, ControllerData.Led.leds())
            self.__redis.publish(ControllerData.Topic.Controller, ControllerData.Key.Led)

        elif _cmd == ControllerData.Command.Buzzer and ControllerData.Buzzer.changed:
            self.__redis.set(ControllerData.Key.Buzzer, ControllerData.Buzzer.status())
            self.__redis.publish(ControllerData.Topic.Controller, ControllerData.Key.Buzzer)

    # DONE
    def __execute_motor(self, cmd):
        if cmd == Command.STOP:
            self.__new_motor_values(0, 0, 0, 0, True)

        elif cmd == Command.RUN:
            self.__new_motor_values(-self.__machine.speed,
                                    -self.__machine.speed,
                                    -self.__machine.speed,
                                    -self.__machine.speed,
                                    True)

    # DONE
    def __new_motor_values(self, rum, lum, rlm, llm, emit: bool = False):
        ControllerData.Motor.on_values(rum, lum, rlm, llm)

        if emit and ControllerData.Motor.changed:
            self.__send_command(ControllerData.Command.Motor)

    # DONE
    def __new_led(self, status: bool, arrow: bool = False, frlb: FRLB = None, emit: bool = False):
        ControllerData.Led.on_arrow(arrow, frlb)
        ControllerData.Led.set(status)

        if emit:
            self.__send_command(ControllerData.Command.Led)

    # DONE
    def __new_buzzer(self, status: bool, emit: bool = False):
        ControllerData.Buzzer.set(status)

        if emit:
            self.__send_command(ControllerData.Command.Buzzer)

    #                                                                                           #
    #                                                                                           #
    # *********************************** END REDIS SECTION *********************************** #

    # *************************************** RC SECTION ************************************** #
    #                                         DONE                                              #
    #                                                                                           #
    #                                                                                           #
    #                                                                                           #
    # ***************************************************************************************** #

    # DONE
    def wakeup_remote(self, frlb: FRLB) -> None:
        self.__new_buzzer(True, True)
        time.sleep(0.3)
        self.__new_buzzer(False, True)

        self.__new_led(True, True, frlb, True)

        RemoteControllerData.engaged(True)
        self.__remote.allow()
        ControllerData.Machine.set_ready(0)

    #                                                                                           #
    #                                                                                           #
    # ************************************ END RC SECTION ************************************* #

    # ************************************* PURE ALGORITHM SECTION ************************************* #
    #                                                                                                    #
    # Tree step.                                                                                         #
    # On each loop invoke algorithm.                                                                     #
    # Algorithm will call 'update_tree'. This method will update tree size (add nodes).                  #
    # Method 'control_policy' will make choices.                                                         #
    # '__verify_gate' check if there is a gate.                                                          #
    #                                                                                                    #
    # ************************************************************************************************** #

    # TODO
    # Algorithm entry
    def algorithm(self) -> bool:
        if self.virt_body_ready():
            self._INIT_TIME = time.time()

            self.__logger.log(f'New algorithm iteration -> #{self._ITERATION}', Color.GREEN, newline=True)
            self._ITERATION += 1

            # THINK
            actions, observed_actions = self.__control_policy()
            selected_action = self.__decision_making_policy(observed_actions)
            command, action = selected_action

            self.__logger.log(f'Current left:         {ControllerData.Machine.left()}', Color.GRAY)
            self.__logger.log(f'Current front:        {ControllerData.Machine.front()}', Color.GRAY)
            self.__logger.log(f'Current right:        {ControllerData.Machine.right()}', Color.GRAY)
            self.__logger.log(f'Current orientation:  {self.__rotation_factory.value}', Color.GRAY)
            self.__logger.log(f'Detected actions:     {action}', Color.GRAY)
            self.__logger.log(f'Detected commands:    {observed_actions}', Color.GRAY)
            self.__logger.log(f'Selected command:     {selected_action}', Color.GRAY)
            self.__logger.log(f'Machine state:        {self.__machine.state}', Color.GRAY)
            self.__logger.log(f'Machine mode:         {self.__machine.mode}', Color.GRAY)
            self.__logger.log(f'Machine position:     {self.__machine.position}', Color.GRAY)
            self.__logger.log(f'VirtualBody command:  {command}', Color.GRAY)

            # tree update
            self.__update_tree(actions, action)

            # ACT
            self.__do_action(selected_action)

            if self.__prev_action != action:
                self.__maze.performed_commands.append(action)
                if action in self.__machine.priority:
                    self.__maze.trajectory.append(action)
                self.__prev_action = action

            return self.goal_reached()

        else:
            _curr_time = time.time()
            self.__logger.log('Algorithm locked, waiting for VirtualBody..' + Color.WHITE.value +
                              f' (time spent: {round(float(_curr_time - self._INIT_TIME), 1)}s)' +
                              STDOUTDecor.DEFAULT.value, Color.RED, rewritable=True)
            time.sleep(0.1)

    # DONE
    # Tree updater
    def __update_tree(self, actions, action_chosen) -> None:
        if not self.__machine.state == State.SENSING:
            return

        if self.__machine.mode == Mode.EXPLORING:
            nodes: list = list()
            for action in actions:
                dict_ = f_r_l_b_to_compass(self.__rotation_factory.value)
                if dict_["FRONT"] == action:
                    node = Node("M_" + self.__maze.tree.generate_node_id(), action)
                    self.__maze.tree.append(node, DIRECTION.MID)
                    self.__maze.tree.regress()
                    self.__maze.incr_node_count()
                    nodes.append('New Mid')
                if dict_["LEFT"] == action:
                    node = Node("L_" + self.__maze.tree.generate_node_id(), action)
                    self.__maze.tree.append(node, DIRECTION.LEFT)
                    self.__maze.tree.regress()
                    self.__maze.incr_node_count()
                    nodes.append('New Left')
                if dict_["RIGHT"] == action:
                    node = Node("R_" + self.__maze.tree.generate_node_id(), action)
                    self.__maze.tree.append(node, DIRECTION.RIGHT)
                    self.__maze.tree.regress()
                    self.__maze.incr_node_count()
                    nodes.append('New Right')

            if len(nodes) != 0:
                self.__logger.log(f'Added new nodes:      {nodes}', Color.GRAY)

            self.__maze.tree.current.set_type(Type.EXPLORED)

            dict_ = f_r_l_b_to_compass(self.__rotation_factory.value)
            if dict_["FRONT"] == action_chosen:
                self.__maze.tree.set_current(self.__maze.tree.current.mid)
            elif dict_["LEFT"] == action_chosen:
                self.__maze.tree.set_current(self.__maze.tree.current.left)
            elif dict_["RIGHT"] == action_chosen:
                self.__maze.tree.set_current(self.__maze.tree.current.right)

        elif self.__machine.mode == Mode.ESCAPING:
            cur = None

            # The node is a leaf
            if self.__maze.tree.current.is_leaf:
                self.__maze.tree.current.set_type(Type.DEAD_END)
                self.__maze.incr_node_count()
                self.__logger.log(f'Dead end node:        {self.__maze.tree.current}', Color.GREEN)

                cur = self.__maze.tree.current.parent

            # The children are all DEAD END
            elif ((self.__maze.tree.current.has_left and self.__maze.tree.current.left.type == Type.DEAD_END) or
                  self.__maze.tree.current.left is None) and \
                    ((self.__maze.tree.current.has_right and self.__maze.tree.current.right.type == Type.DEAD_END)
                     or self.__maze.tree.current.right is None) and \
                    ((self.__maze.tree.current.has_mid and self.__maze.tree.current.mid.type == Type.DEAD_END)
                     or self.__maze.tree.current.mid is None):
                self.__maze.tree.current.set_type(Type.DEAD_END)
                self.__maze.incr_dead_end()
                self.__logger.log(f'Dead end nodes chain: {self.__maze.tree.current}', Color.GREEN)

                cur = self.__maze.tree.current.parent

            else:
                self.__logger.log("No leaf or DE children", Color.YELLOW, italic=True)

                if self.__maze.tree.current.has_left and self.__maze.tree.current.left.action == action_chosen:
                    cur = self.__maze.tree.current.left
                elif self.__maze.tree.current.has_mid and self.__maze.tree.current.mid.action == action_chosen:
                    cur = self.__maze.tree.current.mid
                elif self.__maze.tree.current.has_right and self.__maze.tree.current.right.action == action_chosen:
                    cur = self.__maze.tree.current.right
                else:
                    self.__logger.log("Tree exception raised", Color.DARKRED, True, True)
                    raise ControllerException('Tree exception raised')

            self.__maze.tree.set_current(cur)

    # DONE
    # Action maker
    def __control_policy(self) -> tuple:
        """
        Accordingly to the values of the sensors, the state of the robot and the tree of the maze,
        it returns a tuple of two list.
        The first is a list of actions (elements of Compass type), it is used only to update the tree.
        The second list, com_actions that is the abbreviation of command_actions,
        is a list of lists of two elements type: Command and Compass.
        It contains a set of couples [command, action] that the robot can perform, but only one of these can be executed.
        """

        actions = list()
        com_actions = list(list())

        left = ControllerData.Machine.left()
        front = ControllerData.Machine.front()
        right = ControllerData.Machine.right()
        ori = self.__rotation_factory.value

        if self.__machine.state == State.STARTING and self.__machine.position == Position.UNKNOWN:
            if front is None:
                # Ho il muro dietro (dato che nei vincoli non posso mettere il robot
                # in una giunzione dove ci sono 4 direzioni/strade libere)
                if (left is not None and right is not None) or (left is None and right is None):
                    self.__machine.state = State.STARTING
                    self.__machine.position = Position.INITIAL
                    com_actions.insert(0, [Command.START, None])
                else:
                    # Muro a sinistra
                    if left is not None:
                        action = detect_target(detect_target(ori) - 90)
                        com_actions.insert(0, [Command.ROTATE, action])

                    # Muro a destra
                    elif right is not None:
                        action = detect_target(detect_target(ori) + 90)
                        com_actions.insert(0, [Command.ROTATE, action])

            # Faccio 180° per avere il muro dietro
            elif front is not None:
                if left is None or right is None:
                    action = negate_compass(detect_target(ori))
                    com_actions.insert(0, [Command.ROTATE, action])

                elif left is not None and right is not None:
                    action = negate_compass(detect_target(ori))
                    com_actions.insert(0, [Command.ROTATE, action])
                    self.attempts_to_unstuck = 1

            # In questo caso ho sempre il muro dietro

        elif self.__machine.state == State.STARTING and self.__machine.position == Position.INITIAL:

            if front is None:
                if left is not None and right is not None:
                    self.__machine.state = State.SENSING
                    self.__machine.position = Position.EDGE
                    self.attempts_to_unstuck = 0

                elif left is None or right is None:
                    self.__machine.state = State.SENSING
                    self.__machine.position = Position.NODE
                    self.attempts_to_unstuck = 0

            elif front is not None:
                if left is None or right is None:
                    self.__machine.state = State.SENSING
                    self.__machine.position = Position.EDGE
                    self.attempts_to_unstuck = 0

                elif left is not None and right is not None:
                    if self.attempts_to_unstuck == 1:
                        self.__logger.log("Robot in stuck. Cannot solve the maze.", Color.RED, True, True)
                        raise ControllerException("Robot in stuck. Cannot solve the maze.")

            com_actions.insert(0, [Command.START, None])

        elif self.__machine.state == State.ROTATING:  # the robot has already rotated
            if self.__machine.position == Position.UNKNOWN:
                self.__machine.state = State.STARTING
                self.__machine.position = Position.INITIAL
                com_actions.insert(0, [Command.START, None])

            else:
                actions.insert(0, self.__maze.performed_commands[len(self.__maze.performed_commands) - 1])
                com_actions.insert(0, [Command.RUN, detect_target(ori)])

        elif self.__machine.state == State.STOPPED or self.__machine.state == State.SENSING:
            self.__machine.state = State.SENSING

            if self.__machine.mode == Mode.EXPLORING:
                if front is None or front > self._SAFE_DISTANCE:
                    action = f_r_l_b_to_compass(ori)["FRONT"]
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.RUN, action])
                if left is None or left > self._SAFE_DISTANCE:
                    action = f_r_l_b_to_compass(ori)["LEFT"]
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])
                if right is None or right > self._SAFE_DISTANCE:
                    action = f_r_l_b_to_compass(ori)["RIGHT"]
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])

                if not actions:
                    action = negate_compass(self.__maze.tree.current.action)
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])
                    self.__machine.mode = Mode.ESCAPING
                    self.__machine.state = State.SENSING

            elif self.__machine.mode == Mode.ESCAPING:
                if self.__maze.tree.current.left is not None and self.__maze.tree.current.left.type == Type.OBSERVED:
                    action = self.__maze.tree.current.left.action
                    actions.insert(0, action)
                    if action == detect_target(ori):
                        com_actions.insert(0, [Command.RUN, action])
                    else:
                        com_actions.insert(0, [Command.ROTATE, action])
                if self.__maze.tree.current.mid is not None and self.__maze.tree.current.mid.type == Type.OBSERVED:
                    action = self.__maze.tree.current.mid.action
                    actions.insert(0, action)
                    if action == detect_target(ori):
                        com_actions.insert(0, [Command.RUN, action])
                    else:
                        com_actions.insert(0, [Command.ROTATE, action])
                if self.__maze.tree.current.right is not None and self.__maze.tree.current.right.type == Type.OBSERVED:
                    action = self.__maze.tree.current.right.action
                    actions.insert(0, action)
                    if action == detect_target(ori):
                        com_actions.insert(0, [Command.RUN, action])
                    else:
                        com_actions.insert(0, [Command.ROTATE, action])

                if not actions:
                    if (self.__maze.tree.current.left is None or self.__maze.tree.current.left.type == Type.DEAD_END) and \
                            (self.__maze.tree.current.mid is None or self.__maze.tree.current.mid.type == Type.DEAD_END) and \
                            (self.__maze.tree.current.right is None or self.__maze.tree.current.right.type == Type.DEAD_END) and \
                            self.__maze.tree.current.action is None:

                        self.__logger.log("Tree error, robot is stuck", Color.DARKRED, True, True)
                        raise ControllerException('Tree error, robot is stuck')

                    # Coming back, regressing
                    action = negate_compass(self.__maze.tree.current.action)
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])

        elif self.__machine.state == State.RUNNING:

            # Switch da ESCAPING A EXPLORING, messo qui perché se sto in RUNNING l'albero non viene aggiornato
            # in update_tree (per aggiornarlo lo stato deve essere in SENSING).
            # Evito così che vengano aggiunti dei nodi duplicati non voluti.

            if self.__machine.mode == Mode.ESCAPING and self.__maze.tree.current.type == Type.OBSERVED:
                self.__machine.mode = Mode.EXPLORING

            if self.__machine.position == Position.EDGE:
                if (left is not None and left <= 20) or (right is not None and right <= 20):
                    com_actions.insert(0, [Command.GO_TO_JUNCTION, detect_target(ori)])
                elif front is None or front > self._SAFE_DISTANCE:
                    com_actions.insert(0, [Command.RUN, detect_target(ori)])
                elif front <= self._SAFE_DISTANCE:
                    com_actions.insert(0, [Command.STOP, None])

            elif self.__machine.position == Position.NODE:
                if left is not None and right is not None:
                    self.__machine.position = Position.EDGE
                if front is None or front > self._SAFE_DISTANCE:
                    com_actions.insert(0, [Command.RUN, detect_target(ori)])
                elif front <= self._SAFE_DISTANCE:
                    com_actions.insert(0, [Command.STOP, None])

        return actions, com_actions

    # DONE
    def __decision_making_policy(self, com_actions) -> list:
        """ Given a set of actions it decides what action the robot has to perform """

        if len(com_actions) == 1:
            return com_actions[0]

        for direction in self.__machine.priority:
            for com_action in com_actions:
                action = com_action[1]
                if direction == action:
                    return com_action

    # DONE
    @staticmethod
    def virt_body_ready() -> bool:
        return ControllerData.Machine.connection() and ControllerData.Machine.ready()

    #                                                                                                    #
    #                                                                                                    #
    # ************************************ END PURE ALGORITHM SECTION ********************************** #

    # ************************************* ROTATION SECTION ************************************* #
    #                                                                                              #
    # Main method: 'rotate'                                                                        #
    #                                                                                              #
    # ******************************************************************************************** #

    # DONE
    def remote_rotate(self, args: tuple):
        vel, final_g = args

        frlb: FRLB = self.__rotation_factory.compute(int(final_g.value))

        self.wakeup_remote(frlb)

    # TODO
    def mpu_rotate(self, args: tuple):
        """ Rotate function that rotates the robot until it reaches final_g """
        vel, final_g = args

        init_g = self.__rotation_factory.value
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)

        self.__do_rotation(vel, c, degrees, final_g)

    # TODO
    def __do_rotation(self, vel, c: Clockwise, degrees, final_g):
        """
        This method calls __rotate to perform the rotation.
        It also checks the outcome of check_orientation:
        i) If the orientation is correct nothing happens
        ii) If the orientation is not correct it calls adjust_orientation to
            rotate the robot to the correct orientation
        It stops the program when it is not possible to orient correctly the robot when it exceeds the number
        of attempts (Critical case)
        """

        degrees = abs(degrees)
        it = 0

        self.__rotate(vel, c, degrees)

        ok, curr_g, limit_range = self.check_orientation(final_g)

        if not ok:
            ok, it = self.adjust_orientation(final_g)

        # CRITICAL CASE
        if it == self._ROTATION_MAX_ATTEMPTS:
            self.__logger.log(" ** MAX ATTEMPTS REACHED ** ", Color.DARKRED, True, True)
            self.__logger.log(" >>>>  EXITING  <<<< ", Color.DARKRED, italic=True)
            exit(-1)

    # TODO
    def __rotate(self, vel, c: Clockwise, degrees):
        """
        Function that given vel, Clockwise and rotation degrees computes
        the rotation of the Robot around the z axis
        The orientation of the robot must reach the interval [degrees - delta, degrees + delta]
        """
        degrees = abs(degrees)
        init_g = self.__rotation_factory.value

        delta = 0.8
        stop = False
        archived = False
        performed_deg = 0.0

        while not stop:
            curr_g = self.__rotation_factory.value

            if self.__remote is not None:
                self.__new_led(status=True, arrow=True, clockwise=c, emit=True)

                self.__new_buzzer(status=True, emit=True)
                time.sleep(1)
                self.__new_buzzer(status=False, emit=True)

                while not RemoteControllerData.is_rotation_done:
                    time.sleep(0.01)

                self.__new_led(status=False, emit=True)
            else:
                if c == Clockwise.RIGHT:
                    self.__new_motor_values(vel, -vel, vel, -vel, True)
                elif c == Clockwise.LEFT:
                    self.__new_motor_values(-vel, vel, -vel, vel, True)

            performed_deg_temp = self.compute_performed_degrees(c, init_g, curr_g)

            """ 
            Check if there was an unintended move in the opposite direction. 
            The degrees performed in this case would be > 300 and it is not considered (continue)
            """

            if performed_deg_temp > 300:
                continue

            performed_deg = performed_deg_temp

            if degrees - delta < performed_deg < degrees + delta:
                archived = True
                stop = True
            elif performed_deg > degrees + delta:
                archived = False
                stop = True

        self.__execute_motor(Command.STOP)
        return archived, init_g, performed_deg, degrees

    # TODO
    def check_orientation(self, final_g, delta=2):
        """
        This method is used to check if the orientation of the robot is correct
        The orientation of the robot must reach a specific interval according to two cases:
        1) The final_g is 180 or -180 (first if)
        2) Otherwise other intervals (second if)
        """

        self.__logger.log(" ** ORIENTATION CHECKING ** ", Color.GRAY, True, True)

        curr_g = self.__rotation_factory.value

        ok = False
        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if curr_g < limit_g_sx or curr_g > limit_g_dx:
                self.__logger.log(" ~~ perfect ~~ ", Color.GREEN)
                ok = True
            else:
                self.__logger.log(" !! bad orientation !! ", Color.RED)
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= curr_g <= limit_g_sx:
                self.__logger.log(" ~~ perfect ~~ ", Color.GREEN)

                ok = True
            else:
                self.__logger.log(" !! bad orientation !! ", Color.RED)

        self.__logger.log(
            f" >> curr state of the sensors: "
            f"[{round_v(limit_g_sx)}, {round_v(limit_g_dx)}], curr_g: {round_v(curr_g)}", Color.GRAY)

        limit_range = [limit_g_sx, limit_g_dx]
        return ok, curr_g, limit_range

    # TODO
    def adjust_orientation(self, final_g):
        """
        This method is used to adjust the orientation of the robot
        There are a max number of attempts:
        i) If the robot is able to orient himself correctly than the outcome is positive
        ii) If the robot fails, there is an error in adjusting the orientation and attempts stop
        """

        self.__logger.log(" ** ADJUSTING ORIENTATION ** ", Color.GRAY, True, True)

        self.__execute_motor(Command.STOP)

        ok = False
        it = 0

        while not ok and it < self._ROTATION_MAX_ATTEMPTS:
            curr_g = self.__rotation_factory.value

            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)

            self.__logger.log(f" --ATTEMPT: {it + 1} / {self._ROTATION_MAX_ATTEMPTS}", Color.GRAY)
            self.__logger.log(
                f"[Degrees_to_do, curr_g, final_g] = [{round_v(degrees)}, {round_v(curr_g)}, {round_v(final_g)}]",
                Color.GRAY)

            if abs(degrees) < 6:
                self.__rotate(0.25, c, abs(degrees))
            else:
                self.__rotate(45 * pi / 180, c, abs(degrees))

            ok, curr_g, limit_range = self.check_orientation(final_g)
            it += 1

        return ok, it

    # TODO
    @staticmethod
    def compute_performed_degrees(c, init_g, curr_g):
        """
        Computes the angle between init_g and curr_g that the robot performed
        based on the direction of rotation
        """

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

    # TODO
    def best_angle_and_rotation_way(self, init_g, final_g):
        """ Computes the best (minimum) angle between init_g and final_g and how you need to rotate """

        self.__logger.log(" ** BEST ANGLE COMPUTATION ** ", Color.GRAY, True, True)

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
            self.__logger.log(f" >> Rotate clockwise (RIGHT) of {abs(round_v(smallest))}°", Color.GRAY)

            c = Clockwise.RIGHT
        else:
            self.__logger.log(f" >> Rotate anti-clockwise (LEFT) of {abs(round_v(smallest))}°", Color.GRAY)

            c = Clockwise.LEFT

        return smallest, c

    #                                                                                              #
    #                                                                                              #
    # *********************************** END ROTATION SECTION *********************************** #


