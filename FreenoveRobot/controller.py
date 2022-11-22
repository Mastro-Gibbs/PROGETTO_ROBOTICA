import json
import random
import time

from math import pi

from lib.libctrl.remote_serial import RemoteController
from lib.librd.redisdata import ControllerData, RemoteControllerData

from lib.libctrl.utility import Compass, Clockwise
from lib.libctrl.utility import f_r_l_b_to_compass
from lib.libctrl.utility import negate_compass, detect_target
from lib.libctrl.utility import Logger, CFG
from lib.libctrl.utility import round_v, normalize_angle

from lib.libctrl.tree import Tree, Node, Type, DIRECTION

from lib.libctrl.enums import Command, Position, Mode, State
from lib.libctrl.enums import Color, STDOUTDecor

from lib.exit_codes import NOACTIONS, TREEUPDATEERROR

from redis import Redis
from redis.client import PubSubWorkerThread
from redis.exceptions import ConnectionError as RedisConnError


class ControllerException(Exception):
    pass


class Controller:
    _ITERATION: int = 0
    _INIT_TIME: float = time.time()

    _CONTROLLER_DATA = CFG.robot_conf_data()

    _LOGSEVERITY = _CONTROLLER_DATA["SEVERITY"]
    _INTELLIGENCE = _CONTROLLER_DATA["INTELLIGENCE"]
    _SAFE_DISTANCE = _CONTROLLER_DATA['SAFE_DIST']
    _ROTATION_MAX_ATTEMPTS = _CONTROLLER_DATA['MAX_ATTEMPTS']


    # ***************************** INSTANCE MANAGEMENT SECTION ******************************* #
    #  FILL IT                                                                                  #
    #                                                                                           #
    #                                                                                           #
    # ***************************************************************************************** #

    def __init__(self):
        self.__logger = Logger('Controller', Color.CYAN)
        self.__logger.set_logfile(CFG.logger_data()["LOGPATH"])

        try:
            self.__redis_message_handler = None
            self.__redis = Redis(host=ControllerData.Connection.Host, port=ControllerData.Connection.Port, decode_responses=True)
            self.__redis.flushall()
            self.__pubsub = self.__redis.pubsub()
            self.__pubsub.psubscribe(**{ControllerData.Topic.Body: self.__on_message})
        except RedisConnError or OSError or ConnectionRefusedError:
            raise ControllerException(f'Unable to connect to redis server at: '
                                      f'{ControllerData.Connection.Host}:{ControllerData.Connection.Port}')

        self.__remote = RemoteController() if RemoteControllerData.is_enabled else None

        self.__robot_mode: Mode = Mode.EXPLORING
        self.__robot_state: State = State.STARTING
        self.__robot_position: Position = Position.INITIAL
        self.__robot_priority_list = self._CONTROLLER_DATA['PRIORITY_LIST']

        self.__robot_speed = self._CONTROLLER_DATA["SPEED"]
        self.__robot_rotation_speed = self._CONTROLLER_DATA["ROT_SPEED"]

        self.__speed_msec = self.__robot_speed * 0.25 / (self.__robot_speed // 5)
        self.__junction_sim_time = 0.25 / self.__speed_msec

        self.__front_ultrasonic_stored_values: list = list()
        self.__left_ultrasonic_stored_values: list = list()
        self.__right_ultrasonic_stored_values: list = list()

        self.__maze_tree = Tree()
        self.__maze_trajectory = list()
        self.__maze_performed_commands = list()
        self.__prev_action = None

        """ DATA ANALYSIS """
        self.maze_number = 1  # Each maze must have a number to be identified, change this number if the maze changes
        self.maze_name = "Maze" + "_" + str(self.maze_number) + "_" + \
                         (Compass.compass_list_to_concat_string(self.__robot_priority_list)
                          if self._INTELLIGENCE == "low" else "RANDOM")
        self.time_to_solve = 0  # solving time
        self.number_of_nodes = 1
        self.number_of_dead_end = 0

    def virtual_destructor(self) -> None:
        self.__logger.log('Controller Stopped!', Color.GREEN, newline=True, italic=True, underline=True)
        self.__redis_message_handler.stop()
        self.__redis.close()

        if self.__remote is not None:
            self.__remote.stop()

    def begin(self) -> None:
        self.__logger.log('Controller fully initialized', Color.GREEN, newline=True, italic=True)
        self.__new_motor_values(0, 0, 0, 0, emit=True)
        self.__redis_message_handler: PubSubWorkerThread = self.__pubsub.run_in_thread(sleep_time=0.01)

        if self.__remote is not None:
            self.__remote.begin()

    #                                                                                           #
    #                                                                                           #
    # **************************** END INSTANCE MANAGEMENT SECTION **************************** #





    # ************************************* MISC SECTION ************************************** #
    #  FILL IT                                                                                  #
    #                                                                                           #
    #                                                                                           #
    # ***************************************************************************************** #

    # Send main commands
    def __do_action(self, com_action):
        action = com_action[0]

        self.__logger.log(f'Sending action "{action}" to VirtualBody..', Color.GREEN)

        if action == Command.START:
            self.__execute_motor(Command.STOP)

            self.__new_led(status=True, emit=True)
            self.__new_buzzer(status=True, emit=True)
            time.sleep(1)
            self.__new_led(status=False, emit=True)
            self.__new_buzzer(status=False, emit=True)

            return True

        # Stop
        elif action == Command.STOP:
            self.__execute_motor(Command.STOP)
            self.__robot_state = State.STOPPED

            return True

        # Go on
        elif action == Command.RUN:
            self.__execute_motor(Command.RUN)
            self.__robot_state = State.RUNNING

            return True

        # Go to Junction
        elif action == Command.GO_TO_JUNCTION:

            start_time = time.time()
            time_expired = False

            while not time_expired and (ControllerData.Machine.front() is None
                                        or ControllerData.Machine.front() > self._SAFE_DISTANCE):
                self.__execute_motor(Command.RUN)
                if time.time() - start_time >= self.__junction_sim_time:
                    time_expired = True

            self.__execute_motor(Command.STOP)
            self.__robot_state = State.SENSING
            self.__robot_position = Position.JUNCTION

            time.sleep(0.5)

        elif action == Command.ROTATE:
            if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                self.__logger.log(" ** COMMAND ROTATE ** ", Color.GRAY)

            self._state = State.ROTATING
            self.__execute_motor(Command.STOP)
            self.rotate_to_final_g(self.__robot_rotation_speed, com_action[1])
            self.__execute_motor(Command.STOP)

            return True

        else:
            print("ERROR: ACTION NOT RECOGNIZED")
            exit(-1)

    # Check if maze was solved. 
    def goal_reached(self) -> bool:
        return ControllerData.Machine.goal()

    # ending animation if maze were solved
    def ending_animation(self) -> None:
        if ControllerData.Machine.goal():
            self.__logger.log('Maze finished', Color.GREEN, newline=True, italic=True, blink=True)
            self.__new_led(status=True, emit=True)

            for i in range(0, 5, 1):
                self.__new_buzzer(status=True, emit=True)
                time.sleep(0.25)
                self.__new_buzzer(status=False, emit=True)
                time.sleep(0.25)

        self.__new_led(status=False, emit=True)

    # Updater controller configuration
    def update_config(self) -> None:
        self._CONTROLLER_DATA = CFG.robot_conf_data()

        self.__robot_speed = self._CONTROLLER_DATA["SPEED"]
        self.__robot_rotation_speed = self._CONTROLLER_DATA["ROT_SPEED"]
        self.__robot_priority_list = self._CONTROLLER_DATA['PREFERENCE']
        self._ROTATION_MAX_ATTEMPTS = self._CONTROLLER_DATA["MAX_ATTEMPTS"]
        self._SAFE_DISTANCE = self._CONTROLLER_DATA["SAFE_DIST"]
        self._LOGSEVERITY = self._CONTROLLER_DATA["SEVERITY"]

    def intelligent_priority_list(self):
        ...
        print(self.priority_list)

    def write_data_analysis(self):
        priority_list = Compass.compass_list_to_string_comma_sep(self.priority_list)
        CFG.write_data_analysis(self.maze_name,
                                self.time_to_solve,
                                self.__maze_tree.build_tree_dict(),
                                self.number_of_nodes,
                                self.number_of_dead_end,
                                self.__maze_performed_commands,
                                self.__maze_trajectory,
                                self._INTELLIGENCE,
                                priority_list if self._INTELLIGENCE == "low" else "random"
                                )

    #                                                                                           #
    #                                                                                           #
    # ********************************** END MISC SECTION ************************************* #


    # ************************************* REDIS SECTION ************************************* #
    #                                                                                           #
    # @function                                                                                 #
    # '__on_message' -> redis builtin thread body/scope                                         #
    #                 read value from redis, set up instance vars.                              #
    #                                                                                           #
    # '__send_command' -> write on redis db.                                                    #
    #                 @param _cmd instance of RCMD: depending on param type send key and publish#
    #                 @param _val -> value to send                                              #
    #                                                                                           #
    # ***************************************************************************************** #

    # callback receiver
    def __on_message(self, msg):
        _key = msg['data']
        _message: dict = json.loads(self.__redis.get(_key))

        ControllerData.Machine.on_values(_message)

    # sender method
    def __send_command(self, _cmd) -> None: # RESET

        if _cmd == ControllerData.Command.Motor and ControllerData.Motor.changed:
            self.__redis.set(ControllerData.Key.Motor, ControllerData.Motor.values)
            self.__redis.publish(ControllerData.Topic.Controller, ControllerData.Key.Motor)

        elif _cmd == ControllerData.Command.Led and ControllerData.Led.changed:
            self.__redis.set(ControllerData.Key.Led, ControllerData.Led.leds())
            self.__redis.publish(ControllerData.Topic.Controller, ControllerData.Key.Led)

        elif _cmd == ControllerData.Command.Buzzer and ControllerData.Buzzer.changed:
            self.__redis.set(ControllerData.Key.Buzzer, ControllerData.Buzzer.status())
            self.__redis.publish(ControllerData.Topic.Controller, ControllerData.Key.Buzzer)

    def __execute_motor(self, cmd):
        if cmd == Command.STOP:
            self.__new_motor_values(0, 0, 0, 0, True)

        elif cmd == Command.RUN:
            self.__new_motor_values(-self.__robot_speed,
                                    -self.__robot_speed,
                                    -self.__robot_speed,
                                    -self.__robot_speed,
                                    True)

    def __new_motor_values(self, rum, lum, rlm, llm, emit: bool):
        ControllerData.Motor.on_values(rum, lum, rlm, llm)

        if emit:
            self.__send_command(ControllerData.Command.Motor)

    def __new_led(self, status: bool, arrow: bool = False, clockwise: Clockwise = None, emit: bool = False):
        ControllerData.Led.on_arrow(arrow, clockwise)
        ControllerData.Led.set(status)

        if emit:
            self.__send_command(ControllerData.Command.Led)

    def __new_buzzer(self, status: bool, emit: bool = False):
        ControllerData.Buzzer.set(status)

        if emit:
            self.__send_command(ControllerData.Command.Buzzer)

    #                                                                                           #
    #                                                                                           #
    # *********************************** END REDIS SECTION *********************************** #





    # ************************************* PURE ALGORITHM SECTION ************************************* #
    #                                                                                                    #
    # Tree step.                                                                                         #
    # On each loop invoke algorithm.                                                                     #
    # Algorithm will call 'update_tree'. This method will update tree size (add nodes).                  #
    # Method 'control_policy' will make choices.                                                         #
    # '__verify_gate' check if there is a gate.                                                          #
    #                                                                                                    #
    # ************************************************************************************************** #

    # OK
    # Algorithm entry
    def algorithm(self) -> bool:
        if self.__is_algorithm_unlocked():

            if self._ITERATION == 0:
                print()

            if self.goal_reached():
                return True

            self.__logger.log(f'New algorithm iteration -> #{self._ITERATION}', Color.YELLOW, newline=True, underline=True)
            self._ITERATION += 1

            # SENSE
            self.__left_ultrasonic_stored_values.append(ControllerData.Machine.left())
            self.__front_ultrasonic_stored_values.append(ControllerData.Machine.front())
            self.__right_ultrasonic_stored_values.append(ControllerData.Machine.right())

            # THINK
            actions, com_actions = self.__control_policy()
            com_action = self.__decision_making_policy(com_actions)
            command = com_action[0]
            action = com_action[1]

            if Logger.is_loggable(self._LOGSEVERITY, "low"):
                self.__logger.log(f"--MODE: {self.mode}")
                self.__logger.log(f"--ACTIONS: {com_actions}")
                self.__logger.log(f"--ACTION: {com_action}")
                self.__logger.log(f"--CURRENT NODE: {self.__maze_tree.current}")

            # tree update
            self.__update_tree(actions, action)

            if Logger.is_loggable(self._LOGSEVERITY, "low"):
                self.__logger.log("--CURRENT TREE:", Color.GRAY)
                self.__logger.log(f"{self.__maze_tree.build_tree_dict()}", Color.GRAY, noheader=True)
                self.__logger.log(f"--CURRENT NODE: {self.__maze_tree.current}", Color.GRAY, newline=True)
                self.__logger.log(f"--Available actions: {com_actions}", Color.GREEN)

            if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                self.__logger.log(f"--(STATE, POSITION): ({self._state}, {self._position})", Color.GRAY)
                self.__logger.log(f"--Performing action: {com_action}", Color.GRAY)

            if com_action is None:
                self.__logger.log("NO ACTION AVAILABLE!", Color.DARKRED, newline=True)
                self.__logger.log(" >>>>  EXITING  <<<< ", Color.DARKRED, italic=True)
                self.virtual_destructor()
                exit(-1)

            # ACT
            performed = self.__do_action(com_action)

            if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                self.__logger.log(f"--(STATE, POSITION): ({self._state}, {self._position})", Color.GRAY)

            if performed and self.__prev_action != action:
                self.__maze_performed_commands.append(action)
                if action in self.priority_list:
                    self.__maze_trajectory.append(action)
                self.__prev_action = action

            return False

        else:
            _curr_time = time.time()
            self.__logger.log('Algorithm locked, waiting for VirtualBody..' + Color.WHITE.value +
                              f' (time spent: {round(float(_curr_time-self._INIT_TIME), 1)}s)' +
                              STDOUTDecor.DEFAULT.value, Color.RED, italic=True, _stdout=True)
            time.sleep(0.1)

    # OK
    # Tree updater
    def __update_tree(self, actions, action_chosen) -> None:
        if not self._state == State.SENSING:
            return

        if self.mode == Mode.EXPLORING:
            if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                self.__logger.log("*** UPDATING TREE (MODE: EXPLORING) ***", Color.GRAY, newline=True)

            """ 
            Different actions returned by Control Policy.
            In this section are added the nodes of the tree based on the available actions, 
            updating also the current node as EXPLORED 
            """
            for action in actions:
                dict_ = f_r_l_b_to_compass(ControllerData.Machine.z_axis())
                if dict_["FRONT"] == action:
                    node = Node("M_" + self.__maze_tree.generate_node_id(), action)
                    self.__maze_tree.append(node, DIRECTION.MID)
                    self.__maze_tree.regress()
                    self.number_of_nodes += 1
                    if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                        self.__logger.log("ADDED MID", Color.DARKGREEN)
                if dict_["LEFT"] == action:
                    node = Node("L_" + self.__maze_tree.generate_node_id(), action)
                    self.__maze_tree.append(node, DIRECTION.LEFT)
                    self.__maze_tree.regress()
                    self.number_of_nodes += 1
                    if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                        self.__logger.log("ADDED LEFT", Color.DARKGREEN)
                if dict_["RIGHT"] == action:
                    node = Node("R_" + self.__maze_tree.generate_node_id(), action)
                    self.__maze_tree.append(node, DIRECTION.RIGHT)
                    self.__maze_tree.regress()
                    self.number_of_nodes += 1
                    if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                        self.__logger.log("ADDED RIGHT", Color.DARKGREEN)

            self.__maze_tree.current.set_type(Type.EXPLORED)

            """ 
            Only one action that has been decided by DMP. 
            In this section it is updated the current node of the tree based on the action chosen 
            """
            dict_ = f_r_l_b_to_compass(ControllerData.Machine.z_axis())
            if dict_["FRONT"] == action_chosen:
                self.__maze_tree.set_current(self.__maze_tree.current.mid)
            elif dict_["LEFT"] == action_chosen:
                self.__maze_tree.set_current(self.__maze_tree.current.left)
            elif dict_["RIGHT"] == action_chosen:
                self.__maze_tree.set_current(self.__maze_tree.current.right)

        elif self.mode == Mode.ESCAPING:
            if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                self.__logger.log("*** UPDATING TREE (MODE: ESCAPING) ***", Color.GRAY, newline=True)

            """ 
            In this section it is updated the type property of the current node accordingly if
            the current node is a leaf or has children that are all dead end, otherwise it is updated the current node
            """

            cur = None

            # The node is a leaf
            if self.__maze_tree.current.is_leaf:
                self.__maze_tree.current.set_type(Type.DEAD_END)
                self.number_of_dead_end += 1
                if Logger.is_loggable(self._LOGSEVERITY, "low"):
                    self.__logger.log("*** DEAD END NODE DETECTED ***", Color.GREEN)
                    self.__logger.log(" >>>> REGRESSION <<<< ", Color.YELLOW, newline=True)
                    self.__logger.log(f" --CURRENT NODE: {self.__maze_tree.current}", Color.YELLOW)
                    self.__logger.log(f" --PARENT NODE: {self.__maze_tree.current.parent}", Color.YELLOW)

                cur = self.__maze_tree.current.parent

            # The children are all DEAD END
            elif ((self.__maze_tree.current.has_left and self.__maze_tree.current.left.type == Type.DEAD_END) or
                  self.__maze_tree.current.left is None) and \
                    ((self.__maze_tree.current.has_right and self.__maze_tree.current.right.type == Type.DEAD_END)
                     or self.__maze_tree.current.right is None) and \
                    ((self.__maze_tree.current.has_mid and self.__maze_tree.current.mid.type == Type.DEAD_END)
                     or self.__maze_tree.current.mid is None):
                self.__maze_tree.current.set_type(Type.DEAD_END)
                self.number_of_dead_end += 1
                if Logger.is_loggable(self._LOGSEVERITY, "low"):
                    self.__logger.log("*** ALL CHILDREN ARE DEAD END NODES ***", Color.GREEN)
                    self.__logger.log(" >>>> REGRESSION <<<< ", Color.YELLOW, newline=True)
                    self.__logger.log(f" --CURRENT NODE: {self.__maze_tree.current}", Color.YELLOW)
                    self.__logger.log(f" --PARENT NODE: {self.__maze_tree.current.parent}", Color.YELLOW)

                cur = self.__maze_tree.current.parent

            else:
                """ 
                This is the case when the action chosen by DMP is an action that brings the robot
                to an OBSERVED node and this node becomes the current node.
                """
                if Logger.is_loggable(self._LOGSEVERITY, "low"):
                    self.__logger.log("No leaf or DEAD END children", Color.YELLOW, italic=True)

                if self.__maze_tree.current.has_left and self.__maze_tree.current.left.action == action_chosen:
                    cur = self.__maze_tree.current.left
                elif self.__maze_tree.current.has_mid and self.__maze_tree.current.mid.action == action_chosen:
                    cur = self.__maze_tree.current.mid
                elif self.__maze_tree.current.has_right and self.__maze_tree.current.right.action == action_chosen:
                    cur = self.__maze_tree.current.right
                else:
                    if Logger.is_loggable(self._LOGSEVERITY, "low"):
                        self.__logger.log("!!! ESCAPING ERROR UPDATING CURRENT !!!", Color.DARKRED, True, True)
                        self.__logger.log(" >>>>  EXITING  <<<< ", Color.RED, italic=True)
                    exit(-1)

            self.__maze_tree.set_current(cur)

    # OK
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
        ori = ControllerData.Machine.z_axis()

        if self._state == State.STARTING and self._position == Position.INITIAL:
            if left is not None and right is not None:
                self._position = Position.CORRIDOR
            elif left is None or right is None:
                self._position = Position.JUNCTION
            self._state = State.SENSING
            # actions.insert(0, Command.START)
            com_actions.insert(0, [Command.START, None])

        elif self._state == State.ROTATING:
            actions.insert(0, self.__maze_performed_commands[len(self.__maze_performed_commands) - 1])
            com_actions.insert(0, [Command.RUN, detect_target(ControllerData.Machine.z_axis())])

        elif self._state == State.STOPPED or self._state == State.SENSING:

            self._state = State.SENSING

            if self.mode == Mode.EXPLORING:
                if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                    self.__logger.log("Control policy EXPLORING", Color.GRAY)

                if front is None:
                    action = f_r_l_b_to_compass(ori)["FRONT"]
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.RUN, action])
                if left is None:
                    action = f_r_l_b_to_compass(ori)["LEFT"]
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])
                if right is None:
                    action = f_r_l_b_to_compass(ori)["RIGHT"]
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])

                if not actions:
                    action = negate_compass(self.__maze_tree.current.action)
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])
                    self.mode = Mode.ESCAPING
                    self._state = State.SENSING

            elif self.mode == Mode.ESCAPING:
                if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                    self.__logger.log("Control policy ESCAPING", Color.GRAY)

                if self.__maze_tree.current.left is not None and self.__maze_tree.current.left.type == Type.OBSERVED:
                    action = self.__maze_tree.current.left.action
                    actions.insert(0, action)
                    if action == detect_target(ControllerData.Machine.z_axis()):
                        com_actions.insert(0, [Command.RUN, action])
                    else:
                        com_actions.insert(0, [Command.ROTATE, action])
                if self.__maze_tree.current.mid is not None and self.__maze_tree.current.mid.type == Type.OBSERVED:
                    action = self.__maze_tree.current.mid.action
                    actions.insert(0, action)
                    if action == detect_target(ControllerData.Machine.z_axis()):
                        com_actions.insert(0, [Command.RUN, action])
                    else:
                        com_actions.insert(0, [Command.ROTATE, action])
                if self.__maze_tree.current.right is not None and self.__maze_tree.current.right.type == Type.OBSERVED:
                    action = self.__maze_tree.current.right.action
                    actions.insert(0, action)
                    if action == detect_target(ControllerData.Machine.z_axis()):
                        com_actions.insert(0, [Command.RUN, action])
                    else:
                        com_actions.insert(0, [Command.ROTATE, action])

                """ 
                # If there are no OBSERVED nodes
                if not actions:
                    if self.__maze_tree.current.left is not None and self.__maze_tree.current.left.type == Type.EXPLORED:
                        action = self.__maze_tree.current.left.action
                        actions.insert(0, action)
                    if self.__maze_tree.current.mid is not None and self.__maze_tree.current.mid.type == Type.EXPLORED:
                        action = self.__maze_tree.current.mid.action
                        actions.insert(0, action)
                    if self.__maze_tree.current.right is not None and self.__maze_tree.current.right.type == Type.EXPLORED:
                        action = self.__maze_tree.current.right.action
                        actions.insert(0, action)
                """

                # Coming back, regressing
                if not actions:
                    action = negate_compass(self.__maze_tree.current.action)
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])

                if not actions:
                    if Logger.is_loggable(self._LOGSEVERITY, "low"):
                        self.__logger.log("NO OBSERVED NO EXPLORED NO ACTIONS", Color.DARKRED, True, True)
                        self.__logger.log(" >>>>  EXITING  <<<< ", Color.DARKRED, italic=True)
                    exit(-1)

        elif self._state == State.RUNNING:

            # Switch da ESCAPING A EXPLORING, messo qui perché se sto in RUNNING l'albero non viene aggiornato
            # in update_tree (per aggiornarlo lo stato deve essere in SENSING).
            # Evito così che vengano aggiunti dei nodi duplicati non voluti.

            if self.mode == Mode.ESCAPING and self.__maze_tree.current.type == Type.OBSERVED:
                self.mode = Mode.EXPLORING

            if self._position == Position.CORRIDOR:
                if left is None or right is None:
                    # actions.insert(0, Command.GO_TO_JUNCTION)
                    com_actions.insert(0, [Command.GO_TO_JUNCTION, detect_target(ControllerData.Machine.z_axis())])
                elif front is None or front > self._SAFE_DISTANCE:
                    # actions.insert(0, Command.RUN)
                    com_actions.insert(0, [Command.RUN, detect_target(ControllerData.Machine.z_axis())])
                elif front <= self._SAFE_DISTANCE:
                    # actions.insert(0, Command.STOP)
                    com_actions.insert(0, [Command.STOP, None])

            elif self._position == Position.JUNCTION:
                if left is not None and right is not None:
                    self._position = Position.CORRIDOR
                if front is None or front > self._SAFE_DISTANCE:
                    # actions.insert(0, Command.RUN)
                    com_actions.insert(0, [Command.RUN, detect_target(ControllerData.Machine.z_axis())])
                elif front <= self._SAFE_DISTANCE:
                    # actions.insert(0, Command.STOP)
                    com_actions.insert(0, [Command.STOP, None])

        return actions, com_actions

    # OK
    def __decision_making_policy(self, com_actions) -> list:
        """ Given a set of actions it decides what action the robot has to perform """

        if len(com_actions) == 1:
            return com_actions[0]

        if self._INTELLIGENCE == "mid" and self._state == State.SENSING:
            self.priority_list = random.sample(self.__robot_priority_list, 4)
            if Logger.is_loggable(self._LOGSEVERITY, "low"):
                self.__logger.log(f"--PRIORITY LIST (RANDOM): {self.priority_list}")
        elif self._INTELLIGENCE == "high":
            ...

        for direction in self.priority_list:  # [ S, N, O, E ]
            for com_action in com_actions:  # [[Command.ROTATE, Compass.NORD], [...], [...] ]
                action = com_action[1]
                if direction == action:
                    return com_action


    def __is_algorithm_unlocked(self) -> bool:
        if ControllerData.Machine.z_axis():
            return True
        return False

    #                                                                                                    #
    #                                                                                                    #
    # ************************************ END PURE ALGORITHM SECTION ********************************** #





    # ************************************* ROTATION SECTION ************************************* #
    #                                                                                              #
    # Main method: 'rotate'                                                                        #
    #                                                                                              #
    # ******************************************************************************************** #

    def rotate_to_final_g(self, vel, final_g):
        """ Rotate function that rotates the robot until it reaches final_g """

        init_g = ControllerData.Machine.z_axis()
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)
        
        self.__do_rotation(vel, c, degrees, final_g)

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
            if Logger.is_loggable(self._LOGSEVERITY, "low"):
                self.__logger.log(" ** MAX ATTEMPTS REACHED ** ", Color.DARKRED, True, True)
                self.__logger.log(" >>>>  EXITING  <<<< ", Color.DARKRED, italic=True)
            exit(-1)

    def __rotate(self, vel, c: Clockwise, degrees):
        """
        Function that given vel, Clockwise and rotation degrees computes
        the rotation of the Robot around the z axis
        The orientation of the robot must reach the interval [degrees - delta, degrees + delta]
        """
        degrees = abs(degrees)
        init_g = ControllerData.Machine.z_axis()

        delta = 0.8
        stop = False
        archived = False
        performed_deg = 0.0

        while not stop:
            curr_g = ControllerData.Machine.z_axis()

            '''
                Attivazione RC, emit buzzer e led blinking
            
            if c == Clockwise.RIGHT:
                self.__new_buzzer(status=True, emit=True)
                self.__new_led(status=True, arrow=True, clockwise=c, emit=True)

                time.sleep(1)

                self.__new_buzzer(status=False, emit=True)
                self.__new_led(status=False, arrow=False, emit=True)
                
            '''

            '''
                Algoritmo normale
            '''
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

    def check_orientation(self, final_g, delta=2):
        """
        This method is used to check if the orientation of the robot is correct
        The orientation of the robot must reach a specific interval according to two cases:
        1) The final_g is 180 or -180 (first if)
        2) Otherwise other intervals (second if)
        """

        if Logger.is_loggable(self._LOGSEVERITY, "mid"):
            self.__logger.log(" ** ORIENTATION CHECKING ** ", Color.GRAY, True, True)

        curr_g = ControllerData.Machine.z_axis()

        ok = False
        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if curr_g < limit_g_sx or curr_g > limit_g_dx:
                if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                    self.__logger.log(" ~~ perfect ~~ ", Color.GREEN)
                ok = True
            else:
                if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                    self.__logger.log(" !! bad orientation !! ", Color.RED)
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= curr_g <= limit_g_sx:
                if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                    self.__logger.log(" ~~ perfect ~~ ", Color.GREEN)

                ok = True
            else:
                if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                    self.__logger.log(" !! bad orientation !! ", Color.RED)

        if Logger.is_loggable(self._LOGSEVERITY, "mid"):
            self.__logger.log(
                f" >> curr state of the sensors: "
                f"[{round_v(limit_g_sx)}, {round_v(limit_g_dx)}], curr_g: {round_v(curr_g)}", Color.GRAY)

        limit_range = [limit_g_sx, limit_g_dx]
        return ok, curr_g, limit_range

    def adjust_orientation(self, final_g):
        """
        This method is used to adjust the orientation of the robot
        There are a max number of attempts:
        i) If the robot is able to orient himself correctly than the outcome is positive
        ii) If the robot fails, there is an error in adjusting the orientation and attempts stop
        """

        if Logger.is_loggable(self._LOGSEVERITY, "mid"):
            self.__logger.log(" ** ADJUSTING ORIENTATION ** ", Color.GRAY, True, True)

        self.__execute_motor(Command.STOP)

        ok = False
        it = 0

        while not ok and it < self._ROTATION_MAX_ATTEMPTS:
            curr_g = ControllerData.Machine.z_axis()

            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)

            if Logger.is_loggable(self._LOGSEVERITY, "mid"):
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

    def best_angle_and_rotation_way(self, init_g, final_g):
        """ Computes the best (minimum) angle between init_g and final_g and how you need to rotate """

        if Logger.is_loggable(self._LOGSEVERITY, "mid"):
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
            if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                self.__logger.log(f" >> Rotate clockwise (RIGHT) of {abs(round_v(smallest))}°", Color.GRAY)

            c = Clockwise.RIGHT
        else:
            if Logger.is_loggable(self._LOGSEVERITY, "mid"):
                self.__logger.log(f" >> Rotate anti-clockwise (LEFT) of {abs(round_v(smallest))}°", Color.GRAY)

            c = Clockwise.LEFT

        return smallest, c

    #                                                                                              #
    #                                                                                              #
    # *********************************** END ROTATION SECTION *********************************** #