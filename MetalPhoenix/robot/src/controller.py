import time
from math import pi
import random
from enum import Enum
from physical_body import PhysicalBody
from tools.utility import Logger, Compass, f_r_l_b_to_compass, negate_compass, \
    normalize_angle, round_v, Clockwise, detect_target, CFG
from tools.tree import Tree, Node, DIRECTION, Type


class State(Enum):
    STARTING = -1
    STOPPED = 0
    RUNNING = 1
    ROTATING = 2
    SENSING = 3
    BALANCING = 4


class Position(Enum):
    UNKNOWN = 0
    INITIAL = 1
    CORRIDOR = 2
    JUNCTION = 3


class Mode(Enum):
    EXPLORING = 0
    ESCAPING = 1


class Command(Enum):
    START = -1
    STOP = 0
    RUN = 1
    ROTATE = 2
    GO_TO_JUNCTION = 3
    BALANCE = 4


class Controller:
    def __init__(self):

        self.__class_logger = Logger(class_name="Controller", color="cyan")
        self.__class_logger.set_logfile(CFG.logger_data()["CLOGFILE"])
        self._severity = CFG.logger_data()["SEVERITY"]
        self.__class_logger.log("CONTROLLER LAUNCHED", color="green", italic=True)
        self.__class_logger.log(f"LOG SEVERITY: {str.upper(self._severity)}", color="green")

        self._body = PhysicalBody()
        self._body.stop()

        # Initial states
        self._state = State.STARTING
        self._position = Position.UNKNOWN
        self._mode = Mode.EXPLORING

        # Initializing robot's variables and the values are taken from the config file
        self.robot_config_data = CFG.robot_conf_data()
        self.speed = self.robot_config_data["SPEED"]
        self.rot_speed = self.robot_config_data["ROT_SPEED"]
        self.safe_distance = self.robot_config_data["SAFE_DIST"]
        self.safe_side_distance = self.robot_config_data["SAFE_SIDE_DIST"]
        self.max_rot_attempts = self.robot_config_data["MAX_ROT_ATTEMPTS"]
        self.priority_list = self.robot_config_data["PRIORITY_LIST"]
        self.intelligence = self.robot_config_data["INTELLIGENCE"]
        self.auto_balancing = self.robot_config_data["AUTO_BALANCING"]

        # TIMERS
        # Junction Time. Time it takes to position in the center of a junction
        # Original version:
        self._speed_m_on_sec = self.speed * 0.17 / 5
        self.junction_sim_time = 0.23 / self._speed_m_on_sec
        # Modified version:
        # //: floor of the value obtained by the division
        # self._speed_m_on_sec = self.speed * 0.25 / (self.speed // 5)
        # self.junction_sim_time = 0.25 / self._speed_m_on_sec

        # Start and end timers
        self.start_time = None
        self.end_time = None

        # Variables of the robot orientation, proximity sensors and gate sensor
        self.orientation = None
        self.left_value = None
        self.front_value = None
        self.right_value = None
        self.gate = False

        # Lists of values of the proximity sensors
        self.front_values = list()
        self.left_values = list()
        self.right_values = list()

        self.trajectory = list()
        self.performed_commands = list()
        self.performed_com_actions = list(list())
        self.prev_action = None
        self.prev_com_action = list()
        self.tree = Tree()

        # Variables used for balancing
        self.max_side_distance = 0.16
        self.attempts_to_unstuck = 0
        self.cycle = 0
        self.store = Store()

        """ DATA ANALYSIS """
        # Each maze must have a number to be identified, change this number if the maze changes
        self.maze_number = CFG.maze_data()["MAZE_NUMBER"]
        self.maze_name = "Maze" + "_" + str(self.maze_number) + "_" + \
                         (Compass.compass_list_to_concat_string(self.priority_list) if self.intelligence == "low"
                          else "RANDOM")
        self.__class_logger.log("Maze name: " + str(self.maze_name), "green")
        self.execution_time = 0
        self.number_of_nodes = 1
        self.number_of_dead_end = 0
        self.maze_solved = False

        self.__class_logger.log(f'>>> Declared constants <<<', "green")
        for k, v in self.robot_config_data.items():
            self.__class_logger.log(f"{k} : {v}", "green")
        time.sleep(2)

        self.start_time = time.time()

    def virtual_destructor(self):
        self._body.virtual_destructor()
        self.__class_logger.log("CONTROLLER STOPPED", "red", italic=True)

    def write_data_analysis(self):
        priority_list = Compass.compass_list_to_string_comma_sep(self.priority_list)
        CFG.write_data_analysis(self.maze_name, self.maze_solved, round_v(self.execution_time), self.speed,
                                self.rot_speed, self.safe_distance, self.safe_side_distance, self.max_rot_attempts,
                                self.number_of_nodes, self.number_of_dead_end, self.tree.build_tree_dict(),
                                self.trajectory, self.performed_commands, self.performed_com_actions, self.intelligence,
                                "on" if self.auto_balancing == "on" else "off",
                                priority_list if self.intelligence == "low" else "random")

    def read_sensors(self):
        self.left_value = self._body.get_proxL()
        self.front_value = self._body.get_proxF()
        self.right_value = self._body.get_proxR()
        self.orientation = self._body.get_orientation_deg()

    def algorithm(self):
        """ Algorithm used to explore and solve the maze """

        self.cycle = self.cycle + 1

        self.__class_logger.log(" >>>>>> NEW ALGORITHM CYCLE <<<<<< ", "green", newline=True)

        if self.goal_reached():
            return True

        # SENSE
        self.read_sensors()
        self.left_values.append(self.left_value)
        self.front_values.append(self.front_value)
        self.right_values.append(self.right_value)

        # THINK
        actions, com_actions = self.control_policy()
        com_action = self.decision_making_policy(com_actions)
        if com_action is None:
            self.__class_logger.log("NO ACTION AVAILABLE!", "dkred", newline=True)
            self.__class_logger.log(" >>>>  EXITING  <<<< ", "dkred", italic=True)
            self.virtual_destructor()
            exit(-1)
        command, action = com_action

        if Logger.is_loggable(self._severity, "low"):
            self.__class_logger.log(f"--MODE: {self._mode}")
            self.__class_logger.log(f"--ACTIONS: {com_actions}")
            self.__class_logger.log(f"--ACTION: {com_action}")
            self.__class_logger.log(f"--CURRENT NODE: {self.tree.current}")

        # Update of the tree
        self.update_tree(actions, action)

        if Logger.is_loggable(self._severity, "low"):
            self.__class_logger.log("--CURRENT TREE:", "gray")
            self.__class_logger.log(f"{self.tree.build_tree_dict()}", "gray", noheader=True)
            self.__class_logger.log(f"--CURRENT NODE: {self.tree.current}", "gray", newline=True)
            self.__class_logger.log(f"--Available actions: {com_actions}", "green")

        if Logger.is_loggable(self._severity, "mid"):
            self.__class_logger.log(f"--(STATE, POSITION): ({self._state}, {self._position})", "gray")
            self.__class_logger.log(f"--Performing action: {com_action}", "gray")

        # ACT
        performed = self.do_action(com_action)

        if Logger.is_loggable(self._severity, "mid"):
            self.__class_logger.log(f"--(STATE, POSITION): ({self._state}, {self._position})", "gray")

        # Saving performed commands and actions
        if self.prev_com_action != com_action:
            self.performed_commands.append(command)
            self.performed_com_actions.append(com_action)
            self.prev_com_action = com_action
        # Saving trajectory
        if performed and com_action[0] is not Command.ROTATE and action is not None and self.prev_action != action:
            self.trajectory.append(action)
            self.prev_action = action

        return False

    def update_tree(self, actions, action_chosen):
        """
         This method is used to update the tree of the maze accordingly to the actions and the current state
         of the robot.
         The update is done if only the robot is in SENSING mode and the actions and action_chosen type must
         be a Compass or a list of Compass elements respectively.
        """

        """ if not actions:
            print("UPDATE_TREE NO ACTIONS")
            exit(-1)
        """

        if not self._state == State.SENSING:
            return

        if self._mode == Mode.EXPLORING:
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log("*** UPDATING TREE (MODE: EXPLORING) ***", "gray", newline=True)

            """ 
            Different actions returned by Control Policy.
            In this section are added the nodes of the tree based on the available actions, 
            updating also the current node as EXPLORED 
            """
            for action in actions:
                dict_ = f_r_l_b_to_compass(self.orientation)
                if dict_["FRONT"] == action:
                    node = Node("M_" + self.tree.generate_node_id(), action)
                    self.tree.append(node, DIRECTION.MID)
                    self.tree.regress()
                    self.number_of_nodes += 1
                    if Logger.is_loggable(self._severity, "mid"):
                        self.__class_logger.log("ADDED MID", "dkgreen")
                if dict_["LEFT"] == action:
                    node = Node("L_" + self.tree.generate_node_id(), action)
                    self.tree.append(node, DIRECTION.LEFT)
                    self.tree.regress()
                    self.number_of_nodes += 1
                    if Logger.is_loggable(self._severity, "mid"):
                        self.__class_logger.log("ADDED LEFT", "dkgreen")
                if dict_["RIGHT"] == action:
                    node = Node("R_" + self.tree.generate_node_id(), action)
                    self.tree.append(node, DIRECTION.RIGHT)
                    self.tree.regress()
                    self.number_of_nodes += 1
                    if Logger.is_loggable(self._severity, "mid"):
                        self.__class_logger.log("ADDED RIGHT", "dkgreen")

            self.tree.current.set_type(Type.EXPLORED)

            """ 
            Only one action that has been decided by DMP. 
            In this section it is updated the current node of the tree based on the action chosen 
            """
            dict_ = f_r_l_b_to_compass(self.orientation)
            if dict_["FRONT"] == action_chosen:
                self.tree.set_current(self.tree.current.mid)
            elif dict_["LEFT"] == action_chosen:
                self.tree.set_current(self.tree.current.left)
            elif dict_["RIGHT"] == action_chosen:
                self.tree.set_current(self.tree.current.right)

        elif self._mode == Mode.ESCAPING:
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log("*** UPDATING TREE (MODE: ESCAPING) ***", "gray", newline=True)

            """ 
            In this section it is updated the type property of the current node accordingly if
            the current node is a leaf or has children that are all dead end, otherwise it is updated the current node
            """

            cur = None

            # The node is a leaf
            if self.tree.current.is_leaf:
                self.tree.current.set_type(Type.DEAD_END)
                self.number_of_dead_end += 1
                if Logger.is_loggable(self._severity, "low"):
                    self.__class_logger.log("*** DEAD END NODE DETECTED ***", "green")
                    self.__class_logger.log(" >>>> REGRESSION <<<< ", "yellow", newline=True)
                    self.__class_logger.log(f" --CURRENT NODE: {self.tree.current}", "yellow")
                    self.__class_logger.log(f" --PARENT NODE: {self.tree.current.parent}", "yellow")

                cur = self.tree.current.parent

            # The children are all DEAD END
            elif ((self.tree.current.has_left and self.tree.current.left.type == Type.DEAD_END) or
                  self.tree.current.left is None) and \
                    ((self.tree.current.has_right and self.tree.current.right.type == Type.DEAD_END)
                     or self.tree.current.right is None) and \
                    ((self.tree.current.has_mid and self.tree.current.mid.type == Type.DEAD_END)
                     or self.tree.current.mid is None):
                self.tree.current.set_type(Type.DEAD_END)
                self.number_of_dead_end += 1
                if Logger.is_loggable(self._severity, "low"):
                    self.__class_logger.log("*** ALL CHILDREN ARE DEAD END NODES ***", "green")
                    self.__class_logger.log(" >>>> REGRESSION <<<< ", "yellow", newline=True)
                    self.__class_logger.log(f" --CURRENT NODE: {self.tree.current}", "yellow")
                    self.__class_logger.log(f" --PARENT NODE: {self.tree.current.parent}", "yellow")

                cur = self.tree.current.parent

            else:
                """ 
                This is the case when the action chosen by DMP is an action that brings the robot
                to an OBSERVED node and this node becomes the current node.
                """
                if Logger.is_loggable(self._severity, "low"):
                    self.__class_logger.log("No leaf or DEAD END children", "yellow+", italic=True)

                if self.tree.current.has_left and self.tree.current.left.action == action_chosen:
                    cur = self.tree.current.left
                elif self.tree.current.has_mid and self.tree.current.mid.action == action_chosen:
                    cur = self.tree.current.mid
                elif self.tree.current.has_right and self.tree.current.right.action == action_chosen:
                    cur = self.tree.current.right
                else:
                    if Logger.is_loggable(self._severity, "low"):
                        self.__class_logger.log("!!! ESCAPING ERROR UPDATING CURRENT !!!", "dkred", True, True)
                        self.__class_logger.log(" >>>>  EXITING  <<<< ", "red", italic=True)
                    self.virtual_destructor()
                    exit(-1)

            self.tree.set_current(cur)

    def control_policy(self) -> tuple:
        """
        Accordingly to the values of the sensors, the state of the robot and the tree of the maze,
        it returns a tuple of two list.
        The first is a list of actions (elements of Compass type), it is used only to update the tree.
        The second list, com_actions that is the abbreviation of command_actions,
        is a list of lists of two elements type: Command and Compass.
        It contains a set of couples [command, action] that the robot can perform, but only one of these
        can be executed.
        """

        actions = list()
        com_actions = list(list())

        left = self.left_value
        front = self.front_value
        right = self.right_value
        ori = self.orientation

        if self.auto_balancing == "on":
            if self.left_value is not None and self.left_value < self.safe_side_distance:
                self.__class_logger.log("ALERT: WALL ON THE LEFT. VALUE: " +
                                        str(self.left_value), "yellow", True, True)
                # Saving the current state only if it is diff. from BALANCING.
                # It may happen that is necessary two or more balancing, and we have to save
                # only the state before balancing that must be restored after balancing.
                if self._state != State.BALANCING:
                    self.save_current_state()
                self._state = State.BALANCING
                actions = None
                com_actions.insert(0, [Command.BALANCE, None])

            elif self.right_value is not None and self.right_value < self.safe_side_distance:
                self.__class_logger.log("ALERT: WALL ON THE RIGHT. VALUE: " +
                                        str(self.right_value), "yellow", True, True)
                if self._state != State.BALANCING:
                    self.save_current_state()
                self._state = State.BALANCING
                actions = None
                com_actions.insert(0, [Command.BALANCE, None])

            elif self._state == State.BALANCING:
                self.load_saved_state()

        if self._state == State.STARTING and self._position == Position.UNKNOWN:

            if front is None:
                # Wall behind
                # (since in the constraints I can't put the robot in a junction where there are
                # 4 free directions/roads).
                if (left is not None and right is not None) or (left is None and right is None):
                    self._state = State.STARTING
                    self._position = Position.INITIAL
                    com_actions.insert(0, [Command.START, None])
                else:
                    # Wall on the left
                    if left is not None:
                        action = detect_target(detect_target(self.orientation) - 90)
                        com_actions.insert(0, [Command.ROTATE, action])
                        self.__class_logger.log("--WALL ON THE LEFT", "yellow", True, True)

                    # Wall on the right
                    elif right is not None:
                        action = detect_target(detect_target(self.orientation) + 90)
                        com_actions.insert(0, [Command.ROTATE, action])
                        self.__class_logger.log("--WALL ON THE RIGHT", "yellow", True, True)

            # The robot now has to rotate of 180° to have the wall behind
            elif front is not None:
                if left is None or right is None:
                    action = negate_compass(detect_target(self.orientation))
                    com_actions.insert(0, [Command.ROTATE, action])

                elif left is not None and right is not None:
                    action = negate_compass(detect_target(self.orientation))
                    com_actions.insert(0, [Command.ROTATE, action])
                    self.attempts_to_unstuck = 1

        # In this case the robot has always the wall behind
        elif self._state == State.STARTING and self._position == Position.INITIAL:

            if front is None:
                if left is not None and right is not None:
                    self._state = State.SENSING
                    self._position = Position.CORRIDOR
                    self.attempts_to_unstuck = 0

                elif left is None or right is None:
                    self._state = State.SENSING
                    self._position = Position.JUNCTION
                    self.attempts_to_unstuck = 0

            elif front is not None:
                if left is None or right is None:
                    self._state = State.SENSING
                    self._position = Position.CORRIDOR
                    self.attempts_to_unstuck = 0

                elif left is not None and right is not None:
                    if self.attempts_to_unstuck == 1:
                        self.__class_logger.log("Robot in stuck. Cannot solve the maze.", "red", True, True)
                        self.virtual_destructor()
                        exit(-1)

            com_actions.insert(0, [Command.START, None])

        elif self._state == State.ROTATING:  # the robot has already rotated
            if self._position == Position.UNKNOWN:
                self._state = State.STARTING
                self._position = Position.INITIAL
                com_actions.insert(0, [Command.START, None])

            else:
                actions.insert(0, self.performed_commands[len(self.performed_commands) - 1])
                com_actions.insert(0, [Command.RUN, detect_target(self.orientation)])

        elif self._state == State.STOPPED or self._state == State.SENSING:

            self._state = State.SENSING

            if self._mode == Mode.EXPLORING:
                if Logger.is_loggable(self._severity, "mid"):
                    self.__class_logger.log("Control policy EXPLORING", "gray")

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
                    self._mode = Mode.ESCAPING
                    self._state = State.SENSING
                    action = negate_compass(self.tree.current.action)
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])

            elif self._mode == Mode.ESCAPING:
                if Logger.is_loggable(self._severity, "mid"):
                    self.__class_logger.log("Control policy ESCAPING", "gray")

                if self.tree.current.left is not None and self.tree.current.left.type == Type.OBSERVED:
                    action = self.tree.current.left.action
                    actions.insert(0, action)
                    if action == detect_target(self.orientation):
                        com_actions.insert(0, [Command.RUN, action])
                    else:
                        com_actions.insert(0, [Command.ROTATE, action])
                if self.tree.current.mid is not None and self.tree.current.mid.type == Type.OBSERVED:
                    action = self.tree.current.mid.action
                    actions.insert(0, action)
                    if action == detect_target(self.orientation):
                        com_actions.insert(0, [Command.RUN, action])
                    else:
                        com_actions.insert(0, [Command.ROTATE, action])
                if self.tree.current.right is not None and self.tree.current.right.type == Type.OBSERVED:
                    action = self.tree.current.right.action
                    actions.insert(0, action)
                    if action == detect_target(self.orientation):
                        com_actions.insert(0, [Command.RUN, action])
                    else:
                        com_actions.insert(0, [Command.ROTATE, action])

                """ 
                # If there are no OBSERVED nodes
                if not actions:
                    if self.tree.current.left is not None and self.tree.current.left.type == Type.EXPLORED:
                        action = self.tree.current.left.action
                        actions.insert(0, action)
                    if self.tree.current.mid is not None and self.tree.current.mid.type == Type.EXPLORED:
                        action = self.tree.current.mid.action
                        actions.insert(0, action)
                    if self.tree.current.right is not None and self.tree.current.right.type == Type.EXPLORED:
                        action = self.tree.current.right.action
                        actions.insert(0, action)
                """

                if not actions:
                    if (self.tree.current.left is None or self.tree.current.left.type == Type.DEAD_END) and \
                            (self.tree.current.mid is None or self.tree.current.mid.type == Type.DEAD_END) and \
                            (self.tree.current.right is None or self.tree.current.right.type == Type.DEAD_END) and \
                            self.tree.current.action is None:
                        self.__class_logger.log("NO OBSERVED NO EXPLORED NO ACTIONS", "dkred", True, True)
                        self.__class_logger.log(" >>>>  EXITING  <<<< ", "dkred", italic=True)
                        self.virtual_destructor()
                        exit(-1)

                    # Coming back, regressing
                    action = negate_compass(self.tree.current.action)
                    actions.insert(0, action)
                    com_actions.insert(0, [Command.ROTATE, action])

        elif self._state == State.RUNNING:

            # Switch from ESCAPING to EXPLORING.
            # Put here because if the robot is in RUNNING mode then the tree cannot be updated in update_tree()
            # (to update the tree it must be in SENSING mode);
            # This prevents unwanted duplicate nodes from being added.

            if self._mode == Mode.ESCAPING and self.tree.current.type == Type.OBSERVED:
                self._mode = Mode.EXPLORING

            if self._position == Position.CORRIDOR:
                if left is None or right is None:
                    com_actions.insert(0, [Command.GO_TO_JUNCTION, detect_target(self.orientation)])
                elif front is None or front > self.safe_distance:
                    com_actions.insert(0, [Command.RUN, detect_target(self.orientation)])
                elif front <= self.safe_distance:
                    com_actions.insert(0, [Command.STOP, None])

            elif self._position == Position.JUNCTION:
                if left is not None and right is not None:
                    self._position = Position.CORRIDOR
                if front is None or front > self.safe_distance:
                    com_actions.insert(0, [Command.RUN, detect_target(self.orientation)])
                elif front <= self.safe_distance:
                    com_actions.insert(0, [Command.STOP, None])

        return actions, com_actions

    def decision_making_policy(self, com_actions) -> list:
        """ Given a set of actions it decides what action the robot has to perform """

        if len(com_actions) == 1:
            return com_actions[0]

        if self.intelligence == "mid" and self._state == State.SENSING:
            self.priority_list = random.sample(self.priority_list, 4)
            if Logger.is_loggable(self._severity, "low"):
                self.__class_logger.log(f"--PRIORITY LIST (RANDOM): {self.priority_list}")
        elif self.intelligence == "high":
            ...

        for direction in self.priority_list:  # [ S, N, W, E ]
            for com_action in com_actions:  # [[Command.ROTATE, Compass.NORD], [...], [...] ]
                action = com_action[1]
                if direction == action:
                    return com_action

    def do_action(self, com_action):
        """
         Accordingly to the action chosen by 'Decision Making Policy' this method translates the action and send
         a specific command to the PhysicalBody that has to perform, changing also the Robot state
        """

        if Logger.is_loggable(self._severity, "high"):
            self.__class_logger.log(" ~~~ [ACTION TIME] ~~~ ", "gray", True, True)

        if com_action[0] == Command.START:
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(" ** COMMAND START ** ", "gray")

            self._body.stop()

            return True

        # Stop
        elif com_action[0] == Command.STOP:
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(" ** COMMAND STOP ** ", "gray")
            self._body.stop()
            time.sleep(0.2)
            print(self._body.get_proxF())
            self._state = State.STOPPED

            return True

        # Go on
        elif com_action[0] == Command.RUN:
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(" ** COMMAND RUN ** ", "gray")

            self._body.move_forward(self.speed)
            self._state = State.RUNNING

            return True

        # Go to Junction
        elif com_action[0] == Command.GO_TO_JUNCTION:
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(" ** JOINING THE JUNCTION ** ", "gray")

            start_time = time.time()
            time_expired = False
            """
            while self._body.get_proxR() is None:
                ...

            end_time = time.time() - start_time

            print("End time: ", end_time)
            exit(-1)
            """

            while not time_expired and (self._body.get_proxF() is None or self._body.get_proxF() > self.safe_distance):
                self._body.move_forward(self.speed)
                if time.time() - start_time >= self.junction_sim_time:
                    time_expired = True

            self._body.stop()
            self._state = State.SENSING
            self._position = Position.JUNCTION

            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(" ** STATE SENSING ARISE ** ", "gray")

            time.sleep(0.5)

            return True

        elif com_action[0] == Command.ROTATE:
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(" ** COMMAND ROTATE ** ", "gray")

            self._state = State.ROTATING
            self._body.stop()
            self.rotate_to_final_g(self.rot_speed, com_action[1])
            self._body.stop()
            time.sleep(0.5)

            return True

        elif com_action[0] == Command.BALANCE:
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(" ** COMMAND BALANCING ** ", "gray")
            self._body.stop()
            action = None
            sign = None

            if self.left_value is not None and self.left_value < self.safe_side_distance:
                action = detect_target(detect_target(self.orientation) + 90)
                sign = -1
            elif self.right_value is not None and self.right_value < self.safe_side_distance:
                action = detect_target(detect_target(self.orientation) - 90)
                sign = +1

            self.rotate_to_final_g(self.rot_speed, action)

            while self._body.get_proxF() < self.max_side_distance:
                self._body.move_backward(5)
                # print("Front value:" + str(self._body.get_proxF()))
            self._body.stop()

            action = detect_target(detect_target(self._body.get_orientation_deg()) + 90 * sign)
            self.rotate_to_final_g(self.rot_speed, action)
            self._body.stop()
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(" ** BALANCED ** ", "green")
            time.sleep(3)
            return True

        else:
            self.__class_logger.log("ERROR: ACTION NOT RECOGNIZED", "red")
            self.virtual_destructor()
            exit(-1)

    def save_current_state(self):
        self.store.store(self._state, self._position, self._mode)

    def load_saved_state(self):
        self._state, self._position, self._mode = self.store.load()

    def rotate_to_final_g(self, vel, final_g):
        """ Rotation function that rotates the robot until it reaches final_g """
        self._body.stop()

        init_g = self._body.get_orientation_deg()
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)

        self.__do_rotation(vel=vel, c=c, degrees=degrees, final_g=final_g)

        self._body.stop()

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

        self._body.stop()
        self.__rotate(vel, c, degrees)

        ok, curr_g, limit_range = self.check_orientation(final_g)

        if not ok:
            ok, it = self.adjust_orientation(final_g)

        # CRITICAL CASE
        if it == self.max_rot_attempts:
            if Logger.is_loggable(self._severity, "low"):
                self.__class_logger.log(" ** MAX ATTEMPTS REACHED ** ", "dkred", True, True)
                self.__class_logger.log(" >>>>  EXITING  <<<< ", "dkred", italic=True)
            self.virtual_destructor()
            exit(-1)

    def __rotate(self, vel, c: Clockwise, degrees):
        """
        Function that given vel, Clockwise and rotation degrees computes
        the rotation of the Robot around the z axis.
        The orientation of the robot must reach the interval [degrees - delta, degrees + delta]
        """
        degrees = abs(degrees)
        init_g = self._body.get_orientation_deg()

        delta = 0.8  # usually 0.8
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

            """ 
            Check if there was an unintended move in the opposite direction. 
            The degrees performed in this case would be > 300 and it is not considered (continue)
            """

            if performed_deg_temp > 300:
                continue

            performed_deg = performed_deg_temp

            if degrees - delta < performed_deg < degrees + delta:
                self._body.stop()
                archived = True
                stop = True
            elif performed_deg > degrees + delta:
                self._body.stop()
                archived = False
                stop = True

        self._body.stop()
        return archived, init_g, performed_deg, degrees

    def check_orientation(self, final_g, delta=2):
        """
        This method is used to check if the orientation of the robot is correct
        The orientation of the robot must reach a specific interval according to two cases:
        1) The final_g is 180 or -180 (first if)
        2) Otherwise other intervals (second if)
        """

        if Logger.is_loggable(self._severity, "mid"):
            self.__class_logger.log(" ** ORIENTATION CHECKING ** ", "gray", True, True)

        curr_g = self._body.get_orientation_deg()

        ok = False
        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if curr_g < limit_g_sx or curr_g > limit_g_dx:
                if Logger.is_loggable(self._severity, "mid"):
                    self.__class_logger.log(" ~~ perfect ~~ ", "green")
                ok = True
            else:
                if Logger.is_loggable(self._severity, "mid"):
                    self.__class_logger.log(" !! bad orientation !! ", "red")
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= curr_g <= limit_g_sx:
                if Logger.is_loggable(self._severity, "mid"):
                    self.__class_logger.log(" ~~ perfect ~~ ", "green")

                ok = True
            else:
                if Logger.is_loggable(self._severity, "mid"):
                    self.__class_logger.log(" !! bad orientation !! ", "red")

        if Logger.is_loggable(self._severity, "mid"):
            self.__class_logger.log(
                f" >> interval: "
                f"[{round_v(limit_g_sx)}, {round_v(limit_g_dx)}], curr_g: {round_v(curr_g)}", "gray")

        limit_range = [limit_g_sx, limit_g_dx]
        return ok, curr_g, limit_range

    def adjust_orientation(self, final_g):
        """
        This method is used to adjust the orientation of the robot
        There are a max number of attempts:
        i) If the robot is able to orient himself correctly than the outcome is positive
        ii) If the robot fails, there is an error in adjusting the orientation and attempts stop
        """

        if Logger.is_loggable(self._severity, "mid"):
            self.__class_logger.log(" ** ADJUSTING ORIENTATION ** ", "gray", True, True)

        self._body.stop()

        ok = False
        it = 0

        while not ok and it < self.max_rot_attempts:
            curr_g = self._body.get_orientation_deg()

            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)

            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(f" --ATTEMPT: {it + 1} / {self.max_rot_attempts}", "gray")
                self.__class_logger.log(
                    f"[Degrees_to_do, curr_g, final_g] = [{round_v(degrees)}, {round_v(curr_g)}, {round_v(final_g)}]",
                    "gray")

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

        if Logger.is_loggable(self._severity, "mid"):
            self.__class_logger.log(" ** BEST ANGLE COMPUTATION ** ", "gray", True, True)

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
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(f" >> Rotate clockwise (RIGHT) of {abs(round_v(smallest))}°", "gray")

            c = Clockwise.RIGHT
        else:
            if Logger.is_loggable(self._severity, "mid"):
                self.__class_logger.log(f" >> Rotate anti-clockwise (LEFT) of {abs(round_v(smallest))}°", "gray")

            c = Clockwise.LEFT

        return smallest, c

    def goal_reached(self) -> bool:
        """
        It is used to check if the robot has found the exit of the maze.
        The exit of the maze is identified using the gate.
        It updates the execution time and the last node of the tree as FINAL node.
        """

        if self._body.get_gate():
            self.end_time = time.time()
            self.execution_time = self.end_time - self.start_time
            self.maze_solved = True
            self.tree.current.set_name("FINAL")
            self.tree.current.set_type(Type.FINAL)
            self.print_data(maze_solved=self.maze_solved)
            self.write_data_analysis()
            return True

        return False

    def load_cfg_values(self):
        """
        It loads the values of the config file since it can be modified also during the execution of the algorithm
        """

        self.robot_config_data = CFG.robot_conf_data()
        self.speed = self.robot_config_data["SPEED"]
        self.rot_speed = self.robot_config_data["ROT_SPEED"]
        self.safe_distance = self.robot_config_data["SAFE_DIST"]
        self.safe_side_distance = self.robot_config_data["SAFE_SIDE_DIST"]
        self.max_rot_attempts = self.robot_config_data["MAX_ROT_ATTEMPTS"]
        self.priority_list = self.robot_config_data["PRIORITY_LIST"]
        self.intelligence = self.robot_config_data["INTELLIGENCE"]
        self.auto_balancing = self.robot_config_data["AUTO_BALANCING"]
        # self.maze_number = self.robot_config_data["MAZE_NUMBER"]
        self._severity = self.robot_config_data["SEVERITY"]
        PhysicalBody.load_cfg_values()

    def print_data(self, maze_solved):
        if maze_solved:
            self.__class_logger.log(" >> MAZE SOLVED << ", "green", italic=True)
        else:
            self.__class_logger.log(" >> MAZE NOT SOLVED << ", "red", italic=True)

        self.__class_logger.log("Maze name: " + str(self.maze_name), "green", True, True)
        self.__class_logger.log("Execution time: " + str(self.execution_time), "green", True, True)
        self.__class_logger.log("Number of nodes: " + str(self.number_of_nodes), "green", True, True)
        self.__class_logger.log("Number of dead end: " + str(self.number_of_dead_end), "green", True, True)
        self.__class_logger.log("Tree: " + str(self.tree.build_tree_dict()), "green", True, True)
        self.__class_logger.log("Trajectory: " + str(self.trajectory), "green", True, True)
        self.__class_logger.log("Performed commands: " + str(self.performed_commands), "green", True, True)
        self.__class_logger.log("Performed commands and actions: " + str(self.performed_com_actions), "green", True,
                                True)


class Store:
    def __init__(self):
        self.state = None
        self.position = None
        self.mode = None

    def store(self, state, position, mode):
        self.state = state
        self.position = position
        self.mode = mode

    def load(self):
        return self.state, self.position, self.mode
