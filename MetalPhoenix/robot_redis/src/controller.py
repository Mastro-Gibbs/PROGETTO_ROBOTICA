import json
import time
import random

from math import pi
from enum import Enum

from tools.utility import Logger, Compass, f_r_l_b_to_compass, negate_compass, \
                          normalize_angle, round_v, Clockwise, detect_target, CFG
from tools.tree import Tree, Node, DIRECTION, Type

from redis import Redis


class State(Enum):
    STARTING = -1
    STOPPED = 0
    RUNNING = 1
    ROTATING = 2
    SENSING = 3


class Position(Enum):
    INITIAL = 0  # or UNKNOWN
    CORRIDOR = 1
    JUNCTION = 2


class Command(Enum):
    START = -1
    STOP = 0
    RUN = 1
    ROTATE = 2  # ?
    GO_TO_JUNCTION = 3


class Mode(Enum):
    EXPLORING = 0
    ESCAPING = 1


class RedisData:
    class Connection:
        Host = CFG.redis_data()["HOST"]
        Port = CFG.redis_data()["PORT"]

    class Topic:
        Body = CFG.redis_data()["B_TOPIC"]
        Remote = CFG.redis_data()["RC_TOPIC"]
        Controller = CFG.redis_data()["C_TOPIC"]

    class Key:
        Led = CFG.redis_data()["LED_KEY"]
        Motor = CFG.redis_data()["MOTORS_KEY"]
        Buzzer = CFG.redis_data()["BUZZER_KEY"]

    class Value:
        class __Value:
            __status = False

            @classmethod
            @property
            def changed(cls):
                return cls.__status

            @classmethod
            def toggle(cls):
                cls.__status = not cls.__status

            @classmethod
            def set(cls, _b: bool):
                cls.__status = _b

        class Machine:
            __data = None

            @classmethod
            def on_values(cls, data):
                cls.__data = data

            @classmethod
            def front(cls):
                return float(cls.__data['proxF']) if cls.__data['proxF'] != 'None' else None

            @classmethod
            def left(cls):
                return float(cls.__data['proxL']) if cls.__data['proxL'] != 'None' else None

            @classmethod
            def right(cls):
                return float(cls.__data['proxR']) if cls.__data['proxR'] != 'None' else None

            @classmethod
            def back(cls):
                return float(cls.__data['proxB']) if cls.__data['proxB'] != 'None' else None

            @classmethod
            def gate(cls):
                return True if cls.__data['proxG'] == 'True' else False

            @classmethod
            def z_axis(cls):
                return float(cls.__data['Zaxis']) if cls.__data['Zaxis'] != 'None' else None

        class Motor(__Value):
            __rum = None
            __lum = None
            __rlm = None
            __llm = None

            @classmethod
            def on_values(cls, rum: int, lum: int, rlm: int, llm: int):
                if cls.__rum == rum and cls.__lum == lum and cls.__rlm == rlm and cls.__llm == llm:
                    super().set(False)
                else:
                    cls.__rum = rum
                    cls.__lum = lum
                    cls.__rlm = rlm
                    cls.__llm = llm

                    super().set(True)

            @classmethod
            @property
            def values(cls):
                data = dict()
                data['rum'] = cls.__rum
                data['lum'] = cls.__lum
                data['rlm'] = cls.__rlm
                data['llm'] = cls.__llm

                super().set(False)

                return json.dumps(data, indent=0)

        class Led(__Value):
            pass

        class Buzzer(__Value):
            pass


class Controller:
    OR_MAX_ATTEMPT = CFG.robot_conf_data()["MAX_ATTEMPTS"]
    SAFE_DISTANCE = CFG.robot_conf_data()["SAFE_DIST"]
    LOG_SEVERITY = CFG.logger_data()["SEVERITY"]
    INTELLIGENCE = CFG.robot_conf_data()["INTELLIGENCE"]

    def __init__(self):
        """<!-- REDIS SECTION -->"""
        self.__redis = Redis(RedisData.Connection.Host, RedisData.Connection.Port, decode_responses=True)
        self.__pubsub = self.__redis.pubsub()
        self.__pubsub.subscribe(RedisData.Topic.Body)

        self.__class_logger = Logger(class_name="Controller", color="cyan")
        self.__class_logger.set_logfile(CFG.logger_data()["CLOGFILE"])
        self.__class_logger.log(f"LOG SEVERITY: {str.upper(self.LOG_SEVERITY)}\n", color="dkgreen")
        self.__class_logger.log("CONTROLLER LAUNCHED", color="green", italic=True)

        self._state = State.STARTING
        self._position = Position.INITIAL
        self.mode = Mode.EXPLORING

        self.robot_config_data = CFG.robot_conf_data()
        self._speed = self.robot_config_data["SPEED"]
        self._rot_speed = self.robot_config_data["ROT_SPEED"]

        self._speed_m_on_sec = self._speed * 0.25 / (self._speed // 5)
        self.junction_sim_time = 0.25 / self._speed_m_on_sec  # Time it takes to position in the center of a junction

        self.front_values = list()
        self.left_values = list()
        self.right_values = list()

        self.target = 0

        self.priority_list = self.robot_config_data["PRIORITY_LIST"]

        self.trajectory = list()
        self.performed_commands = list()
        self.prev_action = None
        self.tree = Tree()

        """ DATA ANALYSIS """
        self.maze_number = 1  # Each maze must have a number to be identified, change this number if the maze changes
        self.maze_name = "Maze" + "_" + str(self.maze_number) + "_" + Compass.compass_list_to_concat_string(
            self.priority_list)
        self.time_to_solve = 0  # solving time
        self.number_of_nodes = 1
        self.number_of_dead_end = 0

    def virtual_destructor(self):
        self.__class_logger.log("CONTROLLER STOPPED", "green", italic=True)

    def write_data_analysis(self):
        priority_list = Compass.compass_list_to_string_comma_sep(self.priority_list)
        CFG.write_data_analysis(self.maze_name,
                                self.time_to_solve,
                                self.tree.build_tree_dict(),
                                self.number_of_nodes,
                                self.number_of_dead_end,
                                self.performed_commands,
                                self.trajectory,
                                self.INTELLIGENCE,
                                priority_list if self.INTELLIGENCE == "low" else "variable"
                                )

    def read_sensors(self):
        msg = self.__pubsub.get_message()
        try:
            if msg["type"] == 'message':
                key = msg["data"]

                values = str(self.__redis.get(key))
                read_values = json.loads(values)

                RedisData.Value.Machine.on_values(read_values)

        except TypeError:
            pass

    def _send_motors_cmd(self, v1, v2, v3, v4):
        RedisData.Value.Motor.on_values(v1, v2, v3, v4)

        if RedisData.Value.Motor.changed:
            self.send_command(RedisData.Key.Motor, RedisData.Value.Motor.values)

    def send_command(self, key, data):
        self.__redis.set(key, data)
        self.__redis.publish(RedisData.Topic.Controller, key)

    def algorithm(self):
        """ Algorithm used to explore and solve the maze """

        self.__class_logger.log(" >>>>>> NEW ALGORITHM CYCLE <<<<<< ", "green", newline=True)

        if self.goal_reached():
            return True

        # SENSE
        self.read_sensors()
        self.left_values.append(RedisData.Value.Machine.left())
        self.front_values.append(RedisData.Value.Machine.front())
        self.right_values.append(RedisData.Value.Machine.right())

        # THINK
        actions = self.control_policy()
        action = self.decision_making_policy(actions)

        if Logger.is_loggable(self.LOG_SEVERITY, "low"):
            self.__class_logger.log(f"--MODE: {self.mode}")
            self.__class_logger.log(f"--ACTIONS: {actions}")
            self.__class_logger.log(f"--ACTION: {action}")
            self.__class_logger.log(f"--CURRENT NODE: {self.tree.current}")

        # Update of the tree
        self.update_tree(actions, action)

        if Logger.is_loggable(self.LOG_SEVERITY, "low"):
            self.__class_logger.log("--CURRENT TREE:", "gray")
            self.__class_logger.log(f"{self.tree.build_tree_dict()}", "gray", noheader=True)
            self.__class_logger.log(f"--CURRENT NODE: {self.tree.current}", "gray", newline=True)
            self.__class_logger.log(f"--Available actions: {actions}", "green")

        if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
            self.__class_logger.log(f"--(STATE, POSITION): ({self._state}, {self._position})", "gray")
            self.__class_logger.log(f"--Performing action: {action}", "gray")

        if action is None:
            self.__class_logger.log("NO ACTION AVAILABLE!", "dkred", newline=True)
            self.__class_logger.log(" >>>>  EXITING  <<<< ", "dkred", italic=True)
            self.virtual_destructor()
            exit(-1)

        # ACT
        performed = self.do_action(action)

        if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
            self.__class_logger.log(f"--(STATE, POSITION): ({self._state}, {self._position})", "gray")

        if performed and self.prev_action != action:
            self.performed_commands.append(action)
            if action in self.priority_list:
                self.trajectory.append(action)
            self.prev_action = action

        return False

    def update_tree(self, actions, action_chosen):
        """
         This method is used to update the tree of the maze accordingly to the actions and the current state
         of the robot.
         The update is done if only if the robot is in SENSING mode and the actions and action_chosen type must
            be a Compass
         or a list of Compass elements respectively.
        """

        """ if not actions:
            print("UPDATE_TREE NO ACTIONS")
            exit(-1)
        """

        if not self._state == State.SENSING:
            return
        if isinstance(action_chosen, Command):
            return

        if self.mode == Mode.EXPLORING:
            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log("*** UPDATING TREE (MODE: EXPLORING) ***", "gray", newline=True)

            """ 
            Different actions returned by Control Policy.
            In this section are added the nodes of the tree based on the available actions, 
            updating also the current node as EXPLORED 
            """
            for action in actions:
                dict_ = f_r_l_b_to_compass(RedisData.Value.Machine.z_axis())
                if dict_["FRONT"] == action:
                    node = Node("M_" + self.tree.generate_node_id(), action)
                    self.tree.append(node, DIRECTION.MID)
                    self.tree.regress()
                    self.number_of_nodes += 1
                    if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                        self.__class_logger.log("ADDED MID", "dkgreen")
                if dict_["LEFT"] == action:
                    node = Node("L_" + self.tree.generate_node_id(), action)
                    self.tree.append(node, DIRECTION.LEFT)
                    self.tree.regress()
                    self.number_of_nodes += 1
                    if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                        self.__class_logger.log("ADDED LEFT", "dkgreen")
                if dict_["RIGHT"] == action:
                    node = Node("R_" + self.tree.generate_node_id(), action)
                    self.tree.append(node, DIRECTION.RIGHT)
                    self.tree.regress()
                    self.number_of_nodes += 1
                    if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                        self.__class_logger.log("ADDED RIGHT", "dkgreen")

            self.tree.current.set_type(Type.EXPLORED)

            """ 
            Only one action that has been decided by DMP. 
            In this section it is updated the current node of the tree based on the action chosen 
            """
            dict_ = f_r_l_b_to_compass(RedisData.Value.Machine.z_axis())
            if dict_["FRONT"] == action_chosen:
                self.tree.set_current(self.tree.current.mid)
            elif dict_["LEFT"] == action_chosen:
                self.tree.set_current(self.tree.current.left)
            elif dict_["RIGHT"] == action_chosen:
                self.tree.set_current(self.tree.current.right)

        elif self.mode == Mode.ESCAPING:
            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
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
                if Logger.is_loggable(self.LOG_SEVERITY, "low"):
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
                if Logger.is_loggable(self.LOG_SEVERITY, "low"):
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
                if Logger.is_loggable(self.LOG_SEVERITY, "low"):
                    self.__class_logger.log("No leaf or DEAD END children", "yellow+", italic=True)

                if self.tree.current.has_left and self.tree.current.left.action == action_chosen:
                    cur = self.tree.current.left
                elif self.tree.current.has_mid and self.tree.current.mid.action == action_chosen:
                    cur = self.tree.current.mid
                elif self.tree.current.has_right and self.tree.current.right.action == action_chosen:
                    cur = self.tree.current.right
                else:
                    if Logger.is_loggable(self.LOG_SEVERITY, "low"):
                        self.__class_logger.log("!!! ESCAPING ERROR UPDATING CURRENT !!!", "dkred", True, True)
                        self.__class_logger.log(" >>>>  EXITING  <<<< ", "red", italic=True)
                    exit(-1)

            self.tree.set_current(cur)

    def control_policy(self) -> list:
        """
        Accordingly to the values of the sensors, the state of the robot and the tree of the maze,
        it returns a set of actions that the robot can perform but only one of these can be executed effectively
        """

        actions = list()

        left = RedisData.Value.Machine.left()
        front = RedisData.Value.Machine.front()
        right = RedisData.Value.Machine.right()
        ori = RedisData.Value.Machine.z_axis()

        # Sto in RUNNING e l'albero non viene aggiornato
        """if self.mode == Mode.ESCAPING and self.tree.current.type == Type.OBSERVED:
            self.mode = Mode.EXPLORING"""

        if self._state == State.STARTING and self._position == Position.INITIAL:
            if left is not None and right is not None:
                self._position = Position.CORRIDOR
            elif left is None or right is None:
                self._position = Position.JUNCTION
            actions.insert(0, Command.START)

        elif self._state == State.ROTATING:
            actions.insert(0, self.performed_commands[len(self.performed_commands) - 1])

        elif self._state == State.STOPPED or \
                self._state == State.SENSING or \
                (self._state == State.STARTING and not self._position == Position.INITIAL):

            if self._state == State.STARTING:
                self._state = State.SENSING

            if self.mode == Mode.EXPLORING:
                if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                    self.__class_logger.log("Control policy EXPLORING", "gray")

                if front is None:
                    action = f_r_l_b_to_compass(ori)["FRONT"]
                    actions.insert(0, action)
                if left is None:
                    action = f_r_l_b_to_compass(ori)["LEFT"]
                    actions.insert(0, action)
                if right is None:
                    action = f_r_l_b_to_compass(ori)["RIGHT"]
                    actions.insert(0, action)

                if not actions:
                    action = negate_compass(self.tree.current.action)
                    actions.insert(0, action)
                    self.mode = Mode.ESCAPING
                    self._state = State.SENSING

            elif self.mode == Mode.ESCAPING:
                if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                    self.__class_logger.log("Control policy ESCAPING", "gray")

                if self.tree.current.left is not None and self.tree.current.left.type == Type.OBSERVED:
                    action = self.tree.current.left.action
                    actions.insert(0, action)
                if self.tree.current.mid is not None and self.tree.current.mid.type == Type.OBSERVED:
                    action = self.tree.current.mid.action
                    actions.insert(0, action)
                if self.tree.current.right is not None and self.tree.current.right.type == Type.OBSERVED:
                    action = self.tree.current.right.action
                    actions.insert(0, action)

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

                # Coming back, regressing
                if not actions:
                    action = negate_compass(self.tree.current.action)
                    actions.insert(0, action)

                if not actions:
                    if Logger.is_loggable(self.LOG_SEVERITY, "low"):
                        self.__class_logger.log("NO OBSERVED NO EXPLORED NO ACTIONS", "dkred", True, True)
                        self.__class_logger.log(" >>>>  EXITING  <<<< ", "dkred", italic=True)
                    exit(-1)

        elif self._state == State.RUNNING:

            # Switch da ESCAPING A EXPLORING, messo qui perché se sto in RUNNING l'albero non viene aggiornato
            # in update_tree (per aggiornarlo lo stato deve essere in SENSING).
            # Evito così che vengano aggiunti dei nodi duplicati non voluti.

            if self.mode == Mode.ESCAPING and self.tree.current.type == Type.OBSERVED:
                self.mode = Mode.EXPLORING

            if self._position == Position.CORRIDOR:
                if left is None or right is None:
                    actions.insert(0, Command.GO_TO_JUNCTION)
                elif front is None or front > self.SAFE_DISTANCE:
                    actions.insert(0, Command.RUN)
                elif front <= self.SAFE_DISTANCE:
                    actions.insert(0, Command.STOP)

            elif self._position == Position.JUNCTION:
                if left is not None and right is not None:
                    self._position = Position.CORRIDOR
                if front is None or front > self.SAFE_DISTANCE:
                    actions.insert(0, Command.RUN)
                elif front <= self.SAFE_DISTANCE:
                    actions.insert(0, Command.STOP)

        return actions

    def decision_making_policy(self, actions: list) -> Compass | None:
        """ Given a set of actions it decides what action the robot has to perform """

        if self.INTELLIGENCE == "mid" and self._state == State.SENSING:
            self.priority_list = random.sample(self.priority_list, 4)
            print(self.priority_list)
        elif self.INTELLIGENCE == "high":
            ...

        if not actions:
            return None

        if isinstance(actions[0], Command):
            return actions[0]

        for direction in self.priority_list:  # [ S, N, O, E ]
            for action in actions:  # [ E, O, N ]
                if direction == action:
                    return action

    def do_action(self, action):
        """
         Accordingly to the action chosen by Decision Making Policy this method translates the action and send
         a specific command to the PhysicalBody that has to perform, changing also the Robot state
        """
        if Logger.is_loggable(self.LOG_SEVERITY, "high"):
            self.__class_logger.log(" ~~~ [ACTION TIME] ~~~ ", "gray", True, True)

        if action == Command.START:
            self._send_motors_cmd(0, 0, 0, 0)

            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log(" ** COMMAND START ** ", "gray")

            return True

        # Stop
        elif action == Command.STOP:
            self._send_motors_cmd(0, 0, 0, 0)

            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log(" ** COMMAND STOP ** ", "gray")

            self._state = State.STOPPED
            return True

        # Go on
        elif action == detect_target(RedisData.Value.Machine.z_axis()) or action == Command.RUN:
            self._send_motors_cmd(self._speed, self._speed, self._speed, self._speed)

            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log(" ** COMMAND RUN ** ", "gray")

            self._state = State.RUNNING
            return True

        # Go to Junction
        elif action == Command.GO_TO_JUNCTION:
            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log(" ** JOINING THE JUNCTION ** ", "gray")

            start_time = time.time()
            time_expired = False

            while not time_expired and (RedisData.Value.Machine.front() is None or RedisData.Value.Machine.front() >
                                        self.SAFE_DISTANCE):
                self._send_motors_cmd(self._speed, self._speed, self._speed, self._speed)
                if time.time() - start_time >= self.junction_sim_time:
                    time_expired = True

            self._send_motors_cmd(0, 0, 0, 0)
            self._state = State.SENSING
            self._position = Position.JUNCTION

            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log(" ** STATE SENSING ARISE ** ", "gray")

            time.sleep(0.5)
            return True

        else:
            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log(" ** COMMAND ROTATE ** ", "gray")

            self._send_motors_cmd(0, 0, 0, 0)
            self.rotate_to_final_g(self._rot_speed, action.value)
            self._send_motors_cmd(0, 0, 0, 0)
            self._state = State.ROTATING

            return True

    """ INTELLIGENCE """

    def intelligent_priority_list(self):
        ...
        print(self.priority_list)

    def rotate_to_final_g(self, vel, final_g):
        """ Rotate function that rotates the robot until it reaches final_g """
        self._send_motors_cmd(0, 0, 0, 0)

        init_g = RedisData.Value.Machine.z_axis()
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)

        self.__do_rotation(vel=vel, c=c, degrees=degrees, final_g=final_g)

        self._send_motors_cmd(0, 0, 0, 0)

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

        self._send_motors_cmd(0, 0, 0, 0)
        self.__rotate(vel, c, degrees)

        ok, curr_g, limit_range = self.check_orientation(final_g)

        if not ok:
            ok, it = self.adjust_orientation(final_g)

        # CRITICAL CASE
        if it == self.OR_MAX_ATTEMPT:
            if Logger.is_loggable(self.LOG_SEVERITY, "low"):
                self.__class_logger.log(" ** MAX ATTEMPTS REACHED ** ", "dkred", True, True)
                self.__class_logger.log(" >>>>  EXITING  <<<< ", "dkred", italic=True)
            exit(-1)

    def __rotate(self, vel, c: Clockwise, degrees):
        """
        Function that given vel, Clockwise and rotation degrees computes
        the rotation of the Robot around the z axis
        The orientation of the robot must reach the interval [degrees - delta, degrees + delta]
        """
        degrees = abs(degrees)
        init_g = RedisData.Value.Machine.z_axis()

        delta = 0.8
        stop = False
        archived = False
        performed_deg = 0.0

        while not stop:
            curr_g = RedisData.Value.Machine.z_axis()

            if c == Clockwise.RIGHT:
                self._send_motors_cmd(-vel, vel, -vel, vel)
            elif c == Clockwise.LEFT:
                self._send_motors_cmd(vel, -vel, vel, -vel)

            performed_deg_temp = self.compute_performed_degrees(c, init_g, curr_g)

            """ 
            Check if there was an unintended move in the opposite direction. 
            The degrees performed in this case would be > 300 and it is not considered (continue)
            """

            if performed_deg_temp > 300:
                continue

            performed_deg = performed_deg_temp

            if degrees - delta < performed_deg < degrees + delta:
                self._send_motors_cmd(0, 0, 0, 0)
                archived = True
                stop = True
            elif performed_deg > degrees + delta:
                self._send_motors_cmd(0, 0, 0, 0)
                archived = False
                stop = True

        self._send_motors_cmd(0, 0, 0, 0)
        return archived, init_g, performed_deg, degrees

    def check_orientation(self, final_g, delta=2):
        """
        This method is used to check if the orientation of the robot is correct
        The orientation of the robot must reach a specific interval according to two cases:
        1) The final_g is 180 or -180 (first if)
        2) Otherwise other intervals (second if)
        """
        if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
            self.__class_logger.log(" ** ORIENTATION CHECKING ** ", "gray", True, True)

        curr_g = RedisData.Value.Machine.z_axis()

        ok = False
        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if curr_g < limit_g_sx or curr_g > limit_g_dx:
                if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                    self.__class_logger.log(" ~~ perfect ~~ ", "green")
                ok = True
            else:
                if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                    self.__class_logger.log(" !! bad orientation !! ", "red")
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= curr_g <= limit_g_sx:
                if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                    self.__class_logger.log(" ~~ perfect ~~ ", "green")

                ok = True
            else:
                if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                    self.__class_logger.log(" !! bad orientation !! ", "red")

        if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
            self.__class_logger.log(
                f" >> curr state of the sensors: "
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

        if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
            self.__class_logger.log(" ** ADJUSTING ORIENTATION ** ", "gray", True, True)

        self._send_motors_cmd(0, 0, 0, 0)

        ok = False
        it = 0

        while not ok and it < self.OR_MAX_ATTEMPT:
            curr_g = RedisData.Value.Machine.z_axis()

            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)

            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log(f" --ATTEMPT: {it + 1} / {self.OR_MAX_ATTEMPT}", "gray")
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

        if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
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
            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log(f" >> Rotate clockwise (RIGHT) of {abs(round_v(smallest))}°", "gray")

            c = Clockwise.RIGHT
        else:
            if Logger.is_loggable(self.LOG_SEVERITY, "mid"):
                self.__class_logger.log(f" >> Rotate anti-clockwise (LEFT) of {abs(round_v(smallest))}°", "gray")

            c = Clockwise.LEFT

        return smallest, c

    def goal_reached(self) -> bool:
        """
        It is used to check if the robot has found the exit of the maze
        The exit of the maze is identified using the gate
        It updates the last node of the tree as FINAL node
        """

        if RedisData.Value.Machine.gate():
            self.tree.current.set_type(Type.FINAL)
            self.__class_logger.log(" :D SO HAPPY :D ", "green", True, True)
            self.__class_logger.log(" >> MAZE SOLVED << ", "green", italic=True)
            self.__class_logger.log(" ~~ WATCHING THE LIGHTS ~~ ", "green", True, True)
            self.__class_logger.log(" ~~ SAYING GOODBYE TO DEAD END CHILDREN :( ~~ ", "green", True)
            self.__class_logger.log(" ~~ THANKS TO DEVELOPERS THAT HAVE DONE THIS  ~~ ", "green", True)
            self.__class_logger.log(" ^^ FLYING TO THE HEAVEN ^^ ", "green", True, True)
            print("\n")
            print("Tree: ", self.tree.build_tree_dict(), "\n")
            print("Performed commands: ", self.performed_commands, "\n")
            print("Trajectory: ", self.trajectory, "\n")

            return True

        return False

    def update_cfg(self):
        """
        Updates the values of the config file since it can be modified also during the execution of the algorithm
        """
        self._speed = CFG.robot_conf_data()["SPEED"]
        self._rot_speed = CFG.robot_conf_data()["ROT_SPEED"]
        self.OR_MAX_ATTEMPT = CFG.robot_conf_data()["MAX_ATTEMPTS"]
        self.SAFE_DISTANCE = CFG.robot_conf_data()["SAFE_DIST"]
        self.LOG_SEVERITY = CFG.logger_data()["SEVERITY"]
