import time
from math import pi
from enum import Enum

from physical_body import PhysicalBody
from tools.utility import Logger, Compass, f_r_l_b_to_compass, negate_compass, \
    normalize_angle, round_v, Clockwise, detect_target, CFG

from tools.tree import Tree, Node, WAY, Type

OR_MAX_ATTEMPT = CFG.controller_data()["MAX_ATTEMPTS"]
SAFE_DISTANCE = CFG.controller_data()["SAFE_DIST"]

NODE_ID = "n"
NODE_COUNT = 0

PREV_ACTION = None
LOGSEVERITY = CFG.logger_data()["SEVERITY"]


def generate_node_id() -> str:
    global NODE_COUNT
    NODE_COUNT += 1
    return NODE_ID + str(NODE_COUNT)


class State(Enum):
    STARTING = -1
    STOPPED = 0
    RUNNING = 1
    ROTATING = 2
    SENSING = 3


class Position(Enum):
    INITIAL = 0
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

"""
ESCAPING
Tornare indietro quando c'è un dead end. Navigo a ritroso e verifico se i figli del nodo corrente sono nodi OBSERVED 
o EXPLORED. Nodi OBSERVED hanno priorità più alta ad essere scelti rispetto a quelli EXPLORED. 
Se non ci sono né OBSERVED né EXPLORED allora il nodo successivo è il parent di quello corrente.
"""


class Controller:
    def __init__(self):

        self.__class_logger = Logger(class_name="Controller", color="cyan")
        self.__class_logger.set_logfile(CFG.logger_data()["CLOGFILE"])
        self.__class_logger.log(f"LOG SEVERITY: {str.upper(LOGSEVERITY)}\n", color="dkgreen")
        self.__class_logger.log("CONTROLLER LAUNCHED", color="green", italic=True)

        self._body = PhysicalBody()

        self._state = State.STARTING
        self._position = Position.INITIAL
        self.mode = Mode.EXPLORING

        self._body.stop()

        self._speed = CFG.controller_data()["SPEED"]
        self._rot_speed = CFG.controller_data()["ROT_SPEED"]

        self._speed_m_on_sec = self._speed * 0.25 / (self._speed // 5)
        # Time it takes to position in the center of a junction
        self.junction_sim_time = 0.25 / self._speed_m_on_sec

        self.orientation = None
        self.left_value = None
        self.front_value = None
        self.right_value = None
        self.gate = False

        self.front_values = list()
        self.left_values = list()
        self.right_values = list()

        self.target = 0
        self.priority_list = CFG.controller_data()["PRIORITY_LIST"]

        self.trajectory = list()
        self.performed_commands = list()
        self.tree = Tree()

    def virtual_destructor(self):
        self._body.virtual_destructor()
        self.__class_logger.log("CONTROLLER STOPPED", "green", italic=True)

    def algorithm(self):
        global PREV_ACTION
        global LOGSEVERITY

        self.__class_logger.log(" >>>>>> NEW ALGORITHM CYCLE <<<<<< ", "green", newline=True)

        if self.goal_reached():
            return True

        # SENSE
        self.read_sensors()
        self.left_values.append(self.left_value)
        self.front_values.append(self.front_value)
        self.right_values.append(self.right_value)

        # THINK
        actions = self.control_policy()

        if Logger.is_loggable(LOGSEVERITY, "low"):
            self.__class_logger.log(f"--MODE: {self.mode}")
            self.__class_logger.log(f"--ACTIONS: {actions}")
            self.__class_logger.log(f"--CURRENT NODE: {self.tree.current}")

        # Update tree adding the children if only if the robot is in sensing mode
        self.update_tree(actions)

        # THINK
        action = self.decision_making_policy(actions)

        # Updating tree setting the current node
        self.update_tree(action)

        if Logger.is_loggable(LOGSEVERITY, "low"):
            self.__class_logger.log("--CURRENT TREE:", "gray")
            self.__class_logger.log(f"{self.tree.build_tree_dict()}", "gray", noheader=True)
            self.__class_logger.log(f"--CURRENT NODE: {self.tree.current}", "gray", newline=True)
            self.__class_logger.log(f"--Available actions: {actions}", "green")

        if Logger.is_loggable(LOGSEVERITY, "mid"):
            self.__class_logger.log(f"--(STATE, POSITION): ({self._state}, {self._position})", "gray")
            self.__class_logger.log(f"--Performing action: {action}", "gray")

        if action is None:
            self.__class_logger.log("NO ACTION AVAILABLE!", "dkred", newline=True)
            self.__class_logger.log(" >>>>  EXITING  <<<< ", "dkred", italic=True)
            self.virtual_destructor()
            exit(-1)

        # ACT
        performed = self.do_action(action)

        if Logger.is_loggable(LOGSEVERITY, "mid"):
            self.__class_logger.log(f"--(STATE, POSITION): ({self._state}, {self._position})", "gray")

        if performed and PREV_ACTION != action:
            self.performed_commands.append(action)
            if action in self.priority_list:
                self.trajectory.append(action)
            PREV_ACTION = action

        return False

    def update_tree(self, actions):
        """ if not actions:
            print("UPDATE_TREE NO ACTIONS")
            exit(-1)
        """
        global LOGSEVERITY

        if not self._state == State.SENSING:
            return
        if isinstance(actions, Command):
            return

        if self.mode == Mode.EXPLORING:
            """Only one action that has been decided by DMP"""
            if isinstance(actions, Compass):
                if Logger.is_loggable(LOGSEVERITY, "mid"):
                    self.__class_logger.log("*** 2) UPDATING TREE (MODE EXPLORING) ***", "gray", newline=True)

                action = actions
                dict_ = f_r_l_b_to_compass(self.orientation)
                if dict_["FRONT"] == action:
                    self.tree.set_current(self.tree.current.mid)
                elif dict_["LEFT"] == action:
                    self.tree.set_current(self.tree.current.left)
                elif dict_["RIGHT"] == action:
                    self.tree.set_current(self.tree.current.right)

            else:
                if Logger.is_loggable(LOGSEVERITY, "mid"):
                    self.__class_logger.log("*** 1) UPDATING TREE (MODE EXPLORING) ***", "gray", newline=True)

                for action in actions:
                    dict_ = f_r_l_b_to_compass(self.orientation)
                    if dict_["FRONT"] == action:
                        node = Node("M_" + generate_node_id(), action)
                        self.tree.append(node, WAY.MID)
                        self.tree.regress()
                        if Logger.is_loggable(LOGSEVERITY, "mid"):
                            self.__class_logger.log("ADDED MID", "dkgreen")
                    if dict_["LEFT"] == action:
                        node = Node("L_" + generate_node_id(), action)
                        self.tree.append(node, WAY.LEFT)
                        self.tree.regress()
                        if Logger.is_loggable(LOGSEVERITY, "mid"):
                            self.__class_logger.log("ADDED LEFT", "dkgreen")
                    if dict_["RIGHT"] == action:
                        node = Node("R_" + generate_node_id(), action)
                        self.tree.append(node, WAY.RIGHT)
                        self.tree.regress()
                        if Logger.is_loggable(LOGSEVERITY, "mid"):
                            self.__class_logger.log("ADDED RIGHT", "dkgreen")

                self.tree.current.set_type(Type.EXPLORED)

        elif self.mode == Mode.ESCAPING:
            if isinstance(actions, Compass):
                if Logger.is_loggable(LOGSEVERITY, "mid"):
                    self.__class_logger.log("*** 2) UPDATING TREE (MODE ESCAPING) ***", "gray", newline=True)

                action = actions

                if negate_compass(self.tree.current.action) == action:
                    if Logger.is_loggable(LOGSEVERITY, "low"):
                        self.__class_logger.log(" >>>> REGRESSION <<<< ", "yellow", newline=True)
                        self.__class_logger.log(f" --CURRENT NODE: {self.tree.current}", "yellow")

                    self.tree.regress()
                    return

                # Caso in cui scelgo un OBSERVED e lo metto come corrente
                cur = None
                if self.tree.current.has_left and self.tree.current.left.action == action:
                    cur = self.tree.current.left
                elif self.tree.current.has_mid and self.tree.current.mid.action == action:
                    cur = self.tree.current.mid
                elif self.tree.current.has_right and self.tree.current.right.action == action:
                    cur = self.tree.current.right
                else:
                    if Logger.is_loggable(LOGSEVERITY, "low"):
                        self.__class_logger.log("!!! ESCAPING ERROR UPDATING CURRENT !!!", "dkred", True, True)
                        self.__class_logger.log(" >>>>  EXITING  <<<< ", "red", italic=True)
                    exit(-1)

                self.tree.set_current(cur)
                # print(self.tree.current)

            else:
                if Logger.is_loggable(LOGSEVERITY, "mid"):
                    self.__class_logger.log("*** 1) UPDATING TREE (MODE ESCAPING) ***", "gray", newline=True)
                # print(self.tree.current)

                # The node is a leaf
                if self.tree.current.is_leaf:
                    self.tree.current.set_type(Type.DEAD_END)
                    if Logger.is_loggable(LOGSEVERITY, "low"):
                        self.__class_logger.log("*** DEAD END NODE DETECTED ***", "green")

                # The children are all DEAD END
                elif ((self.tree.current.has_left and self.tree.current.left.type == Type.DEAD_END) or
                      self.tree.current.left is None) and \
                        ((self.tree.current.has_right and self.tree.current.right.type == Type.DEAD_END)
                         or self.tree.current.right is None) and \
                        ((self.tree.current.has_mid and self.tree.current.mid.type == Type.DEAD_END)
                         or self.tree.current.mid is None):
                    self.tree.current.set_type(Type.DEAD_END)

                    if Logger.is_loggable(LOGSEVERITY, "low"):
                        self.__class_logger.log("*** ALL CHILDREN ARE DEAD END NODE ***", "green")

                else:
                    if Logger.is_loggable(LOGSEVERITY, "low"):
                        self.__class_logger.log("No DEAD END children, tree'll be updated on next loop", "yellow+",
                                                italic=True)  # aka warning + debug (?)

                """elif self.tree.current.is_root:
                    ...
                    

                    # if the children are all dead end the maze cannot be solved"""

    def control_policy(self) -> list:
        global LOGSEVERITY

        actions = list()

        left = self.left_value
        front = self.front_value
        right = self.right_value
        ori = self.orientation

        # Sto in RUNNING e l'albero non viene aggiornato
        if self.mode == Mode.ESCAPING and self.tree.current.type == Type.OBSERVED:
            self.mode = Mode.EXPLORING

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
                if Logger.is_loggable(LOGSEVERITY, "mid"):
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
                if Logger.is_loggable(LOGSEVERITY, "mid"):
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

                # Se non ci sono EXPLORED tornare indietro
                if not actions:
                    action = negate_compass(self.tree.current.action)
                    actions.insert(0, action)

                if not actions:
                    if Logger.is_loggable(LOGSEVERITY, "low"):
                        self.__class_logger.log("NO OBSERVED NO EXPLORED NO ACTIONS", "dkred", True, True)
                        self.__class_logger.log(" >>>>  EXITING  <<<< ", "dkred", italic=True)
                    exit(-1)
                else:
                    ...
                    # self.mode = Mode.EXPLORING

        elif self._state == State.RUNNING:
            if self._position == Position.CORRIDOR:
                if left is None or right is None:
                    actions.insert(0, Command.GO_TO_JUNCTION)
                elif front is None or front > SAFE_DISTANCE:
                    actions.insert(0, Command.RUN)
                elif front <= SAFE_DISTANCE:
                    actions.insert(0, Command.STOP)

            elif self._position == Position.JUNCTION:
                if left is not None and right is not None:
                    self._position = Position.CORRIDOR
                if front is None or front > SAFE_DISTANCE:
                    actions.insert(0, Command.RUN)
                elif front <= SAFE_DISTANCE:
                    actions.insert(0, Command.STOP)

        return actions

    def decision_making_policy(self, actions: list) -> Compass | None:

        if not actions:
            return None

        if isinstance(actions[0], Command):
            return actions[0]

        for direction in self.priority_list:  # [ S, N, O, E ]
            for action in actions:  # [ E, O, N ]
                if direction == action:
                    return action

    def do_action(self, action):
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "high"):
            self.__class_logger.log(" ~~~ [ACTION TIME] ~~~ ", "gray", True, True)

        if action == Command.START:
            self._body.stop()

            if Logger.is_loggable(LOGSEVERITY, "mid"):
                self.__class_logger.log(" ** COMMAND START ** ", "gray")

            return True

        # Stop
        elif action == Command.STOP:
            self._body.stop()

            if Logger.is_loggable(LOGSEVERITY, "mid"):
                self.__class_logger.log(" ** COMMAND STOP ** ", "gray")

            self._state = State.STOPPED
            return True

        # Go on
        elif action == detect_target(self.orientation) or action == Command.RUN:
            self._body.move_forward(self._speed)

            if Logger.is_loggable(LOGSEVERITY, "mid"):
                self.__class_logger.log(" ** COMMAND RUN ** ", "gray")

            self._state = State.RUNNING
            return True

        # Go to Junction
        elif action == Command.GO_TO_JUNCTION:
            if Logger.is_loggable(LOGSEVERITY, "mid"):
                self.__class_logger.log(" ** JOINING THE JUNCTION ** ", "gray")

            start_time = time.time()
            time_expired = False

            while not time_expired and (self._body.get_proxF() is None or self._body.get_proxF() > SAFE_DISTANCE):
                self._body.move_forward(self._speed)
                if time.time() - start_time >= self.junction_sim_time:
                    time_expired = True

            self._body.stop()
            self._state = State.SENSING
            self._position = Position.JUNCTION

            if Logger.is_loggable(LOGSEVERITY, "mid"):
                self.__class_logger.log(" ** STATE SENSING ARISE ** ", "gray")

            time.sleep(0.5)
            return True

        else:
            if Logger.is_loggable(LOGSEVERITY, "mid"):
                self.__class_logger.log(" ** COMMAND ROTATE ** ", "gray")

            self._body.stop()
            self.rotate_to_final_g(self._rot_speed, action.value)
            self._body.stop()
            self._state = State.ROTATING

            return True

    def read_sensors(self):
        self.left_value = self._body.get_proxL()
        self.front_value = self._body.get_proxF()
        self.right_value = self._body.get_proxR()
        self.orientation = self._body.get_orientation_deg()

    def rotate_to_final_g(self, vel, final_g):
        """ Rotate function that rotates the robot until it reaches final_g """
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

        global LOGSEVERITY

        degrees = abs(degrees)
        it = 0

        self._body.stop()
        self.__rotate(vel, c, degrees)

        ok, curr_g, limit_range = self.check_orientation(final_g)

        if not ok:
            ok, it = self.adjust_orientation(final_g)

        # CRITICAL CASE
        if it == OR_MAX_ATTEMPT:
            if Logger.is_loggable(LOGSEVERITY, "low"):
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

            """ 
            Check if there was an unintended move in the opposite direction. 
            The degrees performed in this case would be > 300
            The value is not considered (continue)
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
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "mid"):
            self.__class_logger.log(" ** ORIENTATION CHECKING ** ", "gray", True, True)

        curr_g = self._body.get_orientation_deg()

        ok = False
        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if curr_g < limit_g_sx or curr_g > limit_g_dx:
                if Logger.is_loggable(LOGSEVERITY, "mid"):
                    self.__class_logger.log(" ~~ perfect ~~ ", "green")
                ok = True
            else:
                if Logger.is_loggable(LOGSEVERITY, "mid"):
                    self.__class_logger.log(" !! bad orientation !! ", "red")
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= curr_g <= limit_g_sx:
                if Logger.is_loggable(LOGSEVERITY, "mid"):
                    self.__class_logger.log(" ~~ perfect ~~ ", "green")

                ok = True
            else:
                if Logger.is_loggable(LOGSEVERITY, "mid"):
                    self.__class_logger.log(" !! bad orientation !! ", "red")

        if Logger.is_loggable(LOGSEVERITY, "mid"):
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
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "mid"):
            self.__class_logger.log(" ** ADJUSTING ORIENTATION ** ", "gray", True, True)

        self._body.stop()

        ok = False
        it = 0

        while not ok and it < OR_MAX_ATTEMPT:
            curr_g = self._body.get_orientation_deg()

            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)

            if Logger.is_loggable(LOGSEVERITY, "mid"):
                self.__class_logger.log(f" --ATTEMPT: {it + 1} / {OR_MAX_ATTEMPT}", "gray")
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
        global LOGSEVERITY

        if Logger.is_loggable(LOGSEVERITY, "mid"):
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
            if Logger.is_loggable(LOGSEVERITY, "mid"):
                self.__class_logger.log(f" >> Rotate clockwise (RIGHT) of {abs(round_v(smallest))}°", "gray")

            c = Clockwise.RIGHT
        else:
            if Logger.is_loggable(LOGSEVERITY, "mid"):
                self.__class_logger.log(f" >> Rotate anti-clockwise (LEFT) of {abs(round_v(smallest))}°", "gray")

            c = Clockwise.LEFT

        return smallest, c

    def goal_reached(self) -> bool:
        """
        It is used to check if the robot has found the exit of the maze
        The exit of the maze is identified using the gate
        It updates the last node of the tree as FINAL node
        """
        global LOGSEVERITY

        if self._body.get_gate():

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
        """ Updates the values of the config file since it can be modified also during the execution of the algorithm """
        global OR_MAX_ATTEMPT
        global SAFE_DISTANCE
        global LOGSEVERITY

        self._speed = CFG.controller_data()["SPEED"]
        self._rot_speed = CFG.controller_data()["ROT_SPEED"]
        OR_MAX_ATTEMPT = CFG.controller_data()["MAX_ATTEMPTS"]
        SAFE_DISTANCE = CFG.controller_data()["SAFE_DIST"]
        LOGSEVERITY = CFG.logger_data()["SEVERITY"]

        PhysicalBody.update_cfg()

