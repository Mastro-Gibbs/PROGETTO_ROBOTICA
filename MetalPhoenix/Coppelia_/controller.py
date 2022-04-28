import time
from time import sleep
from math import pi
from physical_body import PhysicalBody
from utility import StdoutLogger, Compass, \
    LIFOStack, f_r_l_b_to_compass, negate_compass, normalize_angle, round_v, Clockwise, detect_target
from tree import Tree, Node, WAY, Type
from enum import Enum

OR_MAX_ATTEMPT = 10
SAFE_DISTANCE = 0.22  # aka 22cm

NODE_ID = "n"
NODE_COUNT = 0


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
TO DO:
1) Gestire meglio comandi per la ROTAZIONE
2) Gestire meglio switch da ESCAPING a EXPLORING e viceversa
3) Togliere ambiguità update tree

"""

"""
ESCAPING
Tornare indietro quando c'è un dead end. Navigo a ritroso e verifico se i figli del nodo corrente sono nodi OBSERVED 
o EXPLORED. Nodi OBSERVED hanno priorità più alta ad essere scelti rispetto a quelli EXPLORED. 
Se non ci sono né OBSERVED né EXPLORED allora il nodo successivo è il parent di quello corrente.
"""


class Controller:
    def __init__(self):

        self.__class_logger = StdoutLogger(class_name="Controller", color="cyan")

        self._body = PhysicalBody()

        self._state = State.STARTING
        self._position = Position.INITIAL
        self.mode = Mode.EXPLORING
        # self._state_position = list()

        self._body.stop()

        self._speed = 10
        self._rot_speed = 8

        self._speed_m_on_sec = self._speed * 0.25 / 5
        # Tempo che serve per posizionarsi al centro di una giunzione
        self.junction_sim_time = 0.25 / self._speed_m_on_sec

        self.orientation = None
        self.left_value = None
        self.front_value = None
        self.right_value = None
        self.back_value = None

        self.front_values = list()
        self.left_values = list()
        self.right_values = list()

        self.target = 0

        self.goal_reached = False

        self.priority_list = [Compass.NORD, Compass.OVEST, Compass.EST, Compass.SUD]
        # self.priority_list = [Compass.SUD, Compass.OVEST, Compass.EST, Compass.NORD]
        # self.priority_list = [Compass.NORD, Compass.OVEST, Compass.SUD, Compass.EST]
        # self.priority_list = [Compass.NORD, Compass.SUD, Compass.OVEST, Compass.EST]
        # self.priority_list = [Compass.NORD, Compass.SUD, Compass.EST, Compass.OVEST]
        # self.priority_list = [Compass.NORD, Compass.EST, Compass.SUD, Compass.OVEST]
        # self.priority_list = [Compass.NORD, Compass.EST, Compass.OVEST, Compass.SUD]

        self.trajectory = list()
        self.performed_commands = list()
        self.tree = Tree()

    def algorithm(self):
        prev_action = ""

        while not self.goal_reached:

            # SENSE
            self.read_sensors()
            self.left_values.append(self.left_value)
            self.front_values.append(self.front_value)
            self.right_values.append(self.right_value)

            print("NEW CYCLE")

            # THINK
            actions = self.control_policy()

            print("MODE", self.mode)
            print("Current", self.tree.current)
            print("ACTIONS", actions)

            # Update tree adding the children if only if the actions are correct (namely the robot is in sensing mode)
            self.update_tree(actions)

            # THINK
            action = self.decision_making_policy(actions)

            # Updating tree setting the current node
            self.update_tree(action)

            print(self.tree.build_tree_dict())
            print("Current", self.tree.current)
            self.__class_logger.log(f"(STATE, POSITION): ({self._state}, {self._position})")
            self.__class_logger.log(f"Available actions: {actions}")
            self.__class_logger.log(f"Performing action: {action}")

            if action is None:
                exit(-1)

            # ACT
            performed = self.do_action(action)

            self.__class_logger.log(f"(STATE, POSITION): ({self._state}, {self._position})")

            print()
            if performed and prev_action != action:
                self.performed_commands.append(action)
                if action in self.priority_list:
                    self.trajectory.append(action)
                prev_action = action

    def update_tree(self, actions):
        """ if not actions:
            print("UPDATE_TREE NO ACTIONS")
            exit(-1)"""
        if not self._state == State.SENSING:
            return
        if isinstance(actions, Command):
            return

        if self.mode == Mode.EXPLORING:
            # Only one action, l'azione è stata decisa dalla DMP ed è di tipo Compass
            if isinstance(actions, Compass):
                print("--- 2) UPDATING TREE ---")
                action = actions
                dict_ = f_r_l_b_to_compass(self.orientation)
                if dict_["FRONT"] == action:
                    self.tree.set_current(self.tree.current.mid)
                elif dict_["LEFT"] == action:
                    self.tree.set_current(self.tree.current.left)
                elif dict_["RIGHT"] == action:
                    self.tree.set_current(self.tree.current.right)

                print(self.tree.current)

            else:
                print("--- 1) UPDATING TREE ---")
                print(self.tree.current)

                for action in actions:
                    dict_ = f_r_l_b_to_compass(self.orientation)
                    if dict_["FRONT"] == action:
                        node = Node("M_" + generate_node_id(), action)
                        self.tree.append(node, WAY.MID)
                        self.tree.regress()
                        print("ADDED MID")
                    if dict_["LEFT"] == action:
                        node = Node("L_" + generate_node_id(), action)
                        self.tree.append(node, WAY.LEFT)
                        self.tree.regress()
                        print("ADDED LEFT")
                    if dict_["RIGHT"] == action:
                        node = Node("R_" + generate_node_id(), action)
                        self.tree.append(node, WAY.RIGHT)
                        self.tree.regress()
                        print("ADDED RIGHT")

                self.tree.current.set_type(Type.EXPLORED)
                print(self.tree.current)

        elif self.mode == Mode.ESCAPING:
            print("Mode.ESCAPING")

            if isinstance(actions, Compass):
                print("--- 2) UPDATING TREE ---")
                action = actions

                if negate_compass(self.tree.current.action) == action:
                    print("BACK regress")
                    self.tree.regress()
                    print(self.tree.current)
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
                    print("ESCAPING ERROR UPDATING CURRENT")
                    exit(-1)

                self.tree.set_current(cur)
                print(self.tree.current)

            else:
                print("--- 1) UPDATING TREE ---")
                print(self.tree.current)

                # The node is a leaf
                if self.tree.current.is_leaf:
                    self.tree.current.set_type(Type.DEAD_END)
                    print("DEAD END LEAF")

                # The children are all DEAD END
                elif ((self.tree.current.has_left and self.tree.current.left.type == Type.DEAD_END) or
                      self.tree.current.left is None) and \
                        ((self.tree.current.has_right and self.tree.current.right.type == Type.DEAD_END)
                         or self.tree.current.right is None) and \
                        ((self.tree.current.has_mid and self.tree.current.mid.type == Type.DEAD_END)
                         or self.tree.current.mid is None):
                    self.tree.current.set_type(Type.DEAD_END)
                    print("DEAD END NOT LEAF")

                else:
                    print("Nodo senza figli DEAD END, aggiorno il corrente nel prossimo update tree")

                """elif self.tree.current.is_root:
                    ...
                    

                    # if the children are all dead end the maze cannot be solved"""

    def control_policy(self) -> list:
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
                print("Control policy EXPLORING")
                if front is None:
                    # verificare se il nodo è OSSERVATO e quindi non ESPLORATO
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
                print("Control policy ESCAPING")

                if self.tree.current.left is not None and self.tree.current.left.type == Type.OBSERVED:
                    action = self.tree.current.left.action
                    actions.insert(0, action)
                if self.tree.current.mid is not None and self.tree.current.mid.type == Type.OBSERVED:
                    action = self.tree.current.mid.action
                    actions.insert(0, action)
                if self.tree.current.right is not None and self.tree.current.right.type == Type.OBSERVED:
                    action = self.tree.current.right.action
                    actions.insert(0, action)

                # Se non ci sono OBSERVED scegliere EXPLORED
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
                    print("NO OBSERVED NO EXPLORED NO ACTIONS")
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

        if action == Command.START:
            self._body.stop()
            print("STARTING")
            # segnalare con un suono che si è avviato
            return True

        # Stop
        elif action == Command.STOP:
            self._body.stop()
            self._state = State.STOPPED
            return True

        # Go on
        elif action == detect_target(self.orientation) or action == Command.RUN:
            self._body.move_forward(self._speed)
            self._state = State.RUNNING
            return True

        # Go to Junction
        elif action == Command.GO_TO_JUNCTION:
            print("Positioning in the junction...")
            start_time = time.time()
            time_expired = False

            while not time_expired and (self._body.get_proxF() is None or self._body.get_proxF() > SAFE_DISTANCE):
                self._body.move_forward(self._speed)
                if time.time() - start_time >= self.junction_sim_time:
                    time_expired = True

            self._body.stop()
            self._state = State.SENSING
            self._position = Position.JUNCTION
            time.sleep(0.5)
            return True

        # Rotate (DA GESTIRE MEGLIO)
        else:
            self._body.stop()
            self.rotate_to_final_g(self._rot_speed, action.value)
            self._body.stop()
            # self._state = State.STOPPED
            self._state = State.ROTATING
            # self._position = Position.JUNCTION
            return True

    def read_sensors(self):
        self.left_value = self._body.get_proxL()
        self.front_value = self._body.get_proxF()
        self.right_value = self._body.get_proxR()
        self.back_value = self._body.get_proxB()
        self.orientation = self._body.get_orientation_deg()

    def verify_gate(self, c: Compass) -> bool:
        global OR_MAX_ATTEMPT

        it = 0
        _gate: bool = False

        _sens = 0.0
        _not_none_counter = 0

        if c == Compass.OVEST:
            _sens = self._body.get_proxL()
        elif c == Compass.EST:
            _sens = self._body.get_proxR()

        while it < OR_MAX_ATTEMPT:
            it += 1
            if _sens is None:
                _gate = True
                if c == Compass.OVEST:
                    _sens = self._body.get_proxL()
                elif c == Compass.EST:
                    _sens = self._body.get_proxR()
            else:
                _gate = False
                _not_none_counter += 1
                if _not_none_counter == 2:
                    break

        return _gate

    def rotate_to_final_g(self, vel, final_g):
        """Rotate function that rotates the robot until it reaches final_g"""
        self._body.stop()

        init_g = self._body.get_orientation_deg()
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)

        self.__do_rotation(vel=vel, c=c, degrees=degrees, final_g=final_g)

        self._body.stop()

    def __do_rotation(self, vel, c: Clockwise, degrees, final_g):
        degrees = abs(degrees)
        it = 0

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
            # DA SPIEGARE MEGLIO
            # necessario perché se c'è uno spostamento non voluto nel verso opposto va a calcolare una
            # rotazione sicuramente > di 300° che in realtà non è stata fatta!
            # IN QUESTO MODO NON CONSIDERO "ROTAZIONI" SPURIE
            # es init =  90.043° se ruotando verso LEFT si sposta a destra a va a 90.00°
            # la diff dell' orientazione dà un valore > 300 che non è valido!
            if performed_deg_temp > 300:
                # self.__class_logger.log("performed_deg > 300")
                continue
            performed_deg = performed_deg_temp
            # self.__class_logger.log(f"Performed: {performed_deg}")

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
            self.__class_logger.log(
                f"[Degrees_to_do, curr_g, final_g] = [{round_v(degrees)}, {round_v(curr_g)}, {round_v(final_g)}]", 2)

            if abs(degrees) < 6:
                self.__rotate(0.25, c, abs(degrees))
            else:
                self.__rotate(45 * pi / 180, c, abs(degrees))

            ok, curr_g, limit_range = self.check_orientation(final_g)
            it += 1

        return ok, it

    def compute_performed_degrees(self, c, init_g, curr_g):
        """Calculates the angle between init_g and curr_g that the robot performed based on the direction of rotation"""

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
        """Calculate the best (minimum) angle between init_g and final_g and how you need to rotate"""

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
            self.__class_logger.log(f"Rotate clockwise (RIGHT) of {abs(round_v(smallest))} degrees")
            c = Clockwise.RIGHT
        else:
            self.__class_logger.log(f"Rotate anti-clockwise (LEFT) of {abs(round_v(smallest))} degrees")
            c = Clockwise.LEFT

        return smallest, c
