import time

from math import pi

from lib.ctrllib.utility import generate_node_id, f_r_l_b_to_compass, normalize_angle, negate_compass, detect_target
from lib.ctrllib.utility import Logger, CFG

from lib.ctrllib.tree import Tree, Node

from lib.ctrllib.enums import Compass, Clockwise, WAY, Type, Command, Position, Mode, State

from redis import Redis


OR_MAX_ATTEMPT = CFG.controller_data()["MAX_ATTEMPTS"]
SAFE_DISTANCE = CFG.controller_data()["SAFE_DIST"]
LOGSEVERITY = CFG.logger_data()["SEVERITY"]
B_TOPIC = CFG.redis_data()["B_TOPIC"]
C_TOPIC = CFG.redis_data()["C_TOPIC"]
MOTORS_KEY = CFG.redis_data()["MOTORS_KEY"]

GATE = False
PREV_ACTION = None
OLD_CMD = [None, None, None, None]


class Controller:
    def __init__(self):
        """<!-- REDIS SECTION -->"""
        self.__redis = Redis(host=CFG.redis_data()["HOST"], port=CFG.redis_data()[
                             "PORT"], decode_responses=True)
        self.__pubsub = self.__redis.pubsub()
        self.__pubsub.subscribe(B_TOPIC)

        self._state = State.STARTING
        self._position = Position.INITIAL
        self.mode = Mode.EXPLORING

        self.send_command(0, 0, 0, 0)

        self._speed = CFG.controller_data()["SPEED"]
        self._rot_speed = CFG.controller_data()["ROT_SPEED"]

        try:
            self._speed_m_on_sec = self._speed * 0.25 / (self._speed // 5)
        except ZeroDivisionError:
            self._speed_m_on_sec = 1
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

        self.priority_list = [Compass.NORD,
                              Compass.OVEST, Compass.EST, Compass.SUD]

        self.trajectory = list()
        self.performed_commands = list()
        self.tree = Tree()

    def virtual_destructor(self):
        pass

    def algorithm(self):
        global PREV_ACTION
        global LOGSEVERITY

        # SENSE
        self.read_sensors()

        self.left_values.append(self.left_value)
        self.front_values.append(self.front_value)
        self.right_values.append(self.right_value)

        # THINK
        actions = self.control_policy()

        # Update tree adding the children if only if the actions are correct (namely the robot is in sensing mode)
        self.update_tree(actions)

        # THINK
        action = self.decision_making_policy(actions)

        # Updating tree setting the current node
        self.update_tree(action)

        if action is None:
            self.virtual_destructor()
            exit(-1)

        # ACT
        performed = self.do_action(action)

        if performed and PREV_ACTION != action:
            self.performed_commands.append(action)
            if action in self.priority_list:
                self.trajectory.append(action)
            PREV_ACTION = action

    def update_tree(self, actions):
        global LOGSEVERITY

        if not self._state == State.SENSING:
            return
        if isinstance(actions, Command):
            return

        if self.mode == Mode.EXPLORING:
            # Only one action, l'azione è stata decisa dalla DMP ed è di tipo Compass
            if isinstance(actions, Compass):

                action = actions
                dict_ = f_r_l_b_to_compass(self.orientation)
                if dict_["FRONT"] == action:
                    self.tree.set_current(self.tree.current.mid)
                elif dict_["LEFT"] == action:
                    self.tree.set_current(self.tree.current.left)
                elif dict_["RIGHT"] == action:
                    self.tree.set_current(self.tree.current.right)

            else:
                for action in actions:
                    dict_ = f_r_l_b_to_compass(self.orientation)
                    if dict_["FRONT"] == action:
                        node = Node("M_" + generate_node_id(), action)
                        self.tree.append(node, WAY.MID)
                        self.tree.regress()
                    if dict_["LEFT"] == action:
                        node = Node("L_" + generate_node_id(), action)
                        self.tree.append(node, WAY.LEFT)
                        self.tree.regress()
                    if dict_["RIGHT"] == action:
                        node = Node("R_" + generate_node_id(), action)
                        self.tree.append(node, WAY.RIGHT)
                        self.tree.regress()

                self.tree.current.set_type(Type.EXPLORED)

        elif self.mode == Mode.ESCAPING:
            if isinstance(actions, Compass):
                action = actions

                if negate_compass(self.tree.current.action) == action:
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
                    exit(-1)

                self.tree.set_current(cur)

            else:
                # The node is a leaf
                if self.tree.current.is_leaf:
                    self.tree.current.set_type(Type.DEAD_END)

                # The children are all DEAD END
                elif ((self.tree.current.has_left and self.tree.current.left.type == Type.DEAD_END) or
                      self.tree.current.left is None) and \
                        ((self.tree.current.has_right and self.tree.current.right.type == Type.DEAD_END)
                         or self.tree.current.right is None) and \
                        ((self.tree.current.has_mid and self.tree.current.mid.type == Type.DEAD_END)
                         or self.tree.current.mid is None):
                    self.tree.current.set_type(Type.DEAD_END)

                else:
                    # No DEAD END children, tree'll be updated on next loop
                    pass

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
            actions.insert(
                0, self.performed_commands[len(self.performed_commands) - 1])

        elif self._state == State.STOPPED or \
                self._state == State.SENSING or \
                (self._state == State.STARTING and not self._position == Position.INITIAL):

            if self._state == State.STARTING:
                self._state = State.SENSING

            if self.mode == Mode.EXPLORING:
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
                    exit(-1)
                else:
                    pass

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
            self.send_command(0, 0, 0, 0)

            # segnalare con un suono che si è avviato
            return True

        # Stop
        elif action == Command.STOP:
            self.send_command(0, 0, 0, 0)

            self._state = State.STOPPED
            return True

        # Go on
        elif action == detect_target(self.orientation) or action == Command.RUN:
            self.send_command(self._speed, self._speed,
                              self._speed, self._speed)

            self._state = State.RUNNING
            return True

        # Go to Junction
        elif action == Command.GO_TO_JUNCTION:

            start_time = time.time()
            time_expired = False

            self.read_sensors()
            while not time_expired and (self.front_value is None
                                        or self.front_value > SAFE_DISTANCE):
                self.read_sensors()
                self.send_command(self._speed, self._speed,
                                  self._speed, self._speed)
                if time.time() - start_time >= self.junction_sim_time:
                    time_expired = True

            self.send_command(0, 0, 0, 0)
            self._state = State.SENSING
            self._position = Position.JUNCTION

            time.sleep(0.5)
            return True

        # Rotate (DA GESTIRE MEGLIO)
        else:
            self.send_command(0, 0, 0, 0)
            self.rotate_to_final_g(self._rot_speed, action.value)
            self.send_command(0, 0, 0, 0)
            self._state = State.ROTATING

            return True

    def read_sensors(self):
        global GATE
        msg = self.__pubsub.get_message()

        try:
            if msg["type"] == 'message':
                key = msg["data"]

                values = str(self.__redis.get(key))
                read_values = values.split(';')

                self.left_value = float(
                    read_values[0]) if read_values[0] != 'None' else None
                self.front_value = float(
                    read_values[1]) if read_values[1] != 'None' else None
                self.right_value = float(
                    read_values[2]) if read_values[2] != 'None' else None
                self.back_value = float(
                    read_values[3]) if read_values[3] != 'None' else None
                if not GATE:
                    GATE = True if read_values[3] == 'True' else False
                self.orientation = float(
                    read_values[5]) if read_values[5] != 'None' else None

        except TypeError:
            pass

    def send_command(self, v1, v2, v3, v4):
        global MOTORS_KEY
        global C_TOPIC
        global OLD_CMD

        vels = [v1, v2, v3, v4]

        if vels[0] == OLD_CMD[0] and vels[1] == OLD_CMD[1] and vels[2] == OLD_CMD[2] and vels[3] == OLD_CMD[3]:
            return

        OLD_CMD = [v1, v2, v3, v4]

        v1 = str(v1)
        v2 = str(v2)
        v3 = str(v3)
        v4 = str(v4)

        msg = ';'.join([v1, v2, v3, v4])

        self.__redis.set(MOTORS_KEY, msg)
        self.__redis.publish(C_TOPIC, MOTORS_KEY)

    def verify_gate(self, c: Compass) -> bool:
        global OR_MAX_ATTEMPT

        self.read_sensors()

        it = 0
        _gate: bool = False

        _sens = 0.0
        _not_none_counter = 0

        if c == Compass.OVEST:
            _sens = self.left_value
        elif c == Compass.EST:
            _sens = self.right_value

        self.read_sensors()

        while it < OR_MAX_ATTEMPT:
            it += 1
            if _sens is None:
                _gate = True
                if c == Compass.OVEST:
                    _sens = self.left_value
                elif c == Compass.EST:
                    _sens = self.right_value
            else:
                _gate = False
                _not_none_counter += 1
                if _not_none_counter == 2:
                    break

        return _gate

    def rotate_to_final_g(self, vel, final_g):
        """Rotate function that rotates the robot until it reaches final_g"""
        self.send_command(0, 0, 0, 0)

        self.read_sensors()

        init_g = self.orientation
        degrees, c = self.best_angle_and_rotation_way(init_g, final_g)

        self.__do_rotation(vel=vel, c=c, degrees=degrees, final_g=final_g)

        self.send_command(0, 0, 0, 0)

    def __do_rotation(self, vel, c: Clockwise, degrees, final_g):
        degrees = abs(degrees)
        it = 0

        self.send_command(0, 0, 0, 0)
        self.__rotate(vel, c, degrees)

        ok, curr_g, limit_range = self.check_orientation(final_g)

        if not ok:
            ok, it = self.adjust_orientation(final_g)

        if it == OR_MAX_ATTEMPT:  # CRITICAL CASE
            exit(-1)

    def __rotate(self, vel, c: Clockwise, degrees):
        """
        Function that given vel, Clockwise and rotation degrees computes
        the rotation of the Robot around the z axis
        """
        degrees = abs(degrees)
        self.read_sensors()

        init_g = self.orientation

        delta = 0.8
        stop = False
        archived = False
        performed_deg = 0.0

        while not stop:
            self.read_sensors()

            curr_g = self.orientation

            if c == Clockwise.RIGHT:
                self.send_command(-vel, vel, -vel, vel)
            elif c == Clockwise.LEFT:
                self.send_command(vel, -vel, vel, -vel)

            performed_deg_temp = self.compute_performed_degrees(
                c, init_g, curr_g)

            if performed_deg_temp > 300:
                continue
            performed_deg = performed_deg_temp

            if degrees - delta < performed_deg < degrees + delta:
                self.send_command(0, 0, 0, 0)
                archived = True
                stop = True
            elif performed_deg > degrees + delta:
                self.send_command(0, 0, 0, 0)
                archived = False
                stop = True

        self.send_command(0, 0, 0, 0)
        return archived, init_g, performed_deg, degrees

    def check_orientation(self, final_g, delta=2):
        self.read_sensors()

        curr_g = self.orientation

        ok = False
        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if curr_g < limit_g_sx or curr_g > limit_g_dx:
                ok = True
            else:
                # bad ori
                pass
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= curr_g <= limit_g_sx:
                ok = True
            else:
                # bad ori
                pass

        limit_range = [limit_g_sx, limit_g_dx]
        return ok, curr_g, limit_range

    def adjust_orientation(self, final_g):
        self.send_command(0, 0, 0, 0)

        ok = False
        it = 0

        while not ok and it < OR_MAX_ATTEMPT:
            self.read_sensors()

            curr_g = self.orientation

            degrees, c = self.best_angle_and_rotation_way(curr_g, final_g)

            if abs(degrees) < 6:
                self.__rotate(0.25, c, abs(degrees))
            else:
                self.__rotate(45 * pi / 180, c, abs(degrees))

            ok, curr_g, limit_range = self.check_orientation(final_g)
            it += 1

        return ok, it

    @staticmethod
    def compute_performed_degrees(c, init_g, curr_g):
        """Calculates the angle between init_g and curr_g that the robot performed based on the direction of rotation"""

        if init_g == curr_g:
            return 0

        init_g_360 = normalize_angle(init_g, 0)
        curr_g_360 = normalize_angle(curr_g, 0)

        first_angle = curr_g_360 - init_g_360
        second_angle = -1 * first_angle / \
            abs(first_angle) * (360 - abs(first_angle))

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

        second_angle = -1 * first_angle / \
            abs(first_angle) * (360 - abs(first_angle))
        smallest = first_angle

        if abs(first_angle) > 180:
            smallest = second_angle

        if smallest < 0:
            c = Clockwise.RIGHT
        else:
            c = Clockwise.LEFT

        return smallest, c

    def goal_reached(self) -> bool:
        global LOGSEVERITY

        self.read_sensors()

        return GATE

    def update_cfg(self):
        global OR_MAX_ATTEMPT
        global SAFE_DISTANCE
        global LOGSEVERITY

        self._speed = CFG.controller_data()["SPEED"]
        self._rot_speed = CFG.controller_data()["ROT_SPEED"]
        OR_MAX_ATTEMPT = CFG.controller_data()["MAX_ATTEMPTS"]
        SAFE_DISTANCE = CFG.controller_data()["SAFE_DIST"]
        LOGSEVERITY = CFG.logger_data()["SEVERITY"]
