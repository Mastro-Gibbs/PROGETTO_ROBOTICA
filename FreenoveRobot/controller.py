import time

from math import pi

from lib.ctrllib.utility import generate_node_id, f_r_l_b_to_compass
from lib.ctrllib.utility import negate_compass, detect_target, decision_making_policy
from lib.ctrllib.utility import compute_performed_degrees, best_angle_and_rotation_way
from lib.ctrllib.utility import Logger, CFG

from lib.ctrllib.tree import Tree, Node

from lib.ctrllib.enums import Compass, Clockwise, WAY, Type, Command, Position, Mode, State
from lib.ctrllib.enums import RedisKEYS as RK
from lib.ctrllib.enums import RedisTOPICS as RT
from lib.ctrllib.enums import RedisCONNECTION as RC
from lib.ctrllib.enums import RedisCOMMAND as RCMD
from lib.ctrllib.enums import Color, STDOUTDecor

from redis import Redis
from redis.client import PubSubWorkerThread


_OLD_CMD = None
_OLD_VAL = None
_PREV_ACTION = None
_ITERATION: int = 0
_INIT_TIME: float = time.time()

class Controller:
    def __init__(self):
        self.__redis = Redis(host=RC.HOST.value, port=int(RC.PORT.value), decode_responses=True)
        self.__pubsub = self.__redis.pubsub()
        self.__pubsub.psubscribe(**{RT.BODY_TOPIC.value: self.__on_message})

        self.__logger = Logger('Controller', Color.CYAN)
        self.__logger.set_logfile(CFG.logger_data()["LOGPATH"])

        self.__robot_mode: Mode = Mode.EXPLORING
        self.__robot_state: State = State.STARTING
        self.__robot_position: Position = Position.INITIAL
        self.__robot_preference_choice = [Compass.NORD, Compass.OVEST, Compass.EST, Compass.SUD]

        self.__robot_speed = CFG.controller_data()["SPEED"]
        self.__robot_rotation_speed = CFG.controller_data()["ROT_SPEED"]

        self.__speed_msec = self.__robot_speed * 0.25 / (self.__robot_speed // 5)
        self.__junction_sim_time = 0.25 / self.__speed_msec

        self.__orientation_sensor: int = None
        self.__left_ultrasonic_sensor: int = None
        self.__right_ultrasonic_sensor: int = None
        self.__front_ultrasonic_sensor: int = None

        self.__front_ultrasonic_stored_values = list()
        self.__left_ultrasonic_stored_values = list()
        self.__right_ultrasonic_stored_values = list()

        self.__left_infrared_sensor: int = None
        self.__mid_infrared_sensor: int = None
        self.__right_infrared_sensor: int = None
        self.__goal_reached: bool = False

        self.__maze_tree = Tree()
        self.__maze_trajectory = list()
        self.__maze_performed_commands = list()


    def virtual_destructor(self) -> None:
        self.__logger.log('Controller Stopped!', Color.GREEN, newline=True, italic=True, underline=True)
        self.__runner.stop()


    def begin(self) -> None:
        self.__logger.log('Controller fully initialized', Color.GREEN, newline=True, italic=True, blink=True)
        self.__send_command(RCMD.STOP)
        self.__runner: PubSubWorkerThread = self.__pubsub.run_in_thread(sleep_time=0.01)

    # Check if maze was solved. 
    def goal_reached(self) -> bool:
        if self.__left_infrared_sensor and self.__mid_infrared_sensor and self.__right_infrared_sensor:
            self.__goal_reached = True   
        
        return self.__goal_reached

    # ending animation if maze were solved
    def ending_animation(self) -> None:
        if self.__goal_reached:
            self.__logger.log('Maze finished', Color.GREEN, newline=True, italic=True, blink=True)
            self.__send_command(RCMD.LEDEMIT)

            for i in range(0, 5, 1):
                self.__send_command(RCMD.BZZEMIT)
                time.sleep(0.25)
                self.__send_command(RCMD.BZZINTERRUPT)
            
            self.__send_command(RCMD.LEDINTERRUPT)

    # Updater controller configuration
    def update_config(self) -> None:
        global OR_MAX_ATTEMPT
        global SAFE_DISTANCE

        self.__robot_speed = CFG.redis_data()["SPEED"]
        self.__robot_rotation_speed = CFG.redis_data()["ROT_SPEED"]
        OR_MAX_ATTEMPT = CFG.redis_data()["MAX_ATTEMPTS"]
        SAFE_DISTANCE = CFG.redis_data()["SAFE_DIST"]



    # ************************************* REDIS SECTION ************************************* #
    #                                                                                           #
    # @function                                                                                 #
    # '__on_message' -> redis builtin thread body/scope                                         #
    #                 read value from redis, set up instance vars.                              #
    #                                                                                           #
    # 'send_command' -> write on redis db.                                                      #
    #                 @param _cmd instance of RCMD: depending on param type send key and publish#
    #                 @param _val -> value to send                                              #
    #                                                                                           #
    # ***************************************************************************************** #

    # callback receiver
    def __on_message(self, msg):
        _key = msg['data']
        _value = self.__redis.get(_key)

        if _key == RK.MPU.value:
            self.__orientation_sensor = int(_value)
                
        elif _key == RK.ULTRASONIC.value:
            _values = _value.split(';')
            self.__left_ultrasonic_sensor = int(_values[0])
            self.__front_ultrasonic_sensor = int(_values[1])
            self.__right_ultrasonic_sensor = int(_values[2])

        elif _key == RK.INFRARED.value:
            _values = _value.split(';')
            self.__left_infrared_sensor = int(_values[0])
            self.__mid_infrared_sensor = int(_values[1])
            self.__right_infrared_sensor = int(_values[2])

    # sender method
    def __send_command(self, _cmd: RCMD, _val = 0) -> None:
        global _OLD_CMD
        global _OLD_VAL

        if _val:
            _msg = ';'.join([_cmd.value, _val])
        else:
            _msg = _cmd.value

        if _cmd == RCMD.RUN or _cmd == RCMD.ROTATEL or _cmd == RCMD.ROTATER:
            if _OLD_VAL != _val:
                self.__redis.set(RK.MOTORS.value, _msg)
                self.__redis.publish(RT.CTRL_TOPIC.value, RK.MOTORS.value)

        elif _cmd == RCMD.STOP:
            if _OLD_CMD != _cmd:
                self.__redis.set(RK.MOTORS.value, _msg)
                self.__redis.publish(RT.CTRL_TOPIC.value, RK.MOTORS.value)

        elif _cmd == RCMD.LEDEMIT or _cmd == RCMD.LEDINTERRUPT:
            if _OLD_CMD != _cmd:
                if _cmd == RCMD.LEDEMIT:
                    self.__redis.set(RK.LED.value, RCMD.LEDEMIT.value)
                elif _cmd == RCMD.LEDINTERRUPT:
                    self.__redis.set(RK.LED.value, RCMD.LEDINTERRUPT.value)
                self.__redis.publish(RT.CTRL_TOPIC.value, RK.LED.value)

        elif _cmd == RCMD.BZZEMIT or _cmd == RCMD.BZZINTERRUPT:
            if _OLD_CMD != _cmd:
                if _cmd == RCMD.BZZEMIT:
                    self.__redis.set(RK.BUZZER.value, RCMD.BZZEMIT.value)
                elif _cmd == RCMD.BZZINTERRUPT:
                    self.__redis.set(RK.BUZZER.value, RCMD.BZZINTERRUPT.value)
                self.__redis.publish(RT.CTRL_TOPIC.value, RK.BUZZER.value)
        
        _OLD_CMD = _cmd
        _OLD_VAL = _val

    #                                                                                           #
    #                                                                                           #
    # *********************************** END REDIS SECTION *********************************** #




    # ************************************* PURE ALGORITHM SECTION ************************************* #
    #                                                                                                    #
    # Tree step.                                                                                         #
    # On each loop invoke algorithm.                                                                     #
    # Algorithm will call 'update_tree'. This method will update tree size (add nodes).                  #
    # Method 'controll_policy' will make choises.                                                        #
    # '__verify_gate' check if there is a gate.                                                          #
    #                                                                                                    #
    # ************************************************************************************************** #

    # Algorithm entry
    def algorithm(self) -> None:
        global _PREV_ACTION
        global _ITERATION

        if self.__is_algorithm_unlocked():

            self.__logger.log(f'New algorithm iteration -> #{_ITERATION}', Color.YELLOW, newline=True, underline=True)
            _ITERATION += 1

            self.__left_ultrasonic_stored_values.append(self.__left_ultrasonic_sensor)
            self.__front_ultrasonic_stored_values.append(self.__front_ultrasonic_sensor)
            self.__right_ultrasonic_stored_values.append(self.__right_ultrasonic_sensor)

            # THINK
            actions = self.__control_policy()

            # Update tree adding the children if only if the actions are correct (namely the robot is in sensing mode)
            self.__update_tree(actions)

            # THINK
            action = decision_making_policy(self.__robot_preference_choice, actions)

            # Updating tree setting the current node
            self.__update_tree(action)

            if action is None:
                self.__logger.log('No action available, force quitting..', Color.DARKRED, newline=True)
                self.virtual_destructor()
                exit(-1)

            # ACT
            performed = self.__do_action(action)

            if performed and _PREV_ACTION != action:
                self.__maze_performed_commands.append(action)
                if action in self.__robot_preference_choice:
                    self.__maze_trajectory.append(action)
                _PREV_ACTION = action

        else:
            _curr_time = time.time()
            self.__logger.log('Algorithm locked, waiting for VirtualBody..' + Color.WHITE.value + \
                             f' (time spent: {round(float(_curr_time-_INIT_TIME), 1)}s)' + STDOUTDecor.DEFAULT.value,
                            Color.RED, italic=True)
            time.sleep(2.5)

    # Tree updater
    def __update_tree(self, actions) -> None:
        self.__logger.log('Updating tree..', Color.GREEN)

        if not self.__robot_state == State.SENSING:
            return

        if isinstance(actions, Command):
            return

        if self.__robot_mode == Mode.EXPLORING:
            # Only one action, l'azione è stata decisa dalla DMP ed è di tipo Compass
            if isinstance(actions, Compass):

                action = actions
                dict_ = f_r_l_b_to_compass(self.__orientation_sensor)
                if dict_["FRONT"] == action:
                    self.__maze_tree.set_current(self.__maze_tree.current.mid)
                elif dict_["LEFT"] == action:
                    self.__maze_tree.set_current(self.__maze_tree.current.left)
                elif dict_["RIGHT"] == action:
                    self.__maze_tree.set_current(self.__maze_tree.current.right)

            else:
                for action in actions:
                    dict_ = f_r_l_b_to_compass(self.__orientation_sensor)
                    if dict_["FRONT"] == action:
                        node = Node("M_" + generate_node_id(), action)
                        self.__maze_tree.append(node, WAY.MID)
                        self.__maze_tree.regress()
                    if dict_["LEFT"] == action:
                        node = Node("L_" + generate_node_id(), action)
                        self.__maze_tree.append(node, WAY.LEFT)
                        self.__maze_tree.regress()
                    if dict_["RIGHT"] == action:
                        node = Node("R_" + generate_node_id(), action)
                        self.__maze_tree.append(node, WAY.RIGHT)
                        self.__maze_tree.regress()

                self.__maze_tree.current.set_type(Type.EXPLORED)

        elif self.__robot_mode == Mode.ESCAPING:
            if isinstance(actions, Compass):
                action = actions

                if negate_compass(self.__maze_tree.current.action) == action:
                    self.__maze_tree.regress()
                    return

                # Caso in cui scelgo un OBSERVED e lo metto come corrente
                cur = None
                if self.__maze_tree.current.has_left and self.__maze_tree.current.left.action == action:
                    cur = self.__maze_tree.current.left

                elif self.__maze_tree.current.has_mid and self.__maze_tree.current.mid.action == action:
                    cur = self.__maze_tree.current.mid

                elif self.__maze_tree.current.has_right and self.__maze_tree.current.right.action == action:
                    cur = self.__maze_tree.current.right

                else:
                    self.__logger.log('Updating tree generate error, force quitting..', Color.DARKRED, newline=True)
                    exit(-1)

                self.__maze_tree.set_current(cur)

            else:
                # The node is a leaf
                if self.__maze_tree.current.is_leaf:
                    self.__maze_tree.current.set_type(Type.DEAD_END)

                # The children are all DEAD END
                elif ((self.__maze_tree.current.has_left and self.__maze_tree.current.left.type == Type.DEAD_END) or
                      self.__maze_tree.current.left is None) and \
                        ((self.__maze_tree.current.has_right and self.__maze_tree.current.right.type == Type.DEAD_END)
                         or self.__maze_tree.current.right is None) and \
                        ((self.__maze_tree.current.has_mid and self.__maze_tree.current.mid.type == Type.DEAD_END)
                         or self.__maze_tree.current.mid is None):
                    self.__maze_tree.current.set_type(Type.DEAD_END)

                else:
                    self.__logger.log('Updating tree aborted, no dead-end child', Color.YELLOW)
                    # No DEAD END children, tree'll be updated on next loop
                    pass

    # Action maker
    def __control_policy(self) -> list:
        self.__logger.log('Control policy invoked..', Color.GREEN)
        actions = list()

        left = self.__left_ultrasonic_sensor
        front = self.__front_ultrasonic_sensor
        right = self.__right_ultrasonic_sensor
        ori = self.__orientation_sensor

        # Sto in RUNNING e l'albero non viene aggiornato
        if self.__robot_mode == Mode.ESCAPING and self.__maze_tree.current.type == Type.OBSERVED:
            self.__robot_mode = Mode.EXPLORING

        if self.__robot_state == State.STARTING and self.__robot_position == Position.INITIAL:
            if left is not None and right is not None:
                self.__robot_position = Position.CORRIDOR
            elif left is None or right is None:
                self.__robot_position = Position.JUNCTION
            actions.insert(0, Command.START)

        elif self.__robot_state == State.ROTATING:
            actions.insert(
                0, self.__maze_performed_commands[len(self.__maze_performed_commands) - 1])

        elif self.__robot_state == State.STOPPED or \
                self.__robot_state == State.SENSING or \
                (self.__robot_state == State.STARTING and not self.__robot_position == Position.INITIAL):

            if self.__robot_state == State.STARTING:
                self.__robot_state = State.SENSING

            if self.__robot_mode == Mode.EXPLORING:
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
                    action = negate_compass(self.__maze_tree.current.action)
                    actions.insert(0, action)
                    self.__robot_mode = Mode.ESCAPING
                    self.__robot_state = State.SENSING

            elif self.__robot_mode == Mode.ESCAPING:
                if self.__maze_tree.current.left is not None and self.__maze_tree.current.left.type == Type.OBSERVED:
                    action = self.__maze_tree.current.left.action
                    actions.insert(0, action)
                if self.__maze_tree.current.mid is not None and self.__maze_tree.current.mid.type == Type.OBSERVED:
                    action = self.__maze_tree.current.mid.action
                    actions.insert(0, action)
                if self.__maze_tree.current.right is not None and self.__maze_tree.current.right.type == Type.OBSERVED:
                    action = self.__maze_tree.current.right.action
                    actions.insert(0, action)

                # Se non ci sono OBSERVED scegliere EXPLORED
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

                # Se non ci sono EXPLORED tornare indietro
                if not actions:
                    action = negate_compass(self.__maze_tree.current.action)
                    actions.insert(0, action)

                if not actions:
                    self.__logger.log('No action available, force quitting..', Color.DARKRED, newline=True)
                    self.virtual_destructor()
                    exit(-1)
                else:
                    pass

        elif self.__robot_state == State.RUNNING:
            if self.__robot_position == Position.CORRIDOR:
                if left is None or right is None:
                    actions.insert(0, Command.GO_TO_JUNCTION)
                elif front is None or front > SAFE_DISTANCE:
                    actions.insert(0, Command.RUN)
                elif front <= SAFE_DISTANCE:
                    actions.insert(0, Command.STOP)

            elif self.__robot_position == Position.JUNCTION:
                if left is not None and right is not None:
                    self.__robot_position = Position.CORRIDOR
                if front is None or front > SAFE_DISTANCE:
                    actions.insert(0, Command.RUN)
                elif front <= SAFE_DISTANCE:
                    actions.insert(0, Command.STOP)

        return actions


    def __is_algorithm_unlocked(self) -> bool:
        if self.__orientation_sensor and self.__left_infrared_sensor and \
            self.__mid_infrared_sensor and self.__right_infrared_sensor:

            return True

        return False

    # Check there is a gate
    def __verify_gate(self, c: Compass) -> bool: # not used???
        global OR_MAX_ATTEMPT

        self.read_sensors()

        it = 0
        _gate: bool = False

        _sens = 0.0
        _not_none_counter = 0

        if c == Compass.OVEST:
            _sens = self.__left_ultrasonic_sensor
        elif c == Compass.EST:
            _sens = self.__right_ultrasonic_sensor

        self.read_sensors()

        while it < OR_MAX_ATTEMPT:
            it += 1
            if _sens is None:
                _gate = True
                if c == Compass.OVEST:
                    _sens = self.__left_ultrasonic_sensor
                elif c == Compass.EST:
                    _sens = self.__right_ultrasonic_sensor
            else:
                _gate = False
                _not_none_counter += 1
                if _not_none_counter == 2:
                    break

        return _gate

    #                                                                                                    #
    #                                                                                                    #
    # ************************************ END PURE ALGORITHM SECTION ********************************** #

    

    # ************************************* ROTATION SECTION ************************************* #
    #                                                                                              #
    # Main method: '__rotate_to_final_g'                                                           #
    # It will invoke following methods like:                                                       #
    # 1) '__do_rotation' -> wrapper who invoke '__rotate', '__check_orirentation'                  #
    #                       and '__adjust_orientation'.                                            #
    #                                                                                              #
    # ******************************************************************************************** #


    def __rotate_to_final_g(self, final_g) -> None:
        """Rotate function that rotates the robot until it reaches final_g"""

        self.__logger.log(f'Rotating to {final_g}', Color.GREEN)

        _init_g = self.__orientation_sensor
        _degrees, _cloclwise = best_angle_and_rotation_way(_init_g, final_g)

        _ok = self.__do_rotation(clk=_cloclwise, degrees=_degrees, final_g=final_g)

        if _ok:
           self.__logger.log('Rotation complete', Color.GREEN)


    def __do_rotation(self, clk: Clockwise, degrees, final_g) -> bool:
        _degrees = abs(degrees)
        _it = 0

        self.__rotate(clk, _degrees)

        _ok, _, _ = self.__check_orientation(final_g)

        if not _ok:
            _ok, _it = self.__adjust_orientation(final_g)

        if _it == OR_MAX_ATTEMPT:  # CRITICAL CASE
            self.__logger.log('Max adjusting attempts reached, force quitting..', Color.DARKRED, newline=True)
            self.virtual_destructor()
            exit(-1)

        return _ok


    def __rotate(self, c: Clockwise, degrees) -> tuple:
        """
        Function that given vel, Clockwise and rotation degrees computes
        the rotation of the Robot around the z axis
        """
        degrees = abs(degrees)

        init_g = self.__orientation_sensor

        stop = False
        archived = False
        performed_deg = 0.0

        while not stop:
            curr_g = self.__orientation_sensor

            if c == Clockwise.RIGHT:
                self.__send_command(RCMD.ROTATER, self.__robot_rotation_speed)
            elif c == Clockwise.LEFT:
                self.__send_command(RCMD.ROTATEL, self.__robot_rotation_speed)

            performed_deg_temp = compute_performed_degrees(c, init_g, curr_g)

            if performed_deg_temp > 300:
                continue

            performed_deg = performed_deg_temp

            if degrees - 0.8 < performed_deg < degrees + 0.8:
                self.__send_command(RCMD.STOP)
                archived = True
                stop = True

            elif performed_deg > degrees + 0.8:
                self.__send_command(RCMD.STOP)
                archived = False
                stop = True

        return archived, init_g, performed_deg, degrees


    def __check_orientation(self, final_g: int, delta: int = 2) -> tuple:
        self.__logger.log('Checking orientation..', Color.GREEN)

        _curr_g = self.__orientation_sensor
        _ok = False

        if abs(final_g) + delta > 180:
            limit_g_dx = 180 - delta
            limit_g_sx = - 180 + delta
            if _curr_g < limit_g_sx or _curr_g > limit_g_dx:
                _ok = True
            else:
                # bad ori
                pass
        else:
            limit_g_dx = final_g - delta
            limit_g_sx = final_g + delta
            if limit_g_dx <= _curr_g <= limit_g_sx:
                _ok = True
            else:
                # bad ori
                pass

        limit_range = [limit_g_sx, limit_g_dx]
        return _ok, _curr_g, limit_range


    def __adjust_orientation(self, final_g) -> tuple:
        self.__logger.log('Adjusting orientation..', Color.GREEN)

        _ok = False
        _it = 0

        while not _ok and _it < OR_MAX_ATTEMPT:
            _curr_g = self.__orientation_sensor

            degrees, c = best_angle_and_rotation_way(_curr_g, final_g)

            if abs(degrees) < 6:
                self.__rotate(0.25, c, abs(degrees))
            else:
                self.__rotate(45 * pi / 180, c, abs(degrees))

            _ok, _curr_g, _ = self.__check_orientation(final_g)
            _it += 1

        return _ok, _it

    #                                                                                              #
    #                                                                                              #
    # *********************************** END ROTATION SECTION *********************************** #

    # Send main commands
    def __do_action(self, action):
        self.__logger.log('Sending action to VirtualBody..', Color.GREEN)

        if action == Command.START:
            self.__send_command(RCMD.STOP)

            self.__send_command(RCMD.BZZEMIT)
            self.__send_command(RCMD.LEDEMIT)
            time.sleep(1)
            self.__send_command(RCMD.BZZINTERRUPT)
            self.__send_command(RCMD.LEDINTERRUPT)

        # Stop
        elif action == Command.STOP:
            self.__send_command(RCMD.STOP)

            self.__robot_state = State.STOPPED

        # Go on
        elif action == detect_target(self.__orientation_sensor) or action == Command.RUN:
            self.__send_command(RCMD.RUN, self.__robot_speed)

            self.__robot_state = State.RUNNING

        # Go to Junction
        elif action == Command.GO_TO_JUNCTION:

            start_time = time.time()
            time_expired = False

            while not time_expired and (self.__front_ultrasonic_sensor is None
                                        or self.__front_ultrasonic_sensor > SAFE_DISTANCE):
                self.__send_command(RCMD.RUN, self.__robot_speed)
                if time.time() - start_time >= self.__junction_sim_time:
                    time_expired = True

            self.__send_command(RCMD.STOP)
            self.__robot_state = State.SENSING
            self.__robot_position = Position.JUNCTION

            time.sleep(0.5)

        # Rotate (DA GESTIRE MEGLIO)
        else:
            self.__send_command(RCMD.STOP)
            self.__rotate_to_final_g(action.value)
            self.__send_command(RCMD.STOP)
            self.__robot_state = State.ROTATING