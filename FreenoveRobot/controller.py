import time
from math import pi
from sys import stdout

from lib.ctrllib.utility import generate_node_id, f_r_l_b_to_compass
from lib.ctrllib.utility import negate_compass, detect_target, decision_making_policy
from lib.ctrllib.utility import best_angle_and_rotation_way
from lib.ctrllib.utility import Logger, CFG

from lib.ctrllib.tree import Tree, Node

from lib.ctrllib.enums import Compass, Clockwise, WAY, Type, Command, Position, Mode, State
from lib.ctrllib.enums import RedisKEYS as RK
from lib.ctrllib.enums import RedisTOPICS as RT
from lib.ctrllib.enums import RedisCONNECTION as RC
from lib.ctrllib.enums import RedisCOMMAND as RCMD
from lib.ctrllib.enums import Color, STDOUTDecor

from lib.exit_codes import NOACTIONS, TREEUPDATEERROR

from redis import Redis
from redis.client import PubSubWorkerThread
from redis.exceptions import ConnectionError as RedisConnError


class ControllerException(Exception):
    pass


class Controller:
    _OLD_CMD = None
    _OLD_VAL = None
    _PREV_ACTION = None
    _ITERATION: int = 0
    _INIT_TIME: float = time.time()

    _CONTROLLER_DATA = CFG.controller_data()
    _ROTATION_MAX_ATTEMPTS = _CONTROLLER_DATA['MAX_ATTEMPTS']
    _SAFE_DISTANCE = _CONTROLLER_DATA['SAFE_DIST']

    def __init__(self):
        self.__logger = Logger('Controller', Color.CYAN)
        self.__logger.set_logfile(CFG.logger_data()["LOGPATH"])

        try:
            self.__redis = Redis(host=RC.HOST.value, port=int(RC.PORT.value), decode_responses=True)
            self.__redis.flushall()
            self.__pubsub = self.__redis.pubsub()
            self.__pubsub.psubscribe(**{RT.BODY_TOPIC.value: self.__on_message})
        except RedisConnError or OSError or ConnectionRefusedError:
            raise ControllerException(f'Unable to connect to redis server at: {RC.HOST.value}:{RC.PORT.value}')

        self.__robot_mode: Mode = Mode.EXPLORING
        self.__robot_state: State = State.STARTING
        self.__robot_position: Position = Position.INITIAL
        self.__robot_preference_choice = self._CONTROLLER_DATA['PREFERENCE']

        self.__robot_speed = self._CONTROLLER_DATA["SPEED"]
        self.__robot_rotation_speed = self._CONTROLLER_DATA["ROT_SPEED"]

        self.__speed_msec = self.__robot_speed * 0.25 / (self.__robot_speed // 5)
        self.__junction_sim_time = 0.25 / self.__speed_msec

        self.__orientation_sensor: int = None
        self.__left_ultrasonic_sensor: int = None
        self.__right_ultrasonic_sensor: int = None
        self.__front_ultrasonic_sensor: int = None

        self.__front_ultrasonic_stored_values: list = list()
        self.__left_ultrasonic_stored_values: list = list()
        self.__right_ultrasonic_stored_values: list = list()

        self.__left_infrared_sensor: int = None
        self.__mid_infrared_sensor: int = None
        self.__right_infrared_sensor: int = None
        self.__goal_reached: bool = False

        self.__rotation_status: bool = False
        self.__rotation_ack: int = 0

        self.__maze_tree = Tree()
        self.__maze_trajectory = list()
        self.__maze_performed_commands = list()


    def virtual_destructor(self) -> None:
        self.__logger.log('Controller Stopped!', Color.GREEN, newline=True, italic=True, underline=True)
        self.__runner.stop()
        self.__redis.close()


    def begin(self) -> None:
        self.__logger.log('Controller fully initialized', Color.GREEN, newline=True, italic=True)
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
                time.sleep(0.25)
            
            self.__send_command(RCMD.LEDINTERRUPT)

    # Updater controller configuration
    def update_config(self) -> None:
        self._CONTROLLER_DATA = CFG.controller_data()

        self.__robot_speed = self._CONTROLLER_DATA["SPEED"]
        self.__robot_rotation_speed = self._CONTROLLER_DATA["ROT_SPEED"]
        self.__robot_preference_choice = self._CONTROLLER_DATA['PREFERENCE']
        self._ROTATION_MAX_ATTEMPTS = self._CONTROLLER_DATA["MAX_ATTEMPTS"]
        self._SAFE_DISTANCE = self._CONTROLLER_DATA["SAFE_DIST"]




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

        elif _key == RK.ROTATION.value:
            _values = _value.split(';')
            self.__rotation_ack = int(_values[0])
            self.__rotation_status = bool(int(_values[1]))
            

    # sender method
    def __send_command(self, _cmd: RCMD, _val = None) -> None: # RESET  
        _msg : str = str() 

        if _cmd == RCMD.RUN: 
            _msg = ';'.join([_cmd.value, str(_val)])
            if self._OLD_VAL != _val:
                self.__redis.set(RK.MOTORS.value, _msg)
                self.__redis.publish(RT.CTRL_TOPIC.value, RK.MOTORS.value)

        elif _cmd == RCMD.STOP:
            _msg = ';'.join([_cmd.value, '0'])
            if self._OLD_CMD != _cmd:
                self.__redis.set(RK.MOTORS.value, _msg)
                self.__redis.publish(RT.CTRL_TOPIC.value, RK.MOTORS.value)

        elif _cmd == RCMD.ROTATEL or _cmd == RCMD.ROTATER:
            _msg = ';'.join([_cmd.value, str(_val[0]), str(_val[1])])
    
            self.__redis.set(RK.MOTORS.value, _msg)
            self.__redis.publish(RT.CTRL_TOPIC.value, RK.MOTORS.value)

        elif _cmd == RCMD.LEDEMIT or _cmd == RCMD.LEDINTERRUPT:
            if self._OLD_CMD != _cmd:
                if _cmd == RCMD.LEDEMIT:
                    self.__redis.set(RK.LED.value, RCMD.LEDEMIT.value)
                elif _cmd == RCMD.LEDINTERRUPT:
                    self.__redis.set(RK.LED.value, RCMD.LEDINTERRUPT.value)
                self.__redis.publish(RT.CTRL_TOPIC.value, RK.LED.value)

        elif _cmd == RCMD.BZZEMIT or _cmd == RCMD.BZZINTERRUPT:
            if self._OLD_CMD != _cmd:
                if _cmd == RCMD.BZZEMIT:
                    self.__redis.set(RK.BUZZER.value, RCMD.BZZEMIT.value)
                elif _cmd == RCMD.BZZINTERRUPT:
                    self.__redis.set(RK.BUZZER.value, RCMD.BZZINTERRUPT.value)
                self.__redis.publish(RT.CTRL_TOPIC.value, RK.BUZZER.value)
        
        self._OLD_CMD = _cmd
        self._OLD_VAL = _val

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
        if self.__is_algorithm_unlocked():

            if self._ITERATION == 0:
                print()

            self.__logger.log(f'New algorithm iteration -> #{self._ITERATION}', Color.YELLOW, newline=True, underline=True)
            self._ITERATION += 1

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
                exit(NOACTIONS)

            # ACT
            performed = self.__do_action(action)

            if performed and self._PREV_ACTION != action:
                self.__maze_performed_commands.append(action)
                if action in self.__robot_preference_choice:
                    self.__maze_trajectory.append(action)
                self._PREV_ACTION = action

        else:
            _curr_time = time.time()
            self.__logger.log('Algorithm locked, waiting for VirtualBody..' + Color.WHITE.value + \
                             f' (time spent: {round(float(_curr_time-self._INIT_TIME), 1)}s)' + STDOUTDecor.DEFAULT.value,
                            Color.RED, italic=True, _stdout=False)

            stdout.write('\r' + Color.RED.value + 'Algorithm locked, waiting for VirtualBody..' + Color.WHITE.value + \
                        f' (time spent: {round(float(_curr_time-self._INIT_TIME), 1)}s)' + STDOUTDecor.DEFAULT.value + '        ')
            stdout.flush()
            time.sleep(0.1)

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
                    exit(TREEUPDATEERROR)

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
                    exit(NOACTIONS)
                else:
                    pass

        elif self.__robot_state == State.RUNNING:
            if self.__robot_position == Position.CORRIDOR:
                if left is None or right is None:
                    actions.insert(0, Command.GO_TO_JUNCTION)
                elif front is None or front > self._SAFE_DISTANCE:
                    actions.insert(0, Command.RUN)
                elif front <= self._SAFE_DISTANCE:
                    actions.insert(0, Command.STOP)

            elif self.__robot_position == Position.JUNCTION:
                if left is not None and right is not None:
                    self.__robot_position = Position.CORRIDOR
                if front is None or front > self._SAFE_DISTANCE:
                    actions.insert(0, Command.RUN)
                elif front <= self._SAFE_DISTANCE:
                    actions.insert(0, Command.STOP)

        return actions


    def __is_algorithm_unlocked(self) -> bool:
        if self.__orientation_sensor:

            return True

        return False

    #                                                                                                    #
    #                                                                                                    #
    # ************************************ END PURE ALGORITHM SECTION ********************************** #

    

    # ************************************* ROTATION SECTION ************************************* #
    #                                                                                              #
    # Main method: '__rotate_to_final_g'                                                           #
    # It will invoke following methods like:                                                       #
    # 1) '__do_rotation'                                                                           #
    #                                                                                              #
    # ******************************************************************************************** #


    def rotate(self, target: int) -> None:
        while not self.__is_algorithm_unlocked():
            time.sleep(0.25)
            stdout.write('\rWaiting for VirtualBody...')
            stdout.flush()
            time.sleep(0.25)
            stdout.write('\r                          ')
            stdout.flush()

        attempts = self._ROTATION_MAX_ATTEMPTS

        while attempts:
            if not self.__rotation_status:
                self.__rotate_to_final_g(target, attempts) 
                attempts -= 1
            else:
                attempts = 0


    def __rotate_to_final_g(self, final_g, attempts) -> None:
        """Rotate function that rotates the robot until it reaches final_g"""
        self.__logger.log(f'Rotating to {final_g}, attempt: {self._ROTATION_MAX_ATTEMPTS - attempts + 1}°', Color.GREEN)

        _init_g = self.__orientation_sensor
        _, _cloclwise = best_angle_and_rotation_way(_init_g, final_g)

        self.__do_rotation(_cloclwise, final_g)

        if self.__rotation_status:
           self.__logger.log('Rotation complete', Color.GREEN)
        else:
            self.__logger.log('Rotation interrupted, something went wrong, retrying..', Color.RED)



    def __do_rotation(self, clk: Clockwise, target: int, opt_vel: int = None) -> None:
        _LOCAL_ACK = self.__rotation_ack
        _response: int = _LOCAL_ACK
 
        self.__rotation_status = False
        
        if opt_vel != None:
            if clk == Clockwise.RIGHT:
                self.__send_command(RCMD.ROTATER, [opt_vel, abs(target)]) 
            elif clk == Clockwise.LEFT:
                self.__send_command(RCMD.ROTATEL, [opt_vel, abs(target)])
        else:
            if clk == Clockwise.RIGHT:
                self.__send_command(RCMD.ROTATER, [self.__robot_rotation_speed, abs(target)])
            elif clk == Clockwise.LEFT:
                self.__send_command(RCMD.ROTATEL, [self.__robot_rotation_speed, abs(target)])

        while _response == _LOCAL_ACK:
            time.sleep(0.1)
            _response = self.__rotation_ack

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
                                        or self.__front_ultrasonic_sensor > self._SAFE_DISTANCE):
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
            self.rotate(action.value)
            self.__send_command(RCMD.STOP)
            self.__robot_state = State.ROTATING
