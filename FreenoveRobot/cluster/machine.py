from lib.libctrl.enums import Position, Mode, State
from lib.libctrl.utility import CFG


class Machine:
    __MACHINE_CONF__ = CFG.robot_conf_data()

    __mode = Mode.EXPLORING
    __state = State.STARTING
    __position = Position.UNKNOWN

    __speed = __MACHINE_CONF__["SPEED"]
    __rot_speed = __MACHINE_CONF__["ROT_SPEED"]

    __auto_balancing = __MACHINE_CONF__["AUTO_BALANCING"]

    __priority_list = __MACHINE_CONF__['PRIORITY_LIST']

    @property
    def mode(self):
        return self.__mode

    @property
    def state(self):
        return self.__state

    @property
    def position(self):
        return self.__position

    @property
    def speed(self):
        return self.__speed

    @property
    def rot_speed(self):
        return self.__rot_speed

    @property
    def autobalance(self):
        return self.__auto_balancing

    @property
    def priority(self):
        return self.__priority_list

    @mode.setter
    def mode(self, value):
        self.__mode = value

    @state.setter
    def state(self, value):
        self.__state = value

    @position.setter
    def position(self, value):
        self.__position = value
        
    @speed.setter
    def speed(self, value):
        self.__speed = value

    @rot_speed.setter
    def rot_speed(self, value):
        self.__rot_speed = value

    @autobalance.setter
    def autobalance(self, value):
        self.__auto_balancing = value

    @priority.setter
    def priority(self, value):
        self.__priority_list = value
        
    def update(self) -> None:
        self.__MACHINE_CONF__ = CFG.robot_conf_data()
        
        self.speed = self.__MACHINE_CONF__["SPEED"]
        self.rot_speed = self.__MACHINE_CONF__["ROT_SPEED"]
        self.priority = self.__MACHINE_CONF__['PRIORITY_LIST']
        self.autobalance = self.__MACHINE_CONF__["AUTO_BALANCING"]
