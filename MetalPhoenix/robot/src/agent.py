from controller import Controller
import time
import hashlib
import builtins
from tools.utility import Logger, CFG


class Agent:
    controller: Controller = Controller()

    def __init__(self) -> None:
        self.__logger: Logger = Logger(class_name="Agent", color="yellow")
        self.__logger.set_logfile(CFG.logger_data()["ALOGFILE"])

        self.__goal_reached: bool = False

        self.__hash = None
        self.__hash_old = None

    def run(self):
        self.__logger.log("AGENT LAUNCHED", "green", italic=True)

        self.read_conf_file()

        try:
            while not self.__goal_reached:
                self.__goal_reached = self.controller.algorithm()
                self.check_and_update_from_config_file()

        except KeyboardInterrupt:
            self.controller.execution_time = time.time() - self.controller.start_time
            self.controller.print_data(self.controller.maze_solved)
            self.controller.write_data_analysis()
            self.stop()

    def stop(self):
        self.__goal_reached = True
        self.controller.virtual_destructor()
        self.__logger.log("AGENT STOPPED", "red", italic=True)

    def check_and_update_from_config_file(self):
        self.read_conf_file()
        if self.__hash_old != self.__hash:
            self.__logger.log("Config file changed", "yellow", italic=True)
            self.__hash_old = self.__hash
            self.controller.load_cfg_values()

    def read_conf_file(self):
        with builtins.open("../resources/data/config.conf", "rb") as f:
            self.__hash = hashlib.md5(f.read()).hexdigest()
