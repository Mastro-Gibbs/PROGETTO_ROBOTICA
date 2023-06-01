from controller import Controller
import time
import hashlib
import builtins
from tools.utility import Logger, CFG


class agent:
    controller: Controller = Controller()


    def __init__(self) -> None:
        self.__logger:     Logger   = Logger(class_name="Agent", color="yellow")
        self.__logger.set_logfile(CFG.logger_data()["ALOGFILE"])

        self.__goal_reached: bool = False

        self.__hash     = None
        self.__hash_old = self.__hash


    def run(self):
        self.__logger.log("AGENT LAUNCHED", "green", italic=True)

        start_time = time.time()

        while not self.__goal_reached:
            self.__goal_reached = self.controller.algorithm()

            with builtins.open("../resources/data/config.conf", "rb") as f:
                self.__hash = hashlib.md5(f.read()).hexdigest()

            if self.__hash_old != self.__hash:
                self.__logger.log("Config file changed", "yellow", italic=True)
                self.__hash_old = self.__hash
                self.controller.load_cfg_values()

            end_time = time.time()
            self.controller.execution_time = end_time - start_time


    def stop(self):
        self.__goal_reached = True

        self.controller.virtual_destructor()
        self.__logger.log("AGENT STOPPED", "yellow", italic=True)


