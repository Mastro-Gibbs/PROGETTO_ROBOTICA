import builtins
from hashlib import md5

from controller import Controller
from lib.ctrllib.utility import Logger, CFG


CONFIG_FILE_HASHCODE: str = None
CONFIG_FILE_OLDHASHCODE: str = CONFIG_FILE_HASHCODE

EXIT: bool = False

class Agent:
    def __init__(self) -> None:
        global CONFIG_FILE_HASHCODE
        global CONFIG_FILE_OLDHASHCODE

        with builtins.open('data/config.conf', 'rb') as configfile:
            CONFIG_FILE_HASHCODE = md5(configfile.read()).hexdigest()
            CONFIG_FILE_OLDHASHCODE = CONFIG_FILE_HASHCODE

        self.__controller = Controller()

        self.__logger = Logger('Agent', 'gray')
        self.__logger.set_logfile(CFG.logger_data()["LOGPATH"])


    def begin(self) -> None:
        self.__logger.log('Agent fully initialized', 'green')
        self.__controller.begin()

    def stop(self) -> None:
        self.__logger.log('Agent stopped', 'green')

        if self.__controller.goal_reached():
            self.__controller.ending_animation()
        
        self.__controller.virtual_destructor()

    def loop(self) -> None:
        global CONFIG_FILE_HASHCODE
        global CONFIG_FILE_OLDHASHCODE
        global EXIT
        
        while not self.__controller.goal_reached():
            self.__controller.algorithm()

            with builtins.open('lib/data/config.conf', 'rb') as configfile:
                CONFIG_FILE_HASHCODE = md5(configfile.read()).hexdigest()

            if CONFIG_FILE_OLDHASHCODE != CONFIG_FILE_HASHCODE:
                self.__logger.log('Config file changed', 'green', italic=True)
                CONFIG_FILE_OLDHASHCODE = CONFIG_FILE_HASHCODE
                self.__controller.update_config()
        self.stop()


if __name__ == '__main__':
    agent = Agent()
    try:
        agent.begin()
        agent.loop()
    except KeyboardInterrupt:
        agent.stop()
