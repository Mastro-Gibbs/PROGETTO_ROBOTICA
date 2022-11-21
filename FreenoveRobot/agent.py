import builtins
from hashlib import md5

from controller import Controller, ControllerException
from lib.libctrl.utility import Logger, CFG
from lib.libctrl.enums import Color


CONFIG_FILE_HASHCODE: str = None
CONFIG_FILE_OLDHASHCODE: str = CONFIG_FILE_HASHCODE

class Agent:
    def __init__(self) -> None:
        global CONFIG_FILE_HASHCODE
        global CONFIG_FILE_OLDHASHCODE

        with builtins.open('data/config.conf', 'rb') as configfile:
            CONFIG_FILE_HASHCODE = md5(configfile.read()).hexdigest()
            CONFIG_FILE_OLDHASHCODE = CONFIG_FILE_HASHCODE

        self.__controller = Controller()

        self.__logger = Logger('Agent', Color.YELLOW)
        self.__logger.set_logfile(CFG.logger_data()["LOGPATH"])


    def begin(self) -> None:
        self.__logger.log('Agent fully initialized', Color.GREEN, newline=True, italic=True)
        self.__controller.begin()

    def loop(self) -> None:
        global CONFIG_FILE_HASHCODE
        global CONFIG_FILE_OLDHASHCODE
        
        while not self.__controller.goal_reached():
            self.__controller.algorithm()

            with builtins.open('data/config.conf', 'rb') as configfile:
                CONFIG_FILE_HASHCODE = md5(configfile.read()).hexdigest()

            if CONFIG_FILE_OLDHASHCODE != CONFIG_FILE_HASHCODE:
                self.__logger.log('Config file changed', Color.YELLOW, italic=True, blink=True)
                CONFIG_FILE_OLDHASHCODE = CONFIG_FILE_HASHCODE
                self.__controller.update_config()
        self.stop()


    def stop(self) -> None:
        self.__logger.log('Agent stopped', Color.GREEN, newline=True, italic=True, underline=True)

        if self.__controller.goal_reached():
            self.__controller.ending_animation()
        
        self.__controller.virtual_destructor()


if __name__ == '__main__':
    agent = None
    try:
        agent = Agent()
        agent.begin()
        agent.loop()
    except KeyboardInterrupt:
        pass
    except ControllerException as ce:
        print(ce.args[0])
    finally:
        if agent:
            agent.stop()
