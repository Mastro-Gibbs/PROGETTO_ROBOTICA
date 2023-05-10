from hashlib import md5
from builtins import open as file_observer

from controller import Controller, ControllerException
from lib.libctrl.utility import Logger, CFG
from lib.libctrl.enums import Color


class ConfigFileObserver:
    __CONFIG_FILE_HASHCODE: str
    __CONFIG_FILE_HASHCODE_OLD: str

    __FILE_PATH: str

    def __init__(self, path):
        self.__FILE_PATH = path
        self.__CONFIG_FILE_HASHCODE = ''
        self.__CONFIG_FILE_HASHCODE_OLD = self.__CONFIG_FILE_HASHCODE

    def observe(self):
        self.__CONFIG_FILE_HASHCODE_OLD = self.__CONFIG_FILE_HASHCODE

        with file_observer(self.__FILE_PATH, 'rb') as configfile:
            self.__CONFIG_FILE_HASHCODE = md5(configfile.read()).hexdigest()

        return self

    @property
    def changed(self):
        return self.__CONFIG_FILE_HASHCODE_OLD != self.__CONFIG_FILE_HASHCODE


class Agent:
    __cf_observer: ConfigFileObserver
    __LOGGER_DATA = CFG.logger_data()

    def __init__(self) -> None:
        self.__cf_observer: ConfigFileObserver = ConfigFileObserver('data/config.conf')
        self.__cf_observer.observe()

        self.__logger = Logger('Agent', self.__LOGGER_DATA["SEVERITY"], Color.GREEN)
        self.__logger.set_logfile(self.__LOGGER_DATA["ALOGFILE"])

        self.__controller = Controller()

    def begin(self) -> None:
        self.__logger.log('Initializing agent', Color.YELLOW)
        self.__controller.begin()
        self.__logger.log('Agent initialized', Color.YELLOW)

    def loop(self, alt=None) -> None:
        while not self.__controller.algorithm():

            if self.__cf_observer.observe().changed:
                self.__logger.log('Config file changed', Color.GRAY)
                self.__controller.update_config()

            if alt is not None:
                alt()

        self.stop()

    def stop(self) -> None:
        self.__logger.log('Arresting agent', Color.YELLOW, newline=True)

        if self.__controller.goal_reached():
            self.__controller.ending_animation()

        self.__controller.stop()

        self.__logger.log('Agent arrested', Color.YELLOW, newline=True)


if __name__ == '__main__':
    agent = None

    try:
        agent = Agent()

        agent.begin()
        agent.loop(alt=input)

    except KeyboardInterrupt:
        pass

    except ControllerException as ce:
        print(ce.args[0])

    finally:
        if agent:
            agent.stop()
