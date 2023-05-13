from enum import Enum
from hashlib import md5
from builtins import open as file_observer

from controller import Controller, ControllerException
from lib.libctrl.utility import Logger, CFG
from lib.libctrl.enums import Color

from os import stat


class CFObserverMode(Enum):
    MD5 = 1,
    STAMP = 2


class ConfigFileObserver:
    __CONFIG_FILE_HASHCODE: str
    __CONFIG_FILE_HASHCODE_OLD: str

    __FILE_PATH: str

    __STAMP_CHACHE_OLD: float
    __STAMP_CHACHE: float
    __MODE: CFObserverMode

    def __init__(self, path, mode: CFObserverMode):
        self.__FILE_PATH = path
        self.__MODE      = mode

        if self.__MODE == CFObserverMode.MD5:
            with file_observer(self.__FILE_PATH, 'rb') as configfile:
                self.__CONFIG_FILE_HASHCODE     = md5(configfile.read()).hexdigest()
                self.__CONFIG_FILE_HASHCODE_OLD = self.__CONFIG_FILE_HASHCODE
        else:
            self.__STAMP_CHACHE     = stat(self.__FILE_PATH).st_mtime
            self.__STAMP_CHACHE_OLD = self.__STAMP_CHACHE

    def observe(self):
        if self.__MODE == CFObserverMode.MD5:
            self.__CONFIG_FILE_HASHCODE_OLD = self.__CONFIG_FILE_HASHCODE

            with file_observer(self.__FILE_PATH, 'rb') as configfile:
                self.__CONFIG_FILE_HASHCODE = md5(configfile.read()).hexdigest()
        else:
            self.__STAMP_CHACHE_OLD = self.__STAMP_CHACHE
            self.__STAMP_CHACHE = stat(self.__FILE_PATH).st_mtime

        return self

    @property
    def changed(self) -> bool:
        return (self.__CONFIG_FILE_HASHCODE_OLD != self.__CONFIG_FILE_HASHCODE) \
            if self.__MODE == CFObserverMode.MD5 \
            else \
            (self.__STAMP_CHACHE != self.__STAMP_CHACHE_OLD)


class Agent:
    __config_file_observer: ConfigFileObserver
    __LOGGER_DATA = CFG.logger_data()

    def __init__(self) -> None:
        self.__config_file_observer: ConfigFileObserver = ConfigFileObserver('data/config.conf', CFObserverMode.STAMP)

        self.__logger = Logger('Agent     ', self.__LOGGER_DATA["SEVERITY"], Color.GREEN)
        self.__logger.set_logfile(self.__LOGGER_DATA["ALOGFILE"])

        self.__controller = Controller()

    def begin(self) -> bool:
        self.__logger.log('Initializing', Color.YELLOW)

        try:
            self.__controller.begin()
        except ControllerException as cexc:
            self.__logger.log(f'Controller raised an exception: {cexc.args[0]}\n', Color.RED, newline=True)
            return False

        self.__logger.log('Initialized', Color.YELLOW)

        return True

    def loop(self, alt=None) -> None:
        while not self.__controller.algorithm():

            if self.__config_file_observer.observe().changed:
                self.__logger.log('Config file changed', Color.GRAY)
                self.__controller.update_config()

            if alt is not None:
                alt()

    def stop(self) -> None:
        self.__logger.log('Arresting', Color.YELLOW)

        self.__controller.stop()

        self.__logger.log('Arrested', Color.YELLOW)


if __name__ == '__main__':
    agent = None

    try:
        agent = Agent()

        if agent.begin():
            agent.loop()

    except KeyboardInterrupt:
        pass

    except ControllerException as ce:
        print(ce.args[0])

    finally:
        if agent:
            agent.stop()
