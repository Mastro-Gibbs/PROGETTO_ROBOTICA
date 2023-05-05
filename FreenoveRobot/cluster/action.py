from lib.libctrl.enums import Command
from lib.libctrl.utility import FRLB, Compass


class CommandAction:
    __command: Command
    __observed_paths: tuple[FRLB]
    __compass_action: Compass
    __selected_observed_path: FRLB

    def __init__(self):
        pass

    @property
    def command(self) -> Command:
        return self.__command

    @command.setter
    def command(self, value) -> None:
        self.__command = value

    @property
    def observed_paths(self) -> tuple[FRLB]:
        return self.__observed_paths

    @observed_paths.setter
    def observed_paths(self, value) -> None:
        self.__observed_paths = value

    @property
    def selected_observed_path(self) -> FRLB:
        return self.__selected_observed_path

    @selected_observed_path.setter
    def selected_observed_path(self, value) -> None:
        self.__selected_observed_path = value

    @property
    def compass_action(self) -> Compass:
        return self.__compass_action

    @compass_action.setter
    def compass_action(self, value) -> None:
        self.__compass_action = value
