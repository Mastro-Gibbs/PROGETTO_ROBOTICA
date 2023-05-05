import configparser
import statistics


class Analyzer:
    def __init__(self, path) -> None:
        ...

    @staticmethod
    def analyze(path):
        config = configparser.ConfigParser()

        config.read(path)
        __data = {sect: dict(config.items(sect)) for sect in config.sections()}

        # filtro alcuni risultati
        for key in list(__data.keys()):
            if float(__data[key]['execution_time_sec']) < 60.:
                del __data[key]

        for key in list(__data.keys()):
            if __data[key]['maze_solved'] != 'True':
                del __data[key]

        __data = {__data[sect]['execution_time_sec']: dict(__data[sect]) for sect in __data.keys()}
        for key in list(__data.keys()):
            del __data[key]['maze_solved']
            del __data[key]['execution_time_sec']
            del __data[key]['number_of_dead_end']
            del __data[key]['performed_commands']
            del __data[key]['tree_dict']
            del __data[key]['trajectory']

        __data = dict(sorted(__data.items())[0:5])
        __data = {__data[sect]['number_of_nodes']: dict(__data[sect]) for sect in __data.keys()}
        __data = dict(sorted(__data.items()))

        priorities = list(__data[key]['priority_list'] for key in list(__data.keys()))

        return statistics.mode(priorities)
