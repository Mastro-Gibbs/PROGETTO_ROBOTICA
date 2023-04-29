from lib.libctrl.tree import Tree
from lib.libctrl.utility import Compass, CFG, round_v


class Maze:
    __tree: Tree = Tree()
    __trajectory: list = list()
    __performed_coms: list = list()

    class Ananlysis:
        __MACHINE_CONF__ = CFG.robot_conf_data()

        __ID: int = 0
        __name: str = ""
        __time: float = .0
        __nodes_count: int = 1
        __dead_end_nodes: int = 0
        __status: bool = False
        __priority = __MACHINE_CONF__['PRIORITY_LIST']
        __auto_balancing = __MACHINE_CONF__["AUTO_BALANCING"]
        __intelligence = __MACHINE_CONF__["INTELLIGENCE"]

        def __init__(self, outer_instance):
            self.__outer_instance = outer_instance

        @property
        def id(self):
            return self.__ID

        @id.setter
        def id(self, value):
            self.__ID = value

        @property
        def name(self):
            return self.__name

        @name.setter
        def name(self, value):
            self.__name = value

        @property
        def time(self):
            return self.__time

        @time.setter
        def time(self, value):
            self.__time = value

        @property
        def nodes_count(self):
            return self.__nodes_count

        @nodes_count.setter
        def nodes_count(self, value):
            self.__nodes_count = value

        @property
        def dead_end_nodes(self):
            return self.__dead_end_nodes

        @dead_end_nodes.setter
        def dead_end_nodes(self, value):
            self.__dead_end_nodes = value

        @property
        def status(self):
            return self.__status

        @status.setter
        def status(self, value):
            self.__status = value

        @property
        def priority(self):
            return self.__priority

        @priority.setter
        def priority(self, value):
            self.__priority = value

        @property
        def autobalancing(self):
            return self.__auto_balancing

        @autobalancing.setter
        def autobalancing(self, value):
            self.__auto_balancing = value

        @property
        def intelligence(self):
            return self.__intelligence

        @intelligence.setter
        def intelligence(self, value):
            self.__intelligence = value

        def write(self) -> None:
            priority_list = Compass.compass_list_to_string_comma_sep(self.__priority)
            print(self.__name,
                  self.__status,
                  round_v(self.__time),
                  self.__outer_instance.tree.build_tree_dict(),
                  self.__nodes_count,
                  self.__dead_end_nodes,
                  self.__outer_instance.performed_commands,
                  self.__outer_instance.trajectory,
                  self.__intelligence,
                  "on" if self.__auto_balancing == "on" else "off",
                  priority_list if self.__intelligence == "low" else "random"
                  )

    def __init__(self):
        self.__inner = Maze.Ananlysis(self)

    @property
    def tree(self) -> Tree:
        return self.__tree

    @property
    def trajectory(self) -> list:
        return self.__trajectory

    @property
    def performed_commands(self) -> list:
        return self.__performed_coms

    def analisys(self) -> None:
        self.__inner.write()

    def incr_node_count(self):
        self.__inner.nodes_count += 1

    def incr_dead_end(self):
        self.__inner.dead_end_nodes += 1

