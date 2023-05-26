from tools.utility import Compass
from enum import Enum

"""
This python file contains two class:
    1) Node: it models the tree nodes
    2) Tree: the tree of the maze
To better understand how a node is generated it is advisable to read the documentation and how the algorithm works
"""


class TreeException(Exception):
    pass


class DIRECTION(Enum):
    """ Direction of the node according to the robot orientation """
    LEFT = 1
    MID = 2
    RIGHT = 3


"""
The class Type is used to specify a property of a Node
OBSERVED:
1) A node is OBSERVED when it is first added and not explored yet.
EXPLORED:
1) A node is EXPLORED when it is first visited.
2) It can have DEAD END children but at least one must be EXPLORED namely visited.
DEAD END:
1) A node is dead end if it has not children => it is a leaf.
2) If all his children are dead end.
FINAL:
1) Last node at the end of the maze.

"""


class Type(Enum):
    OBSERVED = "OBSERVED"
    EXPLORED = "EXPLORED"
    DEAD_END = "DEAD_END"
    FINAL = "FINAL"


class Node:
    def __init__(self, name: str, action: Compass = None):
        # self.__id = id
        self.__name = name
        self.__action = action
        self.__type = Type.OBSERVED

        self.parent = None
        self.left = None
        self.mid = None
        self.right = None

    def __repr__(self):
        return "[ Name: {0}, Type: {1}, Action: {2}, Parent: {3}, Left: {4}, Mid: {5}, Right: {6} ]" \
            .format(self.__name, self.__type, self.__action,
                    self.parent.name if self.parent is not None else None,
                    self.left.name if self.left is not None else None,
                    self.mid.name if self.mid is not None else None,
                    self.right.name if self.right is not None else None)

    def set_name(self, _val):
        self.__name = _val

    def set_action(self, _val):
        self.__action = _val

    def set_type(self, _val: Type = None):
        self.__type = _val

    def set_parent(self, _val=None):
        self.parent = _val

    def get_properties(self):
        return self.__name, self.__action, self.__type

    @property
    def has_left(self) -> bool:
        return True if self.left is not None else False

    @property
    def has_mid(self) -> bool:
        return True if self.mid is not None else False

    @property
    def has_right(self) -> bool:
        return True if self.right is not None else False

    @property
    def is_leaf(self) -> bool:
        return False if self.has_left or self.has_mid or self.has_right else True

    @property
    def is_root(self) -> bool:
        return True if self.name == "root" else False

    @property
    def name(self) -> str:
        return self.__name

    @property
    def action(self) -> Compass:
        return self.__action

    @property
    def type(self) -> Type:
        return self.__type

    @property
    def action_str(self):
        if self.__action == Compass.NORTH:
            return "N"
        if self.__action == Compass.WEST:
            return "O"
        if self.__action == Compass.EAST:
            return "E"
        if self.__action == Compass.SOUTH:
            return "S"

    @property
    def type_str(self):
        if self.__type == Type.OBSERVED:
            return "O"
        if self.__type == Type.EXPLORED:
            return "E"
        if self.__type == Type.DEAD_END:
            return "D"
        if self.__type == Type.FINAL:
            return "F"


class Tree:
    def __init__(self):
        self.__root: Node = Node(name="root")
        self.number_of_nodes = 1
        # self.__root.set_type(Type.EXPLORED)
        self.__current: Node = self.__root
        self.__T = {self.__root.name: {}}

    def append(self, _n: Node, _dir: DIRECTION):
        if _dir == DIRECTION.LEFT:
            # print("current", self.__current)
            if not self.__current.has_left:
                _n.parent = self.__current
                self.__current.left = _n
                self.__current = self.__current.left
            else:
                pass
                # raise TreeException("No way to append node")

        elif _dir == DIRECTION.MID:
            if not self.__current.has_mid:
                _n.parent = self.__current
                self.__current.mid = _n
                self.__current = self.__current.mid
            else:
                pass
                # raise TreeException("No way to append node")

        elif _dir == DIRECTION.RIGHT:
            if not self.__current.has_right:
                _n.parent = self.__current
                self.__current.right = _n
                self.__current = self.__current.right
            else:
                pass
                # raise TreeException("No way to append node")

    def regress(self):
        self.__current = self.__current.parent

    def set_current(self, node: Node):
        self.__current = node

    def generate_node_id(self) -> str:
        self.number_of_nodes += 1
        return "n" + str(self.number_of_nodes)

    @property
    def current(self) -> Node:
        return self.__current

    def DFSRec(self):
        self.__DFSRec(self.__root, 0)

    def __DFSRec(self, node, level):
        if node is None:
            return

        print("{0}, Level: {1}".format(node, level))

        self.__DFSRec(node.left, level + 1)
        self.__DFSRec(node.mid, level + 1)
        self.__DFSRec(node.right, level + 1)

    def build_tree_dict(self):
        """
        According to the root node of the tree object this method returns a dictionary of the tree where:
        key: is a node
        value: a dict of his children where each child has, as value, a string of three values:
            1) Left/Mid/Right
            2) NORD/EST/OVEST/SUD
            3) EXPLORED/OBSERVED/DEAD_END/FINAL
        """
        if self.__root is None:
            return None
        self.__T = {self.__root.name: {}}
        self.__DFSRec_T(self.__root)
        return self.__T

    def __DFSRec_T(self, node, level=0):
        if node is None:
            return
        # print(node, level)

        if node.name not in self.__T:
            self.__T[node.name] = {}
        if node.left is not None:
            self.__T[node.name][node.left.name] = 1
            self.__T[node.name][node.left.name] = f"(L , {node.left.action_str} , {node.left.type_str})"
        if node.mid is not None:
            self.__T[node.name][node.mid.name] = 1
            self.__T[node.name][node.mid.name] = f"(M , {node.mid.action_str} , {node.mid.type_str})"
        if node.right is not None:
            self.__T[node.name][node.right.name] = 1
            self.__T[node.name][node.right.name] = f"(R , {node.right.action_str} , {node.right.type_str})"

        self.__DFSRec_T(node.left, level + 1)
        self.__DFSRec_T(node.mid, level + 1)
        self.__DFSRec_T(node.right, level + 1)
