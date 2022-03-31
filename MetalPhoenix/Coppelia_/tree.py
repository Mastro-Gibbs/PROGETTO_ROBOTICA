from utility import Compass
from enum import Enum


class TreeException(Exception):
    pass


class WAY(Enum):
    LEFT = 1
    MID = 2
    RIGHT = 3


class Node:
    def __init__(self, name: str, action: Compass = None):
        self.__name = name
        self.__action = action
        self.__dead_end: bool = False
        self.__final: bool = False

        self.parent = None
        self.left = None
        self.mid = None
        self.right = None

    def __repr__(self):
        return "[ Name: {0}, Parent: {1}, Left: {2}, Mid: {3}, Right: {4} ]" \
            .format(self.name, self.parent.name if self.parent is not None else None,
                    self.left.name if self.left is not None else None,
                    self.mid.name if self.mid is not None else None,
                    self.right.name if self.right is not None else None)

    def set_properties(self, _n: str = None, _c: Compass = None,
                       _d: bool = None, _f: bool = None):
        if _n is not None:
            self.__name = _n

        if _c is not None:
            self.__action = _c

        if _d is not None:
            self.__dead_end = _d

        if _f is not None:
            self.__final = _f

    def set_parent(self, _val=None):
        self.parent = _val

    def set_final(self, _val: bool = True):
        self.__final = _val

    def set_dead_end(self, _val: bool = True):
        self.__dead_end = _val

    def get_properties(self):
        return self.__name, self.__action, self.__dead_end, self.__final

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
        return True if self.has_left and self.has_mid and self.has_right else False

    @property
    def name(self) -> str:
        return self.__name

    @property
    def is_final(self) -> bool:
        return self.__final

    @property
    def is_dead_end(self) -> bool:
        return self.__dead_end


class Tree:
    def __init__(self):
        self.__root: Node = Node(name="root")
        self.__current: Node = self.__root
        self.T = {self.__root.name: {}}

    def append(self, _n: Node, _way: WAY):
        if _way == WAY.LEFT:
            if not self.__current.has_left:
                _n.parent = self.__current
                self.__current.left = _n
                self.__current = self.__current.left
            else:
                pass
                #raise TreeException("No way to append node")

        elif _way == WAY.MID:
            if not self.__current.has_mid:
                _n.parent = self.__current
                self.__current.mid = _n
                self.__current = self.__current.mid
            else:
                pass
                #raise TreeException("No way to append node")

        elif _way == WAY.RIGHT:
            if not self.__current.has_right:
                _n.parent = self.__current
                self.__current.right = _n
                self.__current = self.__current.right
            else:
                pass
                #raise TreeException("No way to append node")

    def regress(self):
        self.__current = self.__current.parent

    def DFSRec(self):
        self.__DFSRec(self.__root, 0)

    def build_tree_dict(self):
        if self.__root is None:
            return None
        self.T = {self.__root.name: {}}
        self.__DFSRec_T(self.__root)
        return self.T

    def __DFSRec(self, node, level):
        if node is None:
            return

        print("{0}, Level: {1}".format(node, level))

        self.__DFSRec(node.left, level + 1)
        self.__DFSRec(node.mid, level + 1)
        self.__DFSRec(node.right, level + 1)

    def __DFSRec_T(self, node, level=0):
        if node is None:
            return
        # print(node, level)

        if node.name not in self.T:
            self.T[node.name] = {}
        if node.left is not None:
            self.T[node.name][node.left.name] = 1
        if node.mid is not None:
            self.T[node.name][node.mid.name] = 1
        if node.right is not None:
            self.T[node.name][node.right.name] = 1

        self.__DFSRec_T(node.left, level + 1)
        self.__DFSRec_T(node.mid, level + 1)
        self.__DFSRec_T(node.right, level + 1)

    @property
    def current(self) -> Node:
        return self.__current
