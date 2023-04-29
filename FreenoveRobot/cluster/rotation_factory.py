from lib.libctrl.utility import Clockwise


class RotationFactory:
    __callback = None
    __ori_callback = None

    __table: dict = {
        0: {
            Clockwise.RIGHT: 90,
            Clockwise.LEFT: -90,
        },
        90: {
            Clockwise.RIGHT: 180,
            Clockwise.LEFT: 0,
        },
        -90: {
            Clockwise.RIGHT: 0,
            Clockwise.LEFT: 180,
        },
        180: {
            Clockwise.RIGHT: -90,
            Clockwise.LEFT: 90,
        }
    }

    __val: int = 0

    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(RotationFactory, cls).__new__(cls)
        return cls.instance

    def attach_rotation_callback(self, callback) -> None:
        self.__callback = callback

    def attach_orientation_callback(self, callback) -> None:
        self.__ori_callback = callback

    def rotate(self, args: tuple) -> None:
        self.__callback(args)

    def compute(self, cw: Clockwise) -> int:
        turn: dict = self.__table[self.__val]
        self.__val: int = turn[cw]

        return self.__val

    @property
    def value(self) -> int:
        if self.__ori_callback is not None:
            return self.__ori_callback()

        return self.__val
