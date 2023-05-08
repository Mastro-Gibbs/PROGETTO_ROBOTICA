from lib.libctrl.utility import FRLB


class RotationFactory:
    __callback = None
    __ori_callback = None

    __val: float = 0.0
    
    __table: dict = {
        0.0: {
            90.0: FRLB.RIGHT,
            -90.0: FRLB.LEFT,
            180.0: FRLB.BACK,
        },
        90.0: {
            180.0: FRLB.RIGHT,
            0.0: FRLB.LEFT,
            -90.0: FRLB.BACK,
        },
        -90.0: {
            0.0: FRLB.RIGHT,
            180.0: FRLB.LEFT,
            90.0: FRLB.BACK,
        },
        180.0: {
            -90.0: FRLB.RIGHT,
            90.0: FRLB.LEFT,
            0.0: FRLB.BACK
        }
    }

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

    def compute(self, ori: float) -> FRLB:
        entry = self.__table[self.__val]
        self.__val: float = ori

        return entry[self.__val]

    @property
    def value(self) -> float:
        if self.__ori_callback is not None:
            return self.__ori_callback()

        return self.__val
