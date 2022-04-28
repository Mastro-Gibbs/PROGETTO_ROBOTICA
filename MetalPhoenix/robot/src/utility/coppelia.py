import RemoteApiPython.sim as sim
from RemoteApiPython.simConst import *


class SimConnectionException(Exception):
    pass


class SimHandleException(Exception):
    pass


class SimConnection:
    def __init__(self, ip: str, port: int):
        self.__id = None
        self.__ip = ip
        self.__port = port

    def begin_connection(self):
        self.__id = sim.simxStart(
            connectionAddress=self.__ip,
            connectionPort=self.__port,
            waitUntilConnected=True,
            doNotReconnectOnceDisconnected=True,
            timeOutInMs=5000,
            commThreadCycleInMs=5)

        if self.__id != simx_return_ok:
            raise SimConnectionException("Failed to connect to CoppeliaSim")

    def id(self) -> int:
        return self.__id

    def end_connection(self) -> None:
        sim.simxFinish(self.__id)

    def start_simulation(self) -> None:
        sim.simxStartSimulation(self.__id, simx_opmode_oneshot_wait)

    def pause_simulation(self) -> None:
        sim.simxPauseSimulation(self.__id, simx_opmode_oneshot_wait)

    def stop_simulation(self) -> None:
        sim.simxStopSimulation(self.__id, simx_opmode_oneshot_wait)

    def get_object_handle(self, name: str) -> int | None:
        code, handle = sim.simxGetObjectHandle(self.__id, name, simx_opmode_oneshot_wait)
        if code == simx_return_ok:
            return handle
        else:
            raise SimHandleException("Component {0} error".format(name))
