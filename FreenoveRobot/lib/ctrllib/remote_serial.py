import json
import sys
import traceback
import ctypes
import inspect

from redis import Redis
from lib.librd.redisdata import RemoteControllerData as rcData

from serial import Serial
from serial.threaded import ReaderThread
from serial.threaded import LineReader


class RemoteReader(ReaderThread):
    def __init__(self, serial_instance, protocol_factory, name):
        super().__init__(serial_instance, protocol_factory)

        self.name = name

    def __tid(self):
        if self.is_alive():
            return ctypes.c_long(self.ident)

        print(f'Thread {self.name} is not alive')
        return None

    def bury(self):
        tid = self.__tid()
        if tid != None:
            exctype = SystemExit

            if not inspect.isclass(exctype):
                exctype = type(exctype)

            res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))

            if res == 1:
                print(f'Thread {self.name} burried')
            elif res == 0:
                raise ValueError("invalid thread id")
            elif res != 1:
                ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
                raise SystemError("PyThreadState_SetAsyncExc failed")


class RemoteEmitter(LineReader):
    __redis: Redis = None

    def connection_made(self, transport):
        super(RemoteEmitter, self).connection_made(transport)

        self.__redis = Redis(host=rcData.Connection.Host, port=rcData.Connection.Port, decode_responses=True)

    def handle_line(self, data):
        if str(data).isdecimal():
            rcData.on_values(None, int(data))
        else:
            rcData.on_values(data, None)

        self.__redis.set(rcData.Key.RC, rcData.values)
        self.__redis.publish(rcData.Topic.Remote, rcData.Key.RC)

    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')


class RemoteController:
    __runner: RemoteReader = None

    @classmethod
    def begin(cls):
        cls.__runner = RemoteReader(Serial('/dev/ttyUSB0', baudrate=9600), RemoteEmitter, 'RemoteDiscover')
        cls.__runner.start()

    @classmethod
    def stop(cls):
        if cls.__runner is not None:
            cls.__runner.bury()
