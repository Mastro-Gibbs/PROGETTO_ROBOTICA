import sys
import traceback

from redis import Redis
from redis.exceptions import ConnectionError as RedisConnError

from lib.workerthread import RobotThread
from lib.librd.redisdata import RemoteControllerData as rcData


from serial import Serial
from serial.threaded import ReaderThread
from serial.threaded import LineReader


class RCException(Exception):
    pass


class RemoteReader(ReaderThread, RobotThread):
    def __init__(self, serial_instance, protocol_factory, name):
        super().__init__(serial_instance, protocol_factory)
        self.name = name

    def close(self):
        if self.protocol is not None:
            self.protocol.close()

    def allow(self):
        if self.protocol is not None:
            self.protocol.allow()

    def dismiss(self):
        if self.protocol is not None:
            self.protocol.dismiss()


class RemoteEmitter(LineReader):
    __redis: Redis = None
    __allow: bool = False

    def __init__(self):
        super().__init__()

    def connection_made(self, transport):
        super(RemoteEmitter, self).connection_made(transport)

        try:
            self.__redis = Redis(host=rcData.Connection.Host, port=rcData.Connection.Port, decode_responses=True)
        except RedisConnError or OSError or ConnectionRefusedError:
            raise RCException(f'Unable to connect to redis server at: '
                                f'{rcData.Connection.Host}:{rcData.Connection.Port}')

    def handle_line(self, data):
        if self.__allow:
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

    def close(self):
        if self.__redis is not None:
            self.__redis.close()

    def allow(self):
        self.__allow = True

    def dismiss(self):
        self.__allow = False


class RemoteController:
    __runner: RemoteReader = None

    def begin(self):
        self.__runner = RemoteReader(Serial('/dev/ttyUSB0', baudrate=9600), RemoteEmitter, 'RemoteDiscover')
        self.__runner.start()

    def allow(self):
        self.__runner.allow()

    def dismiss(self):
        self.__runner.dismiss()

    def stop(self):
        if self.__runner is not None:
            self.__runner.bury()
            self.__runner.close()
