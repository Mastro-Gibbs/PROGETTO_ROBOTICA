import json
import sys
import traceback

from serial import Serial
from serial.threaded import ReaderThread
from serial.threaded import LineReader

from redis import Redis


class RemoteEmitter(LineReader):
    RC_KEY = 'RC_KEY'

    redis = Redis('localhost', 6379, decode_responses=True)

    def connection_made(self, transport):
        super(RemoteEmitter, self).connection_made(transport)

    def handle_line(self, data):
        out = dict()

        if str(data).isdecimal():
            out['command'] = 'null'
            out['speed'] = int(data)
        else:
            out['command'] = data
            out['speed'] = 'null'

        self.redis.set(self.RC_KEY, json.dumps(out, indent=0))
        self.redis.publish("RC", self.RC_KEY)

    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')


runner = ReaderThread(Serial('/dev/ttyUSB0', baudrate=9600), RemoteEmitter)
runner.start()

try:
    runner.join()
except KeyboardInterrupt as kt:
    pass
