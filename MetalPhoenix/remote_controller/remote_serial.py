from serial import Serial
from redis import Redis


serial = Serial('/dev/ttyUSB0', 9600)
redis  = Redis('localhost', 6379, decode_responses=True)


try:
    while True:
        msg = serial.readline().decode('utf-8').replace('\n', '').replace('\r', '')
        redis.publish("CMD", msg)
except KeyboardInterrupt as kt:
    pass
