from enum import Enum
from time import sleep
from redis import Redis
from physiscal_body import PhysicalBody as Body

class KEYS(str, Enum):
    MOTORS = 'MOTORS',
    INFRARED = 'INFRARED',
    ULTRASONIC = 'ULTRASONIC',
    BUZZER = 'BUZZER',
    LED = 'LED'


class VirtualBody:
    def __init__(self) -> None:
        
        # physical_body instance
        self.__body = Body()

        # redis instance
        self.__redis = Redis(host='localhost', port=6379, decode_responses=True)
        self.__pubsub = self.__redis.pubsub()
        self.__pubsub.psubscribe(**{'C_TOPIC': self.__on_message})

    def loop(self):
        self.__body.begin()
        self.__runner = self.__pubsub.run_in_thread(sleep_time=0.01)

        while True:
            sleep(1)

    def stop(self):
        self.__body.virtual_destructor()
        self.__runner.stop()

    def __on_message(self, msg):
        _key = msg['data']
        _value = self.__redis.get(_key)

        if _key == KEYS.BUZZER.value:
            if _value == 'STOP':
                self.__body.interrupt_trill()
            elif _value == 'EMIT':
                self.__body.trill()
        elif _key == KEYS.LED.value:
            if _value == 'STOP':
                self.__body.interrupt_magic_rainbow()
            elif _value == 'EMIT':
                self.__body.magic_rainbow(True)
        elif _key == KEYS.MOTORS.value:
            _values = _value.split(';')
            _cmd = _values[0]
            _val = int(_values[1])

            self.__body.set_motor_model(_cmd, _val)



if __name__ == "__main__":
    v = VirtualBody()
    try:
        v.loop()
    except KeyboardInterrupt:
        v.stop()