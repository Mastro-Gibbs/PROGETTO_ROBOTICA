from time import sleep

from redis import Redis
from redis.client import PubSubWorkerThread

from physiscal_body import PhysicalBody as Body

from lib.ctrllib.enums import RedisKEYS as RK
from lib.ctrllib.enums import RedisTOPICS as RT
from lib.ctrllib.enums import RedisCONNECTION as RC
from lib.ctrllib.enums import RedisCOMMAND as RCMD
from lib.robotAPI.utils import thread_ripper

from threading import Thread


class VirtualBody:
    def __init__(self) -> None:
        
        # physical_body instance
        self.__body = Body()

        # redis instance
        self.__redis = Redis(host=RC.HOST.value, port=int(RC.PORT.value), decode_responses=True)
        self.__pubsub = self.__redis.pubsub()
        self.__pubsub.psubscribe(**{RT.CTRL_TOPIC.value: self.__on_message})


    def begin(self) -> bool:
        self.__redis_runner: PubSubWorkerThread = self.__pubsub.run_in_thread(sleep_time=0.01)
        self.__yaw_thread: Thread = Thread(target=self.__yaw_discover, name='yaw_thread')
        self.__infrared_thread: Thread = Thread(target=self.__infrared_discover, name='infrared_thread')
        self.__ultrasonic_thread: Thread = Thread(target=self.__ultrasonic_discover, name='ultrasonic_thread')

        self.__body.begin()
        self.__yaw_thread.start()
        self.__infrared_thread.start()
        self.__ultrasonic_thread.start()

        if self.__yaw_thread.is_alive() and self.__infrared_thread.is_alive() and \
            self.__ultrasonic_thread.is_alive() and self.__redis_runner.is_alive():
            return True
        else:
            self.__redis_runner.stop()
            thread_ripper(self.__yaw_thread)
            thread_ripper(self.__infrared_thread)
            thread_ripper(self.__ultrasonic_thread)

        return False


    def loop(self) -> None:
        while True:
            VirtualBody.__dummy_function()

    
    def stop(self) -> None:
        self.__body.virtual_destructor()
        self.__redis_runner.stop()
        thread_ripper(self.__yaw_thread)
        thread_ripper(self.__infrared_thread)
        thread_ripper(self.__ultrasonic_thread)

    
    def __on_message(self, msg):
        _key = msg['data']
        _value = self.__redis.get(_key)

        if _key == RK.BUZZER.value:
            if _value == RCMD.BZZEMIT.value:
                self.__body.trill()
            elif _value == RCMD.BZZINTERRUPT.value:
                self.__body.interrupt_trill()
                
        elif _key == RK.LED.value:
            if _value == RCMD.LEDEMIT.value:
                self.__body.magic_rainbow(True)
            elif _value == RCMD.LEDINTERRUPT.value:
                self.__body.interrupt_magic_rainbow()

        elif _key == RK.MOTORS.value:
            _values = _value.split(';')
            _cmd = _values[0]
            _val = int(_values[1])

            self.__body.set_motor_model(_cmd, _val)


    def __yaw_discover(self):
        _OLD_YAW: str = str()

        while True:
            _orientation = self.__body.oritentation()
            _yaw = str(_orientation[2])

            if _OLD_YAW != _yaw:
                self.__redis.set(RK.MPU.value, _yaw)
                self.__redis.publish(RT.BODY_TOPIC.value, RK.MPU.value)
                _OLD_YAW = _yaw
            
            sleep(0.005)

    def __infrared_discover(self):
        _SENSOR_RETURN_TRUE = False
        _OLD_INFRARED_MSG: str = str()

        while not _SENSOR_RETURN_TRUE:
            _infrared = self.__body.infrared_status()
            _infrared_msg = ';'.join([str(_infrared[0]), str(_infrared[1]), str(_infrared[2])])

            if _infrared_msg != _OLD_INFRARED_MSG:
                self.__redis.set(RK.INFRARED.value, _infrared_msg)
                self.__redis.publish(RT.BODY_TOPIC.value, RK.INFRARED.value)
                _OLD_INFRARED_MSG = _infrared_msg

            if _infrared[0] and _infrared[1] and _infrared[2]:
                _SENSOR_RETURN_TRUE = True
            
            sleep(0.005)

    def __ultrasonic_discover(self):
        _OLD_DISTANCES: str = str()

        while True:
            _distances = self.__body.read_distances()
            _distances = ';'.join([str(_distances[0]), str(_distances[1]), str(_distances[2])])

            if _OLD_DISTANCES != _distances:
                self.__redis.set(RK.ULTRASONIC.value, _distances)
                self.__redis.publish(RT.BODY_TOPIC.value, RK.ULTRASONIC.value)
                _OLD_DISTANCES = _distances

            sleep(0.005)

    @staticmethod
    def __dummy_function():
        sleep(0.5)



if __name__ == "__main__":
    v = VirtualBody()
    try:
        while not v.begin():
            sleep(5)

        v.loop()
    except KeyboardInterrupt:
        v.stop()