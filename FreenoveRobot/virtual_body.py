from time import sleep
from redis import Redis
from physiscal_body import PhysicalBody as Body

from lib.ctrllib.enums import RedisKEYS as RK
from lib.ctrllib.enums import RedisTOPICS as RT
from lib.ctrllib.enums import RedisCONNECTION as RC
from lib.ctrllib.enums import RedisCOMMAND as RCMD



class VirtualBody:
    def __init__(self) -> None:
        
        # physical_body instance
        self.__body = Body()

        # redis instance
        self.__redis = Redis(host=RC.HOST.value, port=int(RC.PORT.value), decode_responses=True)
        self.__pubsub = self.__redis.pubsub()
        self.__pubsub.psubscribe(**{RT.CTRL_TOPIC.value: self.__on_message})

    def loop(self) -> None:
        self.__body.begin()
        self.__runner = self.__pubsub.run_in_thread(sleep_time=0.01)

        _old_distances = str()
        _old_yaw = str()
        _old_infrared = str()

        while True:
            _distances = self.__body.read_distances()
            _distances = ';'.join([str(_distances[0]), str(_distances[1]), str(_distances[2])])
            
            _orientation = self.__body.oritentation()
            _yaw = str(_orientation[2])

            _infrared = self.__body.infrared_status()
            _infrared = ';'.join([str(_infrared[0]), str(_infrared[1]), str(_infrared[2])])

            if _old_distances != _distances:
                self.__redis.set(RK.ULTRASONIC.value, _distances)
                self.__redis.publish(RT.BODY_TOPIC.value, RK.ULTRASONIC.value)
                _old_distances = _distances

            if _old_yaw != _yaw:
                self.__redis.set(RK.MPU.value, _yaw)
                self.__redis.publish(RT.BODY_TOPIC.value, RK.MPU.value)
                _old_yaw = _yaw

            if _old_infrared != _infrared:
                self.__redis.set(RK.INFRARED.value, _infrared)
                self.__redis.publish(RT.BODY_TOPIC.value, RK.INFRARED.value)
                _old_infrared = _infrared

            sleep(0.01)


    def stop(self) -> None:
        self.__body.virtual_destructor()
        self.__runner.stop()

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



if __name__ == "__main__":
    v = VirtualBody()
    try:
        v.loop()
    except KeyboardInterrupt:
        v.stop()