from time import sleep, time
from sys import stdout

from redis import Redis
from redis.client import PubSubWorkerThread
from redis.exceptions import ConnectionError as RedisConnError

from physiscal_body import PhysicalBody as Body

from lib.ctrllib.enums import RedisKEYS as RK
from lib.ctrllib.enums import RedisTOPICS as RT
from lib.ctrllib.enums import RedisCONNECTION as RC
from lib.ctrllib.enums import RedisCOMMAND as RCMD
from lib.workerthread import RobotThread
from lib.robotAPI.motor import MOTORSCommand
from lib.exit_codes import CALIB_ERROR



class BodyException(Exception):
    pass


class VirtualBody:
    def __init__(self) -> None:
        
        # physical_body instance
        self.__body = Body()

        try:
            self.__redis = Redis(host=RC.HOST.value, port=int(RC.PORT.value), decode_responses=True)
            self.__pubsub = self.__redis.pubsub()
            self.__pubsub.psubscribe(**{RT.CTRL_TOPIC.value: self.__on_message})
        except RedisConnError or OSError or ConnectionRefusedError:
            raise BodyException(f'Unable to connect to redis server at: {RC.HOST.value}:{RC.PORT.value}')

        self.__rotation_thread: RobotThread = RobotThread(target=self.__rotation, name='rotation_thread')
        self.__rotation_ack: int = 0

    def calibrate_mpu(self) -> bool:
        print('\nWelcome to robot calibration!\n')
        print('Move the robot very slowly, making small swings left and right')
        
        success = True

        self.__body.begin()

        _yaw = self.__body.orientation()[2]

        while _yaw == 0.0 or -8 < _yaw < 8:
            sleep(0.1)
            _yaw = self.__body.orientation()[2]
            
            stdout.write('\rCURRENT MPU VALUE: %d   ' %_yaw)
            stdout.flush()
        
        print('\nMove the robot slowly to the right af around 90 degrees')

        begin_time = time()
        while not (85 < _yaw < 95):
            sleep(0.05)
            _yaw = self.__body.orientation()[2]
            stdout.write('\rCURRENT MPU VALUE: %d   ' %_yaw)
            stdout.write('TARGET: %d   ' %90)
            stdout.flush()

            if time() - begin_time > 20:
                success = False
                break
        
        if success:
            print('\nMove the robot slowly to the left af around -90 degrees')

            begin_time = time()
            while not (-85 > _yaw > -95):
                sleep(0.05)
                _yaw = self.__body.orientation()[2]
                stdout.write('\rCURRENT MPU VALUE: %d   ' %_yaw)
                stdout.write('TARGET: %d   ' %-90)
                stdout.flush()

                if time() - begin_time > 20:
                    success = False
                    break

        if success:
            print('\nCalibration compleated!')
            print('Place the robot on the ground at approximately angle 0°')
        else:
            print('\nCalibration failed!, exiting..')
            self.stop()
            exit(CALIB_ERROR)


    def begin(self) -> bool:
        self.__redis_runner: PubSubWorkerThread = self.__pubsub.run_in_thread(sleep_time=0.01)
        self.__yaw_thread: RobotThread = RobotThread(target=self.__yaw_discover, name='yaw_thread')
        self.__infrared_thread: RobotThread = RobotThread(target=self.__infrared_discover, name='infrared_thread')
        self.__ultrasonic_thread: RobotThread = RobotThread(target=self.__ultrasonic_discover, name='ultrasonic_thread')

        self.__body.begin()
        self.__yaw_thread.start()
        self.__infrared_thread.start()
        self.__ultrasonic_thread.start()

        if self.__yaw_thread.is_alive() and self.__infrared_thread.is_alive() and \
            self.__ultrasonic_thread.is_alive() and self.__redis_runner.is_alive():
            return True
        else:
            self.__redis_runner.stop()
            self.__yaw_thread.bury()
            self.__infrared_thread.bury()
            self.__ultrasonic_thread.bury()

        return False


    def loop(self) -> None:
        while True:
            VirtualBody.__dummy_function()

    
    def stop(self) -> None:
        self.__body.virtual_destructor()
        self.__redis_runner.stop()

        self.__yaw_thread.bury()
        self.__infrared_thread.bury()
        self.__ultrasonic_thread.bury()

    
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
            if _cmd == MOTORSCommand.ROTATEL.value or _cmd == MOTORSCommand.ROTATER.value:
                _until = int(_values[2])
                self.__rotation_thread: RobotThread = RobotThread(target=self.__rotation, name='rotation_thread', args=(_until,))
                self.__rotation_thread.start()
                
            

    def __rotation(self, until: int):
        print('Rotating routine')

        EXIT = False
        SUCCESS = False
        self.__rotation_ack += 1

        delta = 5
        until = abs(until)
 
        start_time = time()
        timeout = 15

        while not EXIT:
            _yaw = abs(self.__body.orientation()[2])
            
            stdout.write('\rCURRENT ANGLE: %d   ' %_yaw)
            stdout.flush()

            if  until - delta < _yaw < until + delta:
                self.__body.set_motor_model(MOTORSCommand.STOP.value)
                EXIT = True  
                SUCCESS = True

            if time() - start_time > timeout:
                self.__body.set_motor_model(MOTORSCommand.STOP.value)
                EXIT = True

        if SUCCESS:
            checking_loops = 5
            while checking_loops:
                if not (until - delta < abs(self.__body.orientation()[2]) < until + delta):
                    SUCCESS = False

                checking_loops -= 1
                sleep(0.05)

        _msg = ';'.join([str(self.__rotation_ack), str(int(SUCCESS))])
        self.__redis.set(RK.ROTATION.value, _msg)
        self.__redis.publish(RT.BODY_TOPIC.value, RK.ROTATION.value)

        print(f'\nTrhead {self.__rotation_thread.name} from VirtualBody instance buried')

    def __yaw_discover(self):
        _OLD_YAW: str = str()

        while True:
            _yaw = str(self.__body.orientation()[2])
            
            if _OLD_YAW != _yaw and _yaw != '0.0':
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

        print(f'Trhead {self.__infrared_thread.name} from VirtualBody instance buried')

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
    virtual_body = None
    try:
        virtual_body = VirtualBody()
        virtual_body.calibrate_mpu()
        sleep(5)
        while not virtual_body.begin():
            print('Missing component, retrying, delay 5 seconds')
            sleep(5)

        print('\nVirtualBody ready\nIgnore possible buried threads messages')
        virtual_body.loop()
    except KeyboardInterrupt:
        pass
    except BodyException as be:
        print(be.args[0])
    finally:
        if virtual_body:
            virtual_body.stop()
