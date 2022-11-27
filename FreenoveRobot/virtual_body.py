import json

from time import sleep, time
from sys import stdout

from redis import Redis
from redis.client import PubSubWorkerThread
from redis.exceptions import ConnectionError as RedisConnError

from physiscal_body import PhysicalBody as Body

from lib.librd.redisdata import BodyData, RemoteControllerData as rcData

from lib.libctrl.utility import Clockwise

from lib.workerthread import RobotThread
from lib.robotAPI.motor import MOTORSCommand
from lib.exit_codes import CALIB_ERROR


class BodyException(Exception):
    pass


class VirtualBody:
    def __init__(self) -> None:

        # physical_body instance
        self.__body = Body()

        # Threads
        self.__yaw_thread = None
        self.__redis_runner = None
        self.__infrared_thread = None
        self.__ultrasonic_thread = None

        try:
            self.__redis = Redis(host=BodyData.Connection.Host, port=BodyData.Connection.Port, decode_responses=True)
            self.__pubsub = self.__redis.pubsub()
            self.__pubsub.psubscribe(**{BodyData.Topic.Controller: self.__on_message})
            self.__pubsub.psubscribe(**{BodyData.Topic.Remote:     self.__on_remote})
        except RedisConnError or OSError or ConnectionRefusedError:
            raise BodyException(f'Unable to connect to redis server at: '
                                f'{BodyData.Connection.Host}:{BodyData.Connection.Port}')

    def calibrate_mpu(self) -> bool:
        print('\nWelcome to robot calibration!\n')
        print('Move the robot very slowly, making small swings left and right')

        success = True

        self.__body.begin()

        _yaw = self.__body.orientation()[2]

        while _yaw == 0.0 or -8 < _yaw < 8:
            sleep(0.1)
            _yaw = self.__body.orientation()[2]

            stdout.write('\rCURRENT MPU VALUE: %d   ' % _yaw)
            stdout.flush()

        print('\nMove the robot slowly to the right af around 90 degrees')

        begin_time = time()
        while not (85 < _yaw < 95):
            sleep(0.05)
            _yaw = self.__body.orientation()[2]
            stdout.write('\rCURRENT MPU VALUE: %d   ' % _yaw)
            stdout.write('TARGET: %d   ' % 90)
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
                stdout.write('\rCURRENT MPU VALUE: %d   ' % _yaw)
                stdout.write('TARGET: %d   ' % -90)
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

        return success

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

        if _key == BodyData.Key.Buzzer:
            status = int(_value)

            if status:
                self.__body.trill()
            else:
                self.__body.interrupt_trill()

        elif _key == BodyData.Key.Led:
            data = json.loads(_value)
            BodyData.Led.set(int(data['status']))
            BodyData.Led.on_arrow(int(data['arrow']), int(data['cw']))

            if BodyData.Led.status():
                if BodyData.Led.arrow:          # LED ON, ARROW ON
                    self.__body.blink_car_arrow(clockwise=BodyData.Led.direction)
                elif not BodyData.Led.arrow:    # LED ON, ARROW OFF
                    self.__body.magic_rainbow(True)
            else:                               # LED OFF
                self.__body.interrupt_led()

        elif _key == BodyData.Key.Motor:
            BodyData.Motor.on_values(_value)

            if BodyData.Motor.changed:
                self.__body.set_tuple_motor_model(BodyData.Motor.values)

    def __on_remote(self, msg):
        _key = msg['data']
        _value = self.__redis.get(_key)

        if _key == rcData.Key.RC:
            data = json.loads(_value)
            rcData.on_values(data['rc_cmd'], data['rc_spd'])

            if rcData.is_valid:
                self.__body.set_tuple_motor_model(rcData.get_motor())

    def __yaw_discover(self):
        while True:
            _yaw = self.__body.orientation()[2]

            BodyData.Yaw.on_value(_yaw)

            if BodyData.Yaw.changed:
                self.__redis.set(BodyData.Key.MPU, BodyData.Yaw.value)
                self.__redis.publish(BodyData.Topic.Body, BodyData.Key.MPU)

            sleep(0.005)

    def __infrared_discover(self):
        _SENSOR_RETURN_TRUE = False

        while not _SENSOR_RETURN_TRUE:
            _irL, _irM, _irR = self.__body.infrared_status()

            BodyData.Infrared.on_values(_irL, _irM, _irR)

            if BodyData.Infrared.changed:
                self.__redis.set(BodyData.Key.Ultrasonic, BodyData.Infrared.values)
                self.__redis.publish(BodyData.Topic.Body, BodyData.Key.Ultrasonic)

            if _irL and _irM and _irR:
                _SENSOR_RETURN_TRUE = True

            sleep(0.005)

        print(f'Trhead {self.__infrared_thread.name} from VirtualBody instance buried')

    def __ultrasonic_discover(self):
        while True:
            _proxL, _proxF, _proxR = self.__body.read_distances()

            BodyData.Ultrasonic.on_values(_proxL, _proxF, _proxR)

            if BodyData.Ultrasonic.changed:
                self.__redis.set(BodyData.Key.Ultrasonic, BodyData.Ultrasonic.values)
                self.__redis.publish(BodyData.Topic.Body, BodyData.Key.Ultrasonic)

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
