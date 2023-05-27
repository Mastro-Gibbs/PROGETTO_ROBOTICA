import json

from time import sleep, time
from sys import stdout

from redis import Redis
from redis.client import PubSubWorkerThread
from redis.exceptions import ConnectionError as RedisConnError

from physiscal_body import PhysicalBody as Body, PhysicalBodyException

from lib.librd.redisdata import BodyData, RemoteControllerData as rcData

from lib.libctrl.utility import Logger, CFG
from lib.libctrl.enums import Color
from lib.workerthread import RobotThread


class BodyException(Exception):
    pass


class VirtualBody:
    _LOGGER_DATA = CFG.logger_data()

    def __init__(self) -> None:
        self.__logger = Logger('VirtualBody  ', self._LOGGER_DATA["SEVERITY"], Color.YELLOW)
        self.__logger.set_logfile(self._LOGGER_DATA["BLOGFILE"])

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
            self.__pubsub.psubscribe(**{BodyData.Topic.Remote: self.__on_remote})
        except RedisConnError or OSError or ConnectionRefusedError:
            raise BodyException(f'Unable to connect to redis server at: '
                                f'{BodyData.Connection.Host}:{BodyData.Connection.Port}')

    def __calibrate_mpu(self) -> bool:
        self.__logger.log('MPU6050 calibration', Color.YELLOW)
        self.__logger.log('Move the robot very slowly, making small swings left and right', Color.GRAY)

        success = True

        _yaw = self.__body.orientation()[2]

        while _yaw == 0.0 or -8 < _yaw < 8:
            sleep(0.1)
            _yaw = self.__body.orientation()[2]

            stdout.write('\rCURRENT MPU VALUE: %d   ' % _yaw)
            stdout.flush()

            self.__logger.log(f'Current Z-Axis value: {_yaw}', Color.GRAY, _stdout=False)

        self.__logger.log('\nMove the robot slowly to the right af around 90 degrees', Color.GRAY)

        begin_time = time()
        while not (85 < _yaw < 95):
            sleep(0.05)
            _yaw = self.__body.orientation()[2]
            stdout.write('\rCURRENT MPU VALUE: %d   ' % _yaw)
            stdout.write('TARGET: %d   ' % 90)
            stdout.flush()

            self.__logger.log(f'Current Z-Axis value: {_yaw}', Color.GRAY, _stdout=False)
            self.__logger.log(f'Target  Z-Axis value: {90}', Color.GRAY, _stdout=False)

            if time() - begin_time > 20:
                success = False
                break

        if success:
            self.__logger.log('\nMove the robot slowly to the left af around -90 degrees', Color.GRAY)

            begin_time = time()
            while not (-85 > _yaw > -95):
                sleep(0.05)
                _yaw = self.__body.orientation()[2]
                stdout.write('\rCURRENT MPU VALUE: %d   ' % _yaw)
                stdout.write('TARGET: %d   ' % -90)
                stdout.flush()

                self.__logger.log(f'Current Z-Axis value: {_yaw}', Color.GRAY, _stdout=False)
                self.__logger.log(f'Target  Z-Axis value: {-90}', Color.GRAY, _stdout=False)

                if time() - begin_time > 20:
                    success = False
                    break

        if success:
            self.__logger.log('MPU6050 calibrated', Color.YELLOW)
            self.__logger.log('Place the robot on the ground at approximately angle 0°', Color.GRAY)
        else:
            self.__logger.log('Calibration failed!', Color.RED)

        return success

    def begin(self, callback) -> bool:
        self.__logger.log('Initializing', Color.YELLOW)

        try:
            self.__body.begin(callback, self.__logger)
        except PhysicalBodyException as exc:
            self.__logger.log(f'{exc.args[0]}', Color.GRAY)
            return False

        self.__logger.reset_context()

        if BodyData.Yaw.is_enabled():
            self.__calibrate_mpu()
            self.__yaw_thread: RobotThread = RobotThread(target=self.__yaw_discover, name='yaw_thread')
            self.__yaw_thread.start()

        self.__redis_runner: PubSubWorkerThread = self.__pubsub.run_in_thread(sleep_time=0.01)
        self.__logger.log('Redis thread:      detached', Color.GRAY)
        self.__infrared_thread: RobotThread = RobotThread(target=self.__infrared_discover, name='infrared_thread')
        self.__logger.log('Infrared thread:   detached', Color.GRAY)
        self.__ultrasonic_thread: RobotThread = RobotThread(target=self.__ultrasonic_discover, name='ultrasonic_thread')
        self.__logger.log('Ultrasonic thread: detached', Color.GRAY)

        self.__infrared_thread.start()
        self.__ultrasonic_thread.start()

        if BodyData.Yaw.is_enabled():
            if self.__yaw_thread.is_alive() and self.__infrared_thread.is_alive() and \
                    self.__ultrasonic_thread.is_alive() and self.__redis_runner.is_alive():
                self.__redis.set(BodyData.Key.SELF, json.dumps({'virtB': 1}, indent=0))
                self.__redis.publish(BodyData.Topic.Body, BodyData.Key.SELF)
                self.__logger.log('Initialized', Color.YELLOW)
                return True
            else:
                msg = self.__yaw_thread.bury()
                self.__logger.log(f'{msg}', Color.GRAY)
        else:
            if self.__infrared_thread.is_alive() and \
                    self.__ultrasonic_thread.is_alive() and self.__redis_runner.is_alive():
                self.__redis.set(BodyData.Key.SELF, json.dumps({'virtB': 1}, indent=0))
                self.__redis.publish(BodyData.Topic.Body, BodyData.Key.SELF)
                self.__logger.log('Initialized', Color.YELLOW)
                return True

        self.__logger.log('Initializing failed', Color.GRAY)

        self.__redis_runner.stop()
        self.__redis.close()
        self.__logger.log(f'Redis thread:         buried', Color.GRAY)
        self.__logger.log(f'Redis connection:     closed', Color.GRAY)

        msg = self.__infrared_thread.bury()
        self.__logger.log(f'{msg}', Color.GRAY)
        msg = self.__ultrasonic_thread.bury()
        self.__logger.log(f'{msg}', Color.GRAY)

        return False

    def loop(self) -> None:
        while True:
            VirtualBody.__dummy_function()

    def stop(self) -> None:
        self.__logger.log(f'Arresting', Color.YELLOW, newline=True)
        self.__body.virtual_destructor(self.__logger)
        self.__logger.reset_context()

        self.__redis_runner.stop()
        self.__redis.close()
        self.__logger.log(f'Redis thread:           buried', Color.GRAY)
        self.__logger.log(f'Redis connection:       closed', Color.GRAY)

        if BodyData.Yaw.is_enabled():
            msg = self.__yaw_thread.bury()
            self.__logger.log(f'{msg}', Color.GRAY)

        msg = self.__infrared_thread.bury()
        self.__logger.log(f'{msg}', Color.GRAY)
        msg = self.__ultrasonic_thread.bury()
        self.__logger.log(f'{msg}', Color.GRAY)

        self.__logger.log(f'Arrested', Color.YELLOW, newline=True)

    def __on_message(self, msg):
        _key = msg['data']
        _value = self.__redis.get(_key)

        _eval = _value.replace('\n', '')

        self.__logger.log(f'Received data (Controller): {_key} -> {_eval}', Color.GRAY)

        if _key == BodyData.Key.Buzzer:
            status = int(_value)

            if status:
                self.__body.trill()
            else:
                self.__body.interrupt_trill()

        elif _key == BodyData.Key.Led:
            data = json.loads(_value)
            BodyData.Led.set(int(data['status']))
            BodyData.Led.on_arrow(int(data['arrow']), data['frlb'])

            if BodyData.Led.status():
                if BodyData.Led.arrow():  # LED ON, ARROW ON
                    self.__body.blink_car_arrow(frlb=BodyData.Led.direction())
                elif not BodyData.Led.arrow():  # LED ON, ARROW OFF
                    self.__body.magic_rainbow(True)
            else:  # LED OFF
                self.__body.interrupt_led()

        elif _key == BodyData.Key.Motor:
            BodyData.Motor.on_values(_value)

            if BodyData.Motor.changed():
                self.__body.set_tuple_motor_model(BodyData.Motor.values())

    def __on_remote(self, msg):
        _key = msg['data']
        _value = self.__redis.get(_key)

        _eval = _value.replace('\n', '')

        self.__logger.log(f'Received data (Remote): {_key} -> {_eval}', Color.GRAY)


        if _key == rcData.Key.RC:
            data = json.loads(_value)
            rcData.on_values(data['rc_cmd'], data['rc_spd'])

            if rcData.is_valid():
                self.__body.set_tuple_motor_model(rcData.get_motor())

    def __yaw_discover(self):
        while True:
            _yaw = self.__body.orientation()[2]

            BodyData.Yaw.on_value(_yaw)

            if BodyData.Yaw.changed():
                self.__redis.set(BodyData.Key.MPU, BodyData.Yaw.value())
                self.__redis.publish(BodyData.Topic.Body, BodyData.Key.MPU)

            sleep(0.005)

    def __infrared_discover(self):
        while True:
            _irL, _irM, _irR = self.__body.infrared_status()

            BodyData.Infrared.on_values(_irL, _irM, _irR)

            if BodyData.Infrared.changed():
                self.__redis.set(BodyData.Key.Ultrasonic, BodyData.Infrared.values())
                self.__redis.publish(BodyData.Topic.Body, BodyData.Key.Ultrasonic)

            sleep(0.005)

    def __ultrasonic_discover(self):
        while True:
            _proxL, _proxF, _proxR = self.__body.read_distances()

            BodyData.Ultrasonic.on_values(_proxL, _proxF, _proxR)

            if BodyData.Ultrasonic.changed():
                self.__redis.set(BodyData.Key.Ultrasonic, BodyData.Ultrasonic.values())
                self.__redis.publish(BodyData.Topic.Body, BodyData.Key.Ultrasonic)

            sleep(0.005)

    def on_ready_btn(self):
        self.__redis.set(BodyData.Key.Btn, json.dumps({'ready': 1}, indent=0))
        self.__redis.publish(BodyData.Topic.Body, BodyData.Key.Btn)
        self.__redis.set(BodyData.Key.SELF, json.dumps({'virtB': 1}, indent=0))
        self.__redis.publish(BodyData.Topic.Body, BodyData.Key.SELF)

    @staticmethod
    def __dummy_function():
        sleep(0.5)



if __name__ == "__main__":
    try:
        vb: VirtualBody = VirtualBody()

        def on_pb_pressed(*args):
            vb.on_ready_btn()

        while not vb.begin(on_pb_pressed):
            print('Body init fails')
            sleep(5)

        try:
            vb.loop()
        except KeyboardInterrupt or Exception:
            pass
        finally:
            vb.stop()

    except BodyException as bexc:
        print(bexc.args[0])
    except PhysicalBodyException as pexc:
        print(pexc.args[0])
    except KeyboardInterrupt:
        pass


