from lib.MPU6050lib.MPU6050 import MPU6050
from lib.robotAPI.utils import thread_ripper
from os import getpid
from sys import argv
from time import sleep
from threading import Thread
import inspect
import ctypes as ct
import smbus

FIFO_buffer: list = [0] * 128
packet_size: int = 0


class MPUSensorException(Exception):
    pass


class MPUSensor:
    def __init__(self, bus: int = 1, debug: bool = True):
        global packet_size

        self.__mpu = MPU6050(bus, a_debug=debug)
        self.__mpu.set_FIFO_enabled(True)
        result, code_error = self.__mpu.dmp_initialize()

        if not result:
            raise MPUSensorException('MPU6050 init failed')

        self.__mpu.set_DMP_enabled(True)

        packet_size = self.__mpu.DMP_get_FIFO_packet_size()

        self.__roll: int = 0.0
        self.__pitch: int = 0.0
        self.__yaw: int = 0.0

        self.__discover = Thread(target=self.__update_vals, name='mpu_discover')
        
    def begin(self):
        self.__discover.start()

    def virtual_destructor(self):
        try:
            if thread_ripper(self.__discover):
                print(f"\nThread {self.__discover.name} from MPUSensor instance buried")
        except ValueError or SystemError as error:
            print(f"Issues while trying to kill the thread {self.__discover.name}")

    def __update_vals(self):
        global FIFO_buffer
        global packet_size

        stack: list    = list()
        stack_ptr: int = 0

        while True:
            FIFO_count = self.__mpu.get_FIFO_count()
            mpu_int_status = self.__mpu.get_int_status()

            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                self.__mpu.reset_FIFO()

            elif mpu_int_status & 0x02:

                while FIFO_count < packet_size:
                    FIFO_count = self.__mpu.get_FIFO_count()

                FIFO_buffer = self.__mpu.get_FIFO_bytes(packet_size)

                accel = self.__mpu.DMP_get_acceleration_int16(FIFO_buffer)
                quat = self.__mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav = self.__mpu.DMP_get_gravity(quat)

                roll_pitch_yaw = self.__mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)

                self.__roll  = int(roll_pitch_yaw.x)
                self.__pitch = int(roll_pitch_yaw.y)
                yaw   = int(roll_pitch_yaw.z * 2 + 4)

                if stack_ptr == 10:
                    sum = 0
                    for i in range(0, 8, 1):
                        sum += stack[i]
                        sum = sum / 9
                    
                    if sum + 2 > yaw:
                        pass
                    elif yaw > stack[9] + 15:
                        pass
                    elif yaw + 15 < stack[9]:
                        pass
                    else:
                        stack = stack[1:]
                        stack.append(yaw)
                        self.__yaw = yaw

                else:
                    stack.append(yaw)
                    stack_ptr += 1
                
                

                print(self.__yaw)
        

    @property
    def roll_pitch_yaw(self) -> tuple:
        return self.__roll, self.__pitch, self.__yaw


