from lib.MPU6050lib.MPU6050 import MPU6050
from lib.robotAPI.utils import thread_ripper
from os import getpid, stat
from sys import argv, stdout
from time import sleep
from threading import Thread
import inspect
import ctypes as ct
import smbus
import scipy.stats as stats
from array import array

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

        self.__stacked_values: array = array('i', [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2])

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

        checker: bool = True

        while True:
            FIFO_count = self.__mpu.get_FIFO_count()
            mpu_int_status = self.__mpu.get_int_status()

            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                self.__mpu.reset_FIFO()

            elif mpu_int_status & 0x02:

                while FIFO_count < packet_size:
                    FIFO_count = self.__mpu.get_FIFO_count()

                FIFO_buffer = self.__mpu.get_FIFO_bytes(packet_size)

                quat = self.__mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav = self.__mpu.DMP_get_gravity(quat)

                roll_pitch_yaw = self.__mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)

                yaw   = int(roll_pitch_yaw.z * 2 + 4)
                
                for i in range(0, 15, 1):
                    if yaw == self.__stacked_values[i]:
                        checker = False
                        break
                
                if checker and self.__yaw != yaw:
                    #print("ARRAY POST APPENDING: ", self.__stacked_values)
                    self.__stacked_values = array('i', self.__stacked_values + array('i', [yaw]))

                    zscore = stats.zscore(self.__stacked_values)
                    #print("ZSCORE: ", zscore)

                    if -2.9 < zscore[15] < 2.9:
                        self.__yaw = yaw
                        self.__stacked_values = self.__stacked_values[1:]
                    else:
                        self.__stacked_values = self.__stacked_values[:-1]
                    #print("ARRAY AFTER PROCESSING: ", self.__stacked_values)

                #print("RAW YAW: ", yaw)
                #stdout.write("\rSETTED YAW: %d   " %self.__yaw)
                #stdout.flush()
                #print('\n')
                checker = True
                

    @property
    def roll_pitch_yaw(self) -> tuple:
        return self.__roll, self.__pitch, self.__yaw


