from MPU6050 import MPU6050
from os import getpid
from sys import argv
from time import sleep
import smbus

RUN = 1

oldYaw = 0
oldRoll = 0
oldPitch = 0

mpu = None

if __name__ == "__main__":
    if argv[1] == "true":
        print("\033[93mINIT MPU6050...\n\033[00m{")
        mpu = MPU6050(1, 0x68, a_debug=True)
        mpu.dmp_initialize()
        mpu.set_DMP_enabled(True)
        print("}\033[93m\nINIT COMPLETE!\033[00m")
    else:
        print("\033[93mINIT MPU6050...\033[00m")
        mpu = MPU6050(1, 0x68, a_debug=False)
        mpu.dmp_initialize()
        mpu.set_DMP_enabled(True)
        print("\033[93mINIT COMPLETE!\033[00m")

    
    FIFO_buffer = [0] * 64
    packet_size = mpu.DMP_get_FIFO_packet_size()

    while RUN:
        try:
            FIFO_count = mpu.get_FIFO_count()
            mpu_int_status = mpu.get_int_status()

            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                mpu.reset_FIFO()

            elif mpu_int_status & 0x02:

                while FIFO_count < packet_size:
                    FIFO_count = mpu.get_FIFO_count()

                FIFO_buffer = mpu.get_FIFO_bytes(packet_size)

                quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav = mpu.DMP_get_gravity(quat)

                roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)

                roll  = int(roll_pitch_yaw.x)
                pitch = int(roll_pitch_yaw.y)
                yaw   = (int(roll_pitch_yaw.z) + 2) * 2

                '''
                if oldRoll != roll:
                    print("\tROLL: \033[92m{0}\033[00m".format(roll))
                    oldRoll = roll
                    sep = True

                if oldPitch != pitch:
                    print("\tPITCH: \033[92m{0}\033[00m".format(pitch))
                    oldPitch = pitch
                    sep = True
                '''

                if oldYaw != yaw:
                    print("\tYAW: \033[92m{0}\033[00m".format(yaw))
                    oldYaw = yaw

                sleep(0.1)

        except KeyboardInterrupt:
            RUN = 0
            print("\n\n\033[91mSIGINT\033[0m has arrived, process \033[92m{0}\033[00m [\033[93m{1}\033[00m] -> \033[91minterrupted..\033[0m".format(argv[0], getpid()))

    
