from time import sleep
# from ultrasonic import Ultrasonic
# from motor import Motor, Command
# from buzzer import Buzzer
# from led import Led
# from ADC import Adc
# from FreenoveRobot.lib.MPU6050lib.MPUSensor import MPUSensor as MPU

from physiscal_body import PhysicalBody

pb = PhysicalBody()
pb.begin()
pb.magic_rainbow(True)
pb.trill()
sleep(2)
pb.interrupt_trill()
pb.virtual_destructor()





































"""m = MPU(bus=1, debug=True)
m.begin()

try:
    while True:
        print(m.roll_pitch_yaw)
        sleep(5)
except KeyboardInterrupt:
    m.virtual_destructor()"""















"""adc = Adc()

vals = adc.recvADC()
print(vals)"""





"""led = Led()
led.rainbowCycle()"""







"""b = Buzzer(12)
b.emit()
sleep(1)
b.stop()
b.play(50, 440)
sleep(1)
b.emit()
sleep(1)
b.stop()"""






"""front = Ultrasonic(22, 27)
right = Ultrasonic(5, 6)
left = Ultrasonic(13, 26)

print(front.distance)
print(right.distance)
print(left.distance)"""



"""m = Motor()

m.setMotorModel(Command.RUN, 1000)
sleep(2)
m.setMotorModel(Command.STOP)
sleep(2)
m.setMotorModel(Command.ROTATEL, 1000)
sleep(2)
m.setMotorModel(Command.ROTATER, 1000)
sleep(2)
m.setMotorModel(Command.STOP)"""
