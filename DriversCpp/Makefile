CXX = g++

APP = run

all: $(APP)

$(APP): main.o controller.o led.o rover.o roverhead.o motor.o servo.o adc.o PCA9585.o  buzzer.o infrared.o ultrasonic.o
	$(CXX) -O -Wall main.o led.o controller.o rover.o roverhead.o motor.o servo.o adc.o PCA9585.o  buzzer.o infrared.o ultrasonic.o -lX11 -lws2811 -li2c -lbcm2835 -o $(APP)

main.o: main.cpp
	$(CXX) -Wall -c main.cpp

motor.o: motor.cpp motor.h
	$(CXX) -Wall -c motor.cpp

PCA9585.o: PCA9585.cpp PCA9585.h
	$(CXX) -Wall -c PCA9585.cpp

rover.o: rover.cpp rover.h
	$(CXX) -Wall -c rover.cpp

roverhead.o: roverhead.cpp roverhead.h
	$(CXX) -Wall -c roverhead.cpp

buzzer.o: buzzer.cpp buzzer.h
	$(CXX) -Wall -c buzzer.cpp

ultrasonic.o: ultrasonic.cpp ultrasonic.h
	$(CXX) -Wall -c ultrasonic.cpp

infrared.o: infrared.cpp infrared.h
	$(CXX) -Wall -c infrared.cpp

servo.o: servo.cpp servo.h
	$(CXX) -Wall -c servo.cpp

adc.o: adc.cpp adc.h
	$(CXX) -Wall -c adc.cpp

led.o: led.cpp led.h
	$(CXX) -Wall -c led.cpp

controller.o: controller.cpp controller.h
	$(CXX) -Wall -c controller.cpp

clean:
	@rm -f led.o
	@rm -f servo.o
	@rm -f adc.o
	@rm -f PCA9585.o
	@rm -f motor.o
	@rm -f main.o
	@rm -f ultrasonic.o
	@rm -f buzzer.o
	@rm -f rover.o
	@rm -f infrared.o
	@rm -f roverhead.o
	@rm -f controller.o
	@echo "Objs cleaned"

cleanAll:
	@rm -f $(APP)
	@rm -f controller.o
	@rm -f led.o
	@rm -f servo.o
	@rm -f PCA9585.o
	@rm -f motor.o
	@rm -f main.o
	@rm -f ultrasonic.o
	@rm -f buzzer.o
	@rm -f roverhead.o
	@rm -f rover.o
	@rm -f infrared.o
	@rm -f adc.o
	@echo "All cleaned"
