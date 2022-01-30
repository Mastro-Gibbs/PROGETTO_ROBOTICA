#ifndef ULTRASONIC_H
#define ULTRASONIC_H


#include <chrono>
#include <sys/time.h>
#include <ctime>
#include <algorithm>
#include <bcm2835.h>

#define SONIC_SPEED 34300
#define USONIC_TRIGGER_PIN RPI_BPLUS_GPIO_J8_13
#define USONIC_ECHO_PIN    RPI_BPLUS_GPIO_J8_15

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::system_clock;
using std::sort;

class Ultrasonic
{
    public:
        Ultrasonic( int echoPin, int triggerPin );
        ~Ultrasonic();

        float distance( unsigned int samplesAccuracy = 5, unsigned int timeout = 100 );

        void send_ping();

        void receive_ping( unsigned int timeout );

    private:
        unsigned long start;
        unsigned long stop;

        int echo;
        int trigger;

};


#endif
