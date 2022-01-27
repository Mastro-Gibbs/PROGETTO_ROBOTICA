#ifndef BUZZER_H
#define BUZZER_H

#include <bcm2835.h>

#define BUZZER_PIN  RPI_BPLUS_GPIO_J8_11
#define PWM_CHANNEL 1
#define RANGE       1200

class Buzzer
{
    public:
        Buzzer();
        ~Buzzer();

        void ffStart();

        void fStart( uint16_t duty );

        void stop();
};

#endif


