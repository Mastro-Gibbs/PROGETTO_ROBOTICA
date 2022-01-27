#ifndef INFRARED_H
#define INFRARED_H

#include <bcm2835.h>
#include <inttypes.h>

#define SX_IR RPI_BPLUS_GPIO_J8_08
#define CX_IR RPI_BPLUS_GPIO_J8_10
#define DX_IR RPI_BPLUS_GPIO_J8_16

class Infrared
{
    public:
        Infrared();
        ~Infrared();

        uint8_t echoSX();

        uint8_t echoCX();

        uint8_t echoDX();
};

#endif // INFRARED_H
