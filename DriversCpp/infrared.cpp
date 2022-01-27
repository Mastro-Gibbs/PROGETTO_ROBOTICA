#include "infrared.h"

Infrared::Infrared()
{
    bcm2835_gpio_fsel( SX_IR, BCM2835_GPIO_FSEL_INPT );
    bcm2835_gpio_fsel( CX_IR, BCM2835_GPIO_FSEL_INPT );
    bcm2835_gpio_fsel( DX_IR, BCM2835_GPIO_FSEL_INPT );
}

Infrared::~Infrared()
{}


uint8_t Infrared::echoSX()
{
    return bcm2835_gpio_lev( SX_IR );
}


uint8_t Infrared::echoCX()
{
    return bcm2835_gpio_lev( CX_IR );
}


uint8_t Infrared::echoDX()
{
    return bcm2835_gpio_lev( DX_IR );
}
