#include "buzzer.h"

Buzzer::Buzzer()
{
    bcm2835_gpio_fsel(BUZZER_PIN, BCM2835_GPIO_FSEL_OUTP);
    //bcm2835_gpio_fsel( BUZZER_PIN, BCM2835_GPIO_FSEL_ALT5 );
    //bcm2835_pwm_set_clock( BCM2835_PWM_CLOCK_DIVIDER_16 );
    //bcm2835_pwm_set_mode( PWM_CHANNEL, 1, 1 );
    //bcm2835_pwm_set_range( PWM_CHANNEL, RANGE );
}

Buzzer::~Buzzer()
{}

void Buzzer::ffStart()
{
    bcm2835_gpio_write(BUZZER_PIN, HIGH);
    //bcm2835_pwm_set_data( PWM_CHANNEL, RANGE );
}

void Buzzer::fStart( uint16_t duty )
{
    if ( duty >= RANGE )
        ffStart();
    else
       bcm2835_pwm_set_data( PWM_CHANNEL, duty );
}


void Buzzer::stop()
{
    bcm2835_gpio_write(BUZZER_PIN, LOW);
    //bcm2835_pwm_set_data( PWM_CHANNEL, 0 );
}
