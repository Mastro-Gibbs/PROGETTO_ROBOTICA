#include "ultrasonic.h"


Ultrasonic::Ultrasonic()
{
    bcm2835_gpio_fsel( USONIC_TRIGGER_PIN, BCM2835_GPIO_FSEL_OUTP );
    bcm2835_gpio_fsel( USONIC_ECHO_PIN,    BCM2835_GPIO_FSEL_INPT );

    //bcm2835_gpio_write( USONIC_TRIGGER_PIN, LOW );

    this->start = 0;
    this->stop  = 0;


}


Ultrasonic::~Ultrasonic()
{}

void Ultrasonic::send_ping()
{
    bcm2835_gpio_write( USONIC_TRIGGER_PIN, HIGH );

    bcm2835_delay( 2 );

    bcm2835_gpio_write( USONIC_TRIGGER_PIN, LOW );
}

void Ultrasonic::receive_ping()
{
    while ( bcm2835_gpio_lev( USONIC_ECHO_PIN ) == LOW )
    {
        this->start = duration_cast<microseconds>( system_clock::now().time_since_epoch() ).count();
    }

    while ( bcm2835_gpio_lev( USONIC_ECHO_PIN ) == HIGH )
    {
        this->stop = duration_cast<microseconds>( system_clock::now().time_since_epoch() ).count();
    }
}

float Ultrasonic::distance()
{
    float samples[5] = {0, 0, 0, 0, 0};

    for ( int i = 0; i < 5; i++ )
    {
        send_ping();
        receive_ping();

        long delta = ( this->stop - this->start ) * SONIC_SPEED;
        delta = delta / 2000.0f;

        float res = delta / 1000.0f;
        samples[i] = res;
    }

    sort( samples, samples + 5 );

    return samples[2];
}
