#include "ultrasonic.h"
#include <stdio.h>


Ultrasonic::Ultrasonic( int echoPin, int triggerPin )
{
    bcm2835_gpio_fsel( triggerPin, BCM2835_GPIO_FSEL_OUTP );
    bcm2835_gpio_fsel( echoPin,    BCM2835_GPIO_FSEL_INPT );

    bcm2835_gpio_write( USONIC_TRIGGER_PIN, LOW );

    this->start = 0;
    this->stop  = 0;

    this->echo    = echoPin;
    this->trigger = triggerPin;

}


Ultrasonic::~Ultrasonic()
{}

void Ultrasonic::send_ping()
{
    bcm2835_gpio_write( this->trigger, HIGH );

    bcm2835_delay( 7 );

    bcm2835_gpio_write( this->trigger, LOW );
}

void Ultrasonic::receive_ping( unsigned int timeout )
{
    unsigned long currTime = duration_cast<milliseconds>( system_clock::now().time_since_epoch() ).count();
    
    unsigned long maxtime = timeout + currTime;

    while ( bcm2835_gpio_lev( this->echo ) == LOW && currTime <= maxtime )
    {
        this->start = duration_cast<microseconds>( system_clock::now().time_since_epoch() ).count();
        currTime = duration_cast<milliseconds>( system_clock::now().time_since_epoch() ).count();
    }	

    currTime = duration_cast<milliseconds>( system_clock::now().time_since_epoch() ).count();
    maxtime = timeout + currTime;

    while ( bcm2835_gpio_lev( this->echo ) == HIGH && currTime <= maxtime )
    {
        this->stop = duration_cast<microseconds>( system_clock::now().time_since_epoch() ).count();
        currTime = duration_cast<milliseconds>( system_clock::now().time_since_epoch() ).count();
    }
    
}

float Ultrasonic::distance( unsigned int samplesAccuracy, unsigned int pause, unsigned int timeout )
{
	float samples[ samplesAccuracy ];

	for ( unsigned int i = 0; i < samplesAccuracy; i++ ) samples[ i ] = 0;

    for ( unsigned int i = 0; i < samplesAccuracy; i++ )
    {
        send_ping();
        receive_ping( timeout );

		bcm2835_delay( pause );

        long delta = ( this->stop - this->start ) * SONIC_SPEED;
        delta = delta / 2000.0f;

        float res = delta / 1000.0f;
        samples[ i ] = res;
    }

    sort( samples, samples + samplesAccuracy );

	float average = 0;
	float div = ( float ) samplesAccuracy;

	for ( unsigned int i = 0; i < samplesAccuracy; i++ ) 
	{
		if ( samples[ i ] > 2 ) average += samples[ i ];
		else div--;
	}

	return ( average / div ) - 1.0f;
}
