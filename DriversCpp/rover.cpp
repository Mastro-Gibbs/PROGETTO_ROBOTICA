#include "rover.h"


Rover::Rover()
{
    this->motor = new Motor();
}


Rover::~Rover()
{
    delete this->motor;
}


void Rover::halt()
{
    this->motor->noModel();
}


void Rover::forward( uint32_t speed )
{
    this->motor->setModel( speed, speed, speed, speed );
}


void Rover::backward( uint32_t speed )
{
    this->motor->setModel( -speed, -speed, -speed, -speed );
}


void Rover::turnLeft( int32_t speedU, int32_t speedL )
{
    this->motor->setSingleModel( LU, speedU );
    this->motor->setSingleModel( LL, speedL );
}


void Rover::turnRight( int32_t speedU, int32_t speedL )
{
    this->motor->setSingleModel( RU, speedU );
    this->motor->setSingleModel( RL, speedL );
}


void Rover::spinLeft( uint32_t speed )
{
    this->motor->setModel( -speed, speed, -speed, speed );
}


void Rover::spinRight( uint32_t speed )
{
    this->motor->setModel( speed, -speed, speed, -speed );
}

void Rover::towerMove( uint8_t servo, uint16_t angle )
{
    if ( servo == SERVO0 )
    {
        if ( angle < S0_MIN )
            angle = S0_MIN;
        else if ( angle > S0_MAX )
            angle = S0_MAX;
    }
    else if ( servo == SERVO1 )
    {
        if ( angle < S1_MIN )
            angle = S1_MIN;
        else if ( angle > S1_MAX )
            angle = S1_MAX;
    }

    this->motor->setServo( servo, angle );
}
