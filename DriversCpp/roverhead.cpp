#include "roverhead.h"

RoverHead::RoverHead()
{
    this->sm = new ServoMotor();
}

RoverHead::~RoverHead()
{
    delete this->sm;
}

void RoverHead::lookahead()
{
    this->sm->setServo( SERVO0, 90 );
    this->sm->setServo( SERVO1, 90 );
}

void RoverHead::lookLeft()
{
    this->sm->setServo( SERVO0, 45 );
}

void RoverHead::lookRight()
{
    this->sm->setServo( SERVO0, 135 );
}

void RoverHead::lookUp()
{
    this->sm->setServo( SERVO1, 135 );
}

void RoverHead::lookDown()
{
    this->sm->setServo( SERVO1, 65 );
}

void RoverHead::towerMove( uint8_t servo, uint16_t angle )
{
    angle = justify( servo, angle );
    this->sm->setServo( servo, angle );
}


uint16_t RoverHead::justify( uint8_t servo, uint16_t angle )
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
    return angle;
}


