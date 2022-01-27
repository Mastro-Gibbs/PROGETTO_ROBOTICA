#include "servo.h"


ServoMotor::ServoMotor()
{
    this->pca = new PCA9585( 1, 0x40 );
    this->pca->setPWMFrequency( 50 );
    this->pca->setServoMotor( SERVO0, SERVO0_START_POS );
    this->pca->setServoMotor( SERVO1, SERVO1_START_POS );
}

ServoMotor::~ServoMotor()
{
    delete this->pca;
}

void ServoMotor::setServo( uint8_t servo, uint16_t angle, uint8_t error )
{
    uint16_t max = 0;
    angle = ( angle + error ) / 0.095;

    if ( servo == SERVO0 )
    {
        max = 2488 - angle;
        this->pca->setServoMotor( SERVO0, max );
    }
    else if ( servo == SERVO1 )
    {
        max = 600 + angle;
        this->pca->setServoMotor( SERVO1, max );
    }
}
