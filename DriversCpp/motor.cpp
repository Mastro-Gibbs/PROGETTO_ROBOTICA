#include "motor.h"

#include <cmath>

Motor::Motor()
{
    this->pca = new PCA9585( 1, 0x40 );
    this->pca->setPWMFrequency( 50 );
    this->pca->setServoMotor( SERVO0, SERVO0_START_POS );
    this->pca->setServoMotor( SERVO1, SERVO1_START_POS );
}

Motor::~Motor()
{
    delete this->pca;
}


// PUBLIC METHODS

void Motor::setSingleModel( uint8_t motor, int32_t duty )
{
    duty = justifyOne( duty );

    if ( motor == LU )
        setLU( duty );
    else if ( motor == RU )
        setRU( duty );
    else if ( motor == LL )
        setLL( duty );
    else if ( motor == RL )
        setRL( duty );
}

void Motor::setModel( int32_t duty1, int32_t duty2, int32_t duty3, int32_t duty4 )
{
    int32_t* duties = justify( duty1, duty2, duty3, duty4 );

    int32_t* ptrCopy;
    ptrCopy = duties;

    this->model[ 0 ] = *duties++;
    this->model[ 1 ] = *duties++;
    this->model[ 2 ] = *duties++;
    this->model[ 3 ] = *duties;

    delete [] ptrCopy;
    duties = nullptr;
    
	setLU( this->model[ 0 ] );
    setRU( this->model[ 1 ] );
    setLL( this->model[ 2 ] );
    setRL( this->model[ 3 ] ); 
    
}


void Motor::noModel()
{
    for ( uint8_t i = 0; i < 3; i++ )
    {
        this->model[ 0 ] /= 2;
        this->model[ 1 ] /= 2;
        this->model[ 2 ] /= 2;
        this->model[ 3 ] /= 2;

	setModel( this->model[ 0 ], this->model[ 1 ], this->model[ 2 ], this->model[ 3 ] );
        
        usleep( 20000 );
    }

    setModel( 0, 0, 0, 0 );
}


int32_t* Motor::justify( int32_t duty1, int32_t duty2, int32_t duty3, int32_t duty4 )
{
    int32_t *res = new int32_t[ 4 ];

    if ( duty1 > 4095 )
        duty1 = 4095;
    else if ( duty1 < -4095 )
        duty1 = -4095;

    if ( duty2 > 4095 )
        duty2 = 4095;
    else if ( duty2 < -4095 )
        duty2 = -4095;

    if ( duty3 > 4095 )
        duty3 = 4095;
    else if ( duty3 < -4095 )
        duty3 = -4095;

    if ( duty4 > 4095 )
        duty4 = 4095;
    else if ( duty4 < -4095 )
        duty4 = -4095;

    res[ 0 ] = duty1;
    res[ 1 ] = duty2;
    res[ 2 ] = duty3;
    res[ 3 ] = duty4;

    return res;
}

int32_t Motor::justifyOne( int32_t duty )
{
    if ( duty > 4095 )
        duty = 4095;
    else if ( duty < -4095 )
        duty = -4095;

    return duty;
}

void Motor::setServo( uint8_t servo, uint16_t angle, uint8_t error )
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

// PRIVATE METHODS

void Motor::setLU( int32_t duty )
{
    if ( duty > 0 )
    {
        this->pca->setMotor( MOTOR_L_U0, duty );
        this->pca->setMotor( MOTOR_L_U1, 0 );
    }
    else if ( duty < 0 )
    {
        this->pca->setMotor( MOTOR_L_U1, abs( duty ) );
        this->pca->setMotor( MOTOR_L_U0, 0 );
    }
    else
    {
        this->pca->setMotor( MOTOR_L_U1, 4095 );
        this->pca->setMotor( MOTOR_L_U0, 4095 );
    }
}


void Motor::setRU( int32_t duty )
{
    if ( duty > 0 )
    {
        this->pca->setMotor( MOTOR_R_U0, duty );
        this->pca->setMotor( MOTOR_R_U1, 0 );
    }
    else if ( duty < 0 )
    {
        this->pca->setMotor( MOTOR_R_U1, abs( duty ) );
        this->pca->setMotor( MOTOR_R_U0, 0 );
    }
    else
    {
        this->pca->setMotor( MOTOR_R_U1, 4095 );
        this->pca->setMotor( MOTOR_R_U0, 4095 );
    }
}

void Motor::setLL( int32_t duty )
{
    if ( duty > 0 )
    {
        this->pca->setMotor( MOTOR_L_L0, duty );
        this->pca->setMotor( MOTOR_L_L1, 0 );
    }
    else if ( duty < 0 )
    {
        this->pca->setMotor( MOTOR_L_L1, abs( duty ) );
        this->pca->setMotor( MOTOR_L_L0, 0 );
    }
    else
    {
        this->pca->setMotor( MOTOR_L_L1, 4095 );
        this->pca->setMotor( MOTOR_L_L0, 4095 );
    }
}

void Motor::setRL( int32_t duty )
{
    if ( duty > 0 )
    {
        this->pca->setMotor( MOTOR_R_L0, duty );
        this->pca->setMotor( MOTOR_R_L1, 0 );
    }
    else if ( duty < 0 )
    {
        this->pca->setMotor( MOTOR_R_L1, abs( duty ) );
        this->pca->setMotor( MOTOR_R_L0, 0 );
    }
    else
    {
        this->pca->setMotor( MOTOR_R_L1, 4095 );
        this->pca->setMotor( MOTOR_R_L0, 4095 );
    }
}
