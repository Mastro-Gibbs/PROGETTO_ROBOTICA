#include "PCA9585.h"

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <sys/ioctl.h>


using namespace std;

#define SLEEP 5


PCA9585::PCA9585( int bus, uint8_t slaveAddr )
{
    this->MODE1        = 0x00;
    this->SUBADR1      = 0x02;
    this->SUBADR2      = 0x03;
    this->SUBADR3      = 0x04;
    this->PRESCALE     = 0xFE;
    this->LED0_ON_L    = 0x06;
    this->LED0_ON_H    = 0x07;
    this->LED0_OFF_L   = 0x08;
    this->LED0_OFF_H   = 0x09;
    this->ALLLED_ON_L  = 0xFA;
    this->ALLLED_ON_H  = 0xFB;
    this->ALLLED_OFF_L = 0xFC;
    this->ALLLED_OFF_H = 0xFD;

    char filepath[ 20 ];

    snprintf( filepath, 19, "/dev/i2c-%d", bus );

    this->fd = open( filepath, O_RDWR );

    setSlave( slaveAddr );

    writeI2C( this->MODE1, 0x00 );
}

PCA9585::~PCA9585()
{
    close( this->fd );
}


// PUBLIC METHODS
void PCA9585::setMotor( uint8_t reg, uint32_t duty )
{
    setPWM( reg, duty, 0 );
}

void PCA9585::setPWMFrequency( uint8_t frequency )
{
    float prescaleval = 25000000.0f;
    prescaleval /= 4096.0f;
    prescaleval /= ( float ) frequency;
    prescaleval -= 1.0f;

    float prescale = floor( prescaleval + 0.5f );

    int32_t omode = readI2C( this->MODE1 );
    uint8_t nmode = ( omode & 0x7F ) | 0x10;

    writeI2C( this->MODE1, nmode );
    writeI2C( this->PRESCALE, ( int ) floor( prescale ) );
    writeI2C( this->MODE1, omode );

    usleep( SLEEP );

    writeI2C( this->MODE1, ( omode | 0x80 ) );
}


void PCA9585::setServoMotor( uint8_t reg, uint32_t pulse )
{
    pulse = ( pulse * 4096 ) / 20000;
    setPWM( reg, pulse, 0 );
}



// PRIVATE METHODS

void PCA9585::writeI2C( uint8_t reg, uint8_t val )
{
    setSlave( this->slaveAddr );

    i2c_smbus_write_byte_data( this->fd, reg, val );
}

int32_t PCA9585::readI2C( uint8_t reg )
{
    setSlave( this->slaveAddr );

    int32_t s = i2c_smbus_read_byte_data( this->fd, reg );
    return s;
}

void PCA9585::setSlave( uint8_t addr )
{
    if ( addr != this->slaveAddr )
    {
        this->slaveAddr = addr;

        ioctl( this->fd, I2C_SLAVE, this->slaveAddr );
    }
}

void PCA9585::setPWM( uint8_t reg, uint32_t duty, uint8_t base )
{
    writeI2C( this->LED0_ON_L + 4 * reg, base & 0xFF );
    writeI2C( this->LED0_ON_H + 4 * reg, base >> 8 );
    writeI2C( this->LED0_OFF_L + 4 * reg, duty & 0xFF );
    writeI2C( this->LED0_OFF_H + 4 * reg, duty >> 8 );
}

