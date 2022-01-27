#include "adc.h"

Adc::Adc()
{
    char filepath[ 20 ];
    int bus = 1;

    snprintf( filepath, 19, "/dev/i2c-%d", bus );

    this->fd = open( filepath, O_RDWR );

    setSlave( I2C_ADDR );

    uint16_t i = 3;

    int32_t val = 0;

    while ( i )
    {
        val = readI2C_data( ADDR_TO_READ );
        if ( val < 150 )
            this->currBoard = BOARD_PCF;
        else
            this->currBoard = BOARD_ADS;
        i--;
    }
}

Adc::~Adc()
{
    close( this->fd );
}


float Adc::recvADC( uint8_t channel )
{
    if ( channel != LEFT_IDR && channel != RIGHT_IDR && channel != POWER )
        return -1.0f;

    float val = 0.0f;

    if ( this->currBoard == BOARD_PCF )
        val = analogPCF_read( channel );
    else if ( this->currBoard == BOARD_ADS )
        val = readADS( channel );

    if ( channel == POWER )
        val = val * 3;

    return val;
}



float Adc::analogPCF_read( uint8_t channel )
{
    uint16_t arrayPtr = 0;

    int32_t vals[ 9 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    while ( arrayPtr != 9 )
    {
        vals[ arrayPtr ] = readI2C_data( PCF8591 + channel );
        arrayPtr++;
    }

    std::sort( vals, vals + 9 );

    float volt = 0;
    volt = ( vals[ 4 ] / 256.0f ) * 3.3f;

    return std::ceil( volt * 100.0f ) / 100.0f;
}


float Adc::readADS( uint8_t channel )
{
    uint8_t cmd = 0x00;
    cmd = ADS7830 | ( ( ( ( channel << 2 ) | ( channel >> 1 ) ) & 0x07 ) << 4 );

    writeI2C( cmd );

    int32_t val = readI2C();

    float volt = 0;
    volt = ( val / 256.0f ) * 3.3f;

    return std::ceil( volt * 100.0f ) / 100.0f;
}


void Adc::setSlave( uint8_t addr )
{
    if ( addr != this->slaveAddr )
    {
        this->slaveAddr = addr;

        ioctl( this->fd, I2C_SLAVE, this->slaveAddr );
    }
}


void Adc::writeI2C_data( uint8_t reg, uint8_t val )
{
    setSlave( this->slaveAddr );

    i2c_smbus_write_byte_data( this->fd, reg, val );
}


int32_t Adc::readI2C_data( uint8_t reg )
{
    setSlave( this->slaveAddr );

    int32_t s = i2c_smbus_read_byte_data( this->fd, reg );
    return s;
}


void Adc::writeI2C( uint8_t val )
{
    setSlave( this->slaveAddr );

    i2c_smbus_write_byte( this->fd, val );
}


int32_t Adc::readI2C()
{
    setSlave( this->slaveAddr );

    int32_t s = i2c_smbus_read_byte( this->fd );
    return s;
}


