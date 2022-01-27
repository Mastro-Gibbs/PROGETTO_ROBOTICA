#ifndef __ADC_H
#define __ADC_H

#include <unistd.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <inttypes.h>

extern "C"
{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#define PCF8591 0x40
#define ADS7830 0x84

#define ADDR_TO_READ 0xF4
#define I2C_ADDR     0x48

#define BOARD_ADS 0x01
#define BOARD_PCF 0x02

#define LEFT_IDR  0x0
#define RIGHT_IDR 0x1
#define POWER     0x2


class Adc
{
    public:
        Adc();
        ~Adc();

        float recvADC( uint8_t channel );

    private:
        float analogPCF_read( uint8_t channel );
        float readADS( uint8_t channel );

        void setSlave( uint8_t addr );

        void    writeI2C_data( uint8_t reg, uint8_t val );
        int32_t readI2C_data( uint8_t reg );

        void    writeI2C( uint8_t val );
        int32_t readI2C();


        uint8_t slaveAddr;

        uint8_t currBoard;

        int fd;

};


#endif // __ADC_H
