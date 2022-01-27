#ifndef PCA9585_H
#define PCA9585_H

extern "C"
{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <inttypes.h>

using namespace std;

class PCA9585
{
    public:
        PCA9585( int bus, uint8_t slaveAddr );
        ~PCA9585();

        void setMotor( uint8_t reg, uint32_t duty );

        void setPWMFrequency( uint8_t frequency );

        void setServoMotor( uint8_t reg, uint32_t pulse );

    private:
        void setSlave( uint8_t addr );

        void writeI2C( uint8_t reg, uint8_t val );

        int32_t readI2C( uint8_t reg );

        void setPWM( uint8_t reg, uint32_t duty, uint8_t base = 0 );

        int fd;

        uint8_t MODE1;
        uint8_t SUBADR1;
        uint8_t SUBADR2;
        uint8_t SUBADR3;
        uint8_t PRESCALE;
        uint8_t LED0_ON_L;
        uint8_t LED0_ON_H;
        uint8_t LED0_OFF_L;
        uint8_t LED0_OFF_H;
        uint8_t ALLLED_ON_L;
        uint8_t ALLLED_ON_H;
        uint8_t ALLLED_OFF_L;
        uint8_t ALLLED_OFF_H;

        uint8_t slaveAddr;
        uint32_t funcs;
};


#endif // GATTO_H
