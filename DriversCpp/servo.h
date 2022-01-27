#ifndef SERVO_H
#define SERVO_H

#include "PCA9585.h"

#define SERVO0 0x8
#define SERVO1 0x9

#define SERVO0_START_POS 1436
#define SERVO1_START_POS 1652

class ServoMotor
{
    public:
        ServoMotor();
        ~ServoMotor();

        void setServo( uint8_t servo, uint16_t angle, uint8_t error = 10 );

    private:
        PCA9585 *pca = nullptr;
};


#endif // SERVO_H
