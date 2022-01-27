#ifndef MOTOR_H
#define MOTOR_H

#define MOTOR_L_U0 0
#define MOTOR_L_U1 1

#define MOTOR_R_U0 6
#define MOTOR_R_U1 7

#define MOTOR_L_L0 3
#define MOTOR_L_L1 2

#define MOTOR_R_L0 4
#define MOTOR_R_L1 5

#define LU 0x00
#define RU 0x01
#define LL 0x02
#define RL 0x03

#define SERVO0 0x8
#define SERVO1 0x9

#define SERVO0_START_POS 1436
#define SERVO1_START_POS 1652

#include <inttypes.h>
#include <unistd.h>

#include "PCA9585.h"

class Motor
{
    public:
        Motor();
        ~Motor();

        void setSingleModel( uint8_t motor, int32_t duty );

        void setModel( int32_t duty1, int32_t duty2, int32_t duty3, int32_t duty4 );

        void noModel();

        int32_t* justify( int32_t duty1, int32_t duty2, int32_t duty3, int32_t duty4 );

        int32_t justifyOne( int32_t duty );

        void setServo( uint8_t servo, uint16_t angle, uint8_t error = 10 );

    private:
        void setLU( int32_t duty );

        void setRU( int32_t duty );

        void setLL( int32_t duty );

        void setRL( int32_t duty );

        PCA9585 *pca = nullptr;

	int32_t model[ 4 ];

};

#endif // MOTOR_H
