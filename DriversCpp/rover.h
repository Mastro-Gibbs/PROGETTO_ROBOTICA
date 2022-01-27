#ifndef ROVER_H
#define ROVER_H

#include "motor.h"

#define MIN_SPEED 300

#define S0_MAX 135
#define S0_MIN 45

#define S1_MAX 135
#define S1_MIN 65

class Rover
{
    public:
        Rover();
        ~Rover();

        void halt();

        void forward( uint32_t speed );

        void backward( uint32_t speed );

        void turnLeft( int32_t speedU, int32_t speedL );

        void turnRight( int32_t speedU, int32_t speedL );

        void spinLeft( uint32_t speed );

        void spinRight( uint32_t speed );

        void towerMove( uint8_t servo, uint16_t angle );

    private:
        Motor *motor = nullptr;
};

#endif // ROVER_H
