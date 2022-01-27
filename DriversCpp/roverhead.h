#ifndef ROVERHEAD_H
#define ROVERHEAD_H

#include "servo.h"

#define S0_MAX 135
#define S0_MIN 45

#define S1_MAX 135
#define S1_MIN 65

class RoverHead
{
    public:
        RoverHead();
        ~RoverHead();

        void lookahead();

        void lookLeft();

        void lookRight();

        void lookUp();

        void lookDown();

        void towerMove( uint8_t servo, uint16_t angle );

    private:
        uint16_t justify( uint8_t servo, uint16_t angle );

        ServoMotor *sm = nullptr;

};


#endif // ROVERHEAD_H
