#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdlib.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>

#include "rover.h"
#include "buzzer.h"
#include "ultrasonic.h"
#include "infrared.h"
#include "adc.h"

#define NOMOVE   0x00
#define FORWARD  0x01
#define BACKWARD 0x02

using namespace std;


struct Action
{
    uint8_t lastAction;

    Action();
};


class Controller
{
    public:
        Controller();
        ~Controller();

        void eventLoop();

    private:
        bool was_it_auto_repeat( Display * d, XEvent * e, int curr_t, int next_t );

        void exec( const uint8_t &mode, const uint8_t &c );
		void repeat( uint8_t c );

		uint8_t getIr();

        Rover rover;
		Buzzer b;
		Ultrasonic u;
		Infrared ir;
		Adc adc;

        uint32_t duty;
        uint16_t  angleS1;
        uint16_t  angleS0;

        uint64_t ccounter;
        Action   act;

        uint8_t torward;
};


#endif // _CONTROLLER_H_
