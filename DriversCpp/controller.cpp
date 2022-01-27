#include "controller.h"
#include <iostream>

using namespace std;

Action::Action()
{
    this->lastAction = 0x00;
}


Controller::Controller()
{
    this->duty = 1500;
    this->ccounter = 0;

    this->angleS0 = 90;
    this->angleS1 = 90;

    this->act = Action();
}

Controller::~Controller()
{

}


void Controller::eventLoop()
{
    this->b.ffStart();
    usleep( 300000 );
    this->b.stop();

    Display * d = XOpenDisplay( NULL );
    Window win = XCreateSimpleWindow( d, RootWindow(d, 0), 1, 1, 200, 200, 0, BlackPixel(d, 0), BlackPixel(d, 0) );

    XSelectInput( d, win, KeyPressMask | KeyReleaseMask | ClientMessage );
    XMapWindow( d, win );
    XFlush( d );

    XEvent event;
    Atom closeMessage = XInternAtom( d, "WM_DELETE_WINDOW", True );
    XSetWMProtocols( d, win, &closeMessage, 1 );

    bool done = false;

	uint8_t sx, dx, cx, tmp;

    while( !done )
    {	
        XNextEvent(d, &event);
        switch(event.type)
        {
            case KeyPress:
                Controller::exec( 0x01, (uint8_t) XLookupKeysym( &event.xkey, 0 ) );
                break;
            case KeyRelease:
                if( was_it_auto_repeat( d, &event, KeyRelease, KeyPress ) )
                    XNextEvent( d, &event ); /* Consume the extra event so we can ignore it. */
                else
                    Controller::exec( 0x00, (uint8_t) XLookupKeysym( &event.xkey, 0 ) );
                break;
            case ClientMessage:
                if ( (Atom)event.xclient.data.l[ 0 ] == closeMessage )
                    done = true;
                break;
        }

		printf("--------------------------------\n");
		
		tmp = getIr();
		dx = ( tmp >> 0 ) & 1;
		cx = ( tmp >> 1 ) & 1;
		sx = ( tmp >> 2 ) & 1;
		
		printf( "SX = %d | CX = %d | DX = %d\n", sx, cx, dx);

		printf( "LUCE -----> %s\n", (( this->adc.recvADC( LEFT_IDR ) < 1.1f ) && ( this->adc.recvADC( RIGHT_IDR ) < 1.1f )) ? "BUIO" : "GIORNO" ); 
		printf( "CARICA ---> %.02f\n", this->adc.recvADC( POWER ) );
        printf( "DISTANCE -> %.02f\n", this->u.distance() );
    }
    XDestroyWindow( d, win );
    XCloseDisplay( d );
}


bool Controller::was_it_auto_repeat( Display * d, XEvent * e, int curr_t, int next_t )
{
    if( e->type == curr_t && XEventsQueued(d, QueuedAfterReading))
    {
        XEvent nev;
        XPeekEvent( d, &nev );
        return ( nev.type == next_t && nev.xkey.time == e->xkey.time && nev.xkey.keycode == e->xkey.keycode);
    }
    return false;
}


void Controller::exec( const uint8_t &mode, const uint8_t &c )
{
    if ( mode )
    {
        switch ( c )
        {
            case 'w':
                this->rover.forward( this->duty );
                //this->torward = FORWARD;
                break;
            case 's':
                this->rover.backward( this->duty );
                //this->torward = BACKWARD;
                break;
            case 'a':
                //if ( this->torward == FORWARD )
                    this->rover.spinLeft( this->duty );
                //else if ( this->torward == BACKWARD )
                  //  this->rover.turnLeft( 1000, 1000 );
                break;
            case 'd':
               // if ( this->torward == FORWARD )
                    this->rover.spinRight( this->duty );
                //else if ( this->torward == BACKWARD )
                  //  this->rover.turnRight( 1000, 1000 );
                break;
            case '-':
                this->duty -= 500;
                break;
            case '+':
                this->duty += 500;
                break;
            default:
                break;
        }
    }
    else
    {
        switch ( c )
        {
            case 'w':
                this->rover.halt();
                //this->torward = NOMOVE;
				cout << "eccoci\n";
                break;
            case 's':
                this->rover.halt();
                //this->torward = NOMOVE;
                break;
            case 'a':
                //if ( this->torward == FORWARD )
                    this->rover.halt();
                //else if ( this->torward == BACKWARD )
                  //  this->rover.backward( this->duty );
                break;
            case 'd':
                //if ( this->torward == FORWARD )
                    this->rover.halt();
                //else if ( this->torward == BACKWARD )
                  //  this->rover.backward( this->duty );
                break;
            default:
                break;
        }
    }
}

void Controller::repeat( uint8_t c )
{



}

uint8_t Controller::getIr()
{
	uint8_t res = 0x00;

	if ( this->ir.echoSX() == 1 )
		res = res | 1<<2;
	if ( this->ir.echoCX() == 1 )
		res = res | 1<<1;
	if ( this->ir.echoDX() == 1 )
		res = res | 1<<0;

	return res;
}
