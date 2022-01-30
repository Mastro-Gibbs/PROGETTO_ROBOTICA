#include "rover.h"
#include "ultrasonic.h"
#include "buzzer.h"
#include "infrared.h"
#include "servo.h"
#include "roverhead.h"
#include "adc.h"
#include "led.h"
#include <unistd.h>
#include "controller.h"
#include <stdio.h>
#include <iostream>
#include <signal.h>

using namespace std;

void testMotor();
void testServo();
void testServoWrapper();
void testLed();
void testADC();
void testIR();
void init();
void close();

bool condition = true;

void bye( int signum )
{
	condition = false;
	cout << "CLOSING LIB" << ", SIGINT COMES" << endl;
}

int main()
{
	init();
	
	signal( SIGINT, bye );

	Ultrasonic right( 5, 16 );
	Ultrasonic left( 6, 26 );
	Ultrasonic front( 22, 27 );

	
	while ( condition )
	{
		cout << "FRONT: " << front.distance( 5, 20, 20 ) << endl;
		// cout << "LEFT: " << left.distance( 5, 20 ) << endl;
		//cout << "RIGHT: " << right.distance( 5, 20 ) << endl;

		cout << "---------------------------" << endl;
		bcm2835_delay(200);
	}

    close();
}



void init()
{
    bcm2835_init();
}

void close()
{
    bcm2835_close();
}

void testLed()
{
    Led l;
    int i = 2;

    while ( i )
    {

        for ( uint8_t r = 0; r < 255; r++ )
        {
            l.setLed( LED1, r, 0, 0 );
            l.setLed( LED2, r, 0, 0 );
            l.setLed( LED3, r, 0, 0 );
            l.setLed( LED4, r, 0, 0 );
            l.setLed( LED5, r, 0, 0 );
            l.setLed( LED6, r, 0, 0 );
            l.setLed( LED7, r, 0, 0 );
            l.setLed( LED8, r, 0, 0 );
            usleep( 10000 );
        }

        for ( uint8_t r = 255, g = 0, b = 0; ; )
        {
            if ( b == 255 ) break;

            l.setLed( LED1, r, g, b );
            l.setLed( LED2, r, g, b );
            l.setLed( LED3, r, g, b );
            l.setLed( LED4, r, g, b );
            l.setLed( LED5, r, g, b );
            l.setLed( LED6, r, g, b );
            l.setLed( LED7, r, g, b );
            l.setLed( LED8, r, g, b );

            if ( r ) r--;
            else if ( g != 255 ) g++;
            else g--;

            if ( !g ) b++;


            usleep( 10000 );
        }

        for ( uint8_t b = 254; b > 0; b-- )
        {
            l.setLed( LED1, 0, 0, b );
            l.setLed( LED2, 0, 0, b );
            l.setLed( LED3, 0, 0, b );
            l.setLed( LED4, 0, 0, b );
            l.setLed( LED5, 0, 0, b );
            l.setLed( LED6, 0, 0, b );
            l.setLed( LED7, 0, 0, b );
            l.setLed( LED8, 0, 0, b );
            usleep( 10000 );
        }

        l.switchOff( LED1 );
        l.switchOff( LED2 );
        l.switchOff( LED3 );
        l.switchOff( LED4 );
        l.switchOff( LED5 );
        l.switchOff( LED6 );
        l.switchOff( LED7 );
        l.switchOff( LED8 );
        i--;
        l.setBrightness( 100 );
    }
}

void testADC()
{
    Adc adc;
    printf( "SX IDR: %f\n", adc.recvADC( LEFT_IDR ) );
    printf( "DX IDR: %f\n", adc.recvADC( RIGHT_IDR ) );
    printf( "POWER: %f\n", adc.recvADC( POWER ) );

}

void testIR()
{
    Infrared ir;

    int i = 30;
    while ( i )
    {
        printf("SX: %d\n", ir.echoSX());
        printf("CX: %d\n", ir.echoCX());
        printf("DX: %d\n", ir.echoDX());
        printf("-----------------------\n");
        sleep( 1 );
        i--;
    }
}


void testServoWrapper()
{
    RoverHead rh;

    sleep(1);

    rh.lookLeft();

    sleep(1);

    rh.lookRight();

    sleep(1);

    rh.lookahead();

    sleep(1);

    rh.lookUp();

    sleep(1);

    rh.lookDown();

    sleep(1);

    rh.lookahead();
}


void testServo()
{
    ServoMotor sm;

    int i = 45;

    while ( i != 135 )
    {
        sm.setServo( SERVO0, i );
        i++;
        usleep( 5000 );
    }

    while ( i != 45 )
    {
        sm.setServo( SERVO0, i );
        i--;
        usleep( 5000 );
    }

    i = 80;
    while ( i != 135 )
    {
        sm.setServo( SERVO1, i );
        i++;
        usleep( 5000 );
    }

    while ( i != 80 )
    {
        sm.setServo( SERVO1, i );
        i--;
        usleep( 5000 );
    }

}


void testMotor()
{
    Rover r;

    r.forward( 1000 );

    sleep( 1 );

    r.backward( 1000 );

    sleep( 1 );

    r.spinLeft( 3000 );

    sleep( 1 );

    r.spinRight( 3000 );

    sleep( 1 );

    r.turnLeft( 2000, 200 );

    sleep( 1 );

    r.turnRight( 2000, 200 );

    sleep( 1 );

    r.halt();
}
