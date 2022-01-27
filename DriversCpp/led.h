#ifndef __LED__H__
#define __LED__H__

#include <ws2811/ws2811.h>
#include <inttypes.h>

#define TARGET_FREQ 800000
#define DMA         10
#define GPIO_PIN    18
#define LED_COUNT   8
#define STRIP_TYPE  WS2811_STRIP_GRB
#define BRIGHTNESS  255

#define LED1 0
#define LED2 1
#define LED3 2
#define LED4 3
#define LED5 4
#define LED6 5
#define LED7 6
#define LED8 7

#define LED_OFF 0x00000000

class Led
{
    public:
        Led();
        ~Led();

        void setLed( uint8_t led, uint8_t r, uint8_t g, uint8_t b );

        void switchOff( uint8_t led );

        void setBrightness( uint8_t val );

    private:
        ws2811_t strip;

};



#endif // __LED__H__
