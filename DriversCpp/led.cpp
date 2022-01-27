#include "led.h"


Led::Led()
{
    ws2811_t leds =
    {
        .freq    = TARGET_FREQ,
        .dmanum  = DMA,
        .channel =
        {
            [ 0 ] =
            {
                .gpionum    = GPIO_PIN,
                .invert     = 0,
                .count      = LED_COUNT,
                .strip_type = STRIP_TYPE,
                .brightness = BRIGHTNESS,
            },
        },
    };

    this->strip = leds;

    ws2811_init( &this->strip );
}

Led::~Led()
{
    for ( uint8_t i = 0; i <= LED8; i++ )
        this->strip.channel[ 0 ].leds[ i ] = LED_OFF;

    ws2811_render( &this->strip );

    ws2811_fini( &this->strip );
}

void Led::setLed( uint8_t led, uint8_t r, uint8_t g, uint8_t b )
{
    if ( led > LED8  )
        return;

    uint32_t color = ( 0 << 24 ) + ( r << 16 ) + ( g << 8 ) + b;

    this->strip.channel[ 0 ].leds[ led ] = color;

    ws2811_render( &this->strip );
}

void Led::switchOff( uint8_t led )
{
    if ( led > LED8  )
        return;

    this->strip.channel[ 0 ].leds[ led ] = LED_OFF;

    ws2811_render( &this->strip );
}

void Led::setBrightness( uint8_t val )
{
    this->strip.channel[ 0 ].brightness = val;
}

