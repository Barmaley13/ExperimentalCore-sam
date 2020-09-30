/*
Copyright (c) 2011 Arduino LLC.  All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Original version:
// https://github.com/aethaniel/ExperimentalCore-sam/blob/master/module/cores/arduino/port_sam/core_digital.c

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_gpio.h"


void pinMode(uint32_t ulPin, PinMode ulMode)
{
    /* Handle non-pin index */
    if ( ulPin >= PINS_COUNT )
    {
        return;
    }
    
    uint8_t gpio = PinMap[ulPin].ulPin;
    enum gpio_direction direction;
    enum gpio_pull_mode pull_mode;

    switch (ulMode) 
    {
        case INPUT_OFF:
            direction = GPIO_DIRECTION_OFF; 
            break;
        case OUTPUT: case OUTPUT_PULLUP: case OUTPUT_PULLDOWN:
            direction = GPIO_DIRECTION_OUT; 
            break;
        default:
            direction = GPIO_DIRECTION_IN;
    }

    switch (ulMode)
    {
        case INPUT_PULLUP: case INPUT_OPENDRAIN_PULLUP: case OUTPUT_PULLUP:
            pull_mode = GPIO_PULL_UP;
            break;
        case INPUT_PULLDOWN: case OUTPUT_PULLDOWN:
            pull_mode = GPIO_PULL_DOWN;
            break;
        default:
            pull_mode = GPIO_PULL_OFF;
    }

    gpio_set_pin_direction(gpio, direction);
    gpio_set_pin_pull_mode(gpio, pull_mode);
    gpio_set_pin_function(gpio, GPIO_PIN_FUNCTION_OFF);

}

void digitalWrite( uint32_t ulPin, uint32_t ulVal )
{
    /* Handle non-pin index */
    if ( ulPin >= PINS_COUNT )
    {
        return ;
    }
    
    uint8_t gpio = PinMap[ulPin].ulPin;
    bool value = (ulVal == LOW) ? false : true;
    gpio_set_pin_level(gpio, value);
}

int digitalRead( uint32_t ulPin )
{
    /* Handle non-pin index */
    if ( ulPin >= PINS_COUNT )
    {
        return LOW ;
    }
    
    uint8_t gpio = PinMap[ulPin].ulPin;
    bool value = gpio_get_pin_level(gpio);

    return (value == false) ? LOW : HIGH;
}

#ifdef __cplusplus
}
#endif
