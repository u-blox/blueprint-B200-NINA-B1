/*
 * Copyright (C) u-blox 
 * 
 * u-blox reserves all rights in this deliverable (documentation, software, etc.,
 * hereafter “Deliverable”). 
 * 
 * u-blox grants you the right to use, copy, modify and distribute the
 * Deliverable provided hereunder for any purpose without fee.
 * 
 * THIS DELIVERABLE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF THIS
 * DELIVERABLE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 * 
 * In case you provide us a feedback or make a contribution in the form of a
 * further development of the Deliverable (“Contribution”), u-blox will have the
 * same rights as granted to you, namely to use, copy, modify and distribute the
 * Contribution provided to us for any purpose without fee.
 */
 
#ifndef LEDS_H_
#define LEDS_H_


/*****************************************************************************
 * Includes 
 *****************************************************************************/
#include "pins.h"

#include <stdbool.h>
#include <stdint.h>

/*****************************************************************************
 * Defines
 *****************************************************************************/


/*****************************************************************************
 * Typedefs
 *****************************************************************************/
typedef enum
{
    LED_RED,
    LED_GREEN,
    LED_BLUE
} led_t;

/*****************************************************************************
 * Data
 *****************************************************************************/


/*****************************************************************************
 * Public Methods
*****************************************************************************/
/**@brief Initiate the LEDs
 *
 * Sets gpios as output and initial state off
 */
void leds_init(void);

/**@brief Set LED state
 *
 * @param[in] led   The LED to set
 * @param[in] lit   True = On, False = Off
 */
void leds_set_led(led_t led, bool lit);

/**@brief Get state of LED pin
 *
 * @param[in] led   The LED to get
 * @return          True = LED is lit, False = led is not lit
 */
bool leds_get(led_t led);

void leds_blink_advertising(void);
void leds_blink_connected(void);
void leds_blink_stop(void);
void leds_set_color(uint8_t red, uint8_t green, uint8_t blue);

#endif /* LEDS_H_ */
