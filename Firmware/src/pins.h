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
 
#ifndef SRC_PINS_H_
#define SRC_PINS_H_


/*****************************************************************************
 * Includes 
 *****************************************************************************/


/*****************************************************************************
 * Defines
 *****************************************************************************/

#if defined(BOARD_PCA10040)
#define PIN_LED_RED         18
#define PIN_LED_GREEN       19
#define PIN_LED_BLUE        20
#define PIN_BUTTON_1        13
#define PIN_BUTTON_2        14
#elif defined(BOARD_NINA_B1)
#define PIN_LED_RED          8
#define PIN_LED_GREEN       16
#define PIN_LED_BLUE        18
#define PIN_BUTTON_1        30
#define PIN_BUTTON_2        16

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#endif

#define PIN_SENSOR_I2C_SCL  3
#define PIN_SENSOR_I2C_SDA  2
#define PIN_SENSOR_INT_1    28
#define PIN_SENSOR_INT_2    29

/*****************************************************************************
 * Typedefs
 *****************************************************************************/


/*****************************************************************************
 * Data
 *****************************************************************************/


/*****************************************************************************
 * Public Methods
 *****************************************************************************/

#endif /* SRC_PINS_H_ */
