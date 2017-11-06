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
 
#ifndef BATTERY_H_
#define BATTERY_H_


/*****************************************************************************
 * Includes 
 *****************************************************************************/
#include <stdint.h>

/*****************************************************************************
 * Defines
 *****************************************************************************/


/*****************************************************************************
 * Typedefs
 *****************************************************************************/


/*****************************************************************************
 * Data
 *****************************************************************************/


/*****************************************************************************
 * Public Methods
 *****************************************************************************/
/**@brief Initiate the adc for battery sampling
 */
void battery_init();

/**@brief Sample battery and get battery estimate
 *
 * This function samples the battery voltage and returns an estimate in percent
 * when a new estimate is available. The function should be called periodically
 * and calculates. If the return value is less than 0, no new estimate is
 * available.
 *
 * @return      Greater than 0: Battery estimate in percent
 *              Less than 0: No new estimate is available
 */
int8_t battery_sample();

uint8_t battery_get_last_estimate();

#endif /* BATTERY_H_ */
