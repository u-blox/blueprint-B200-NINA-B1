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
 
#ifndef UUIDS_H__
#define UUIDS_H__

/**
 * Custom service UUIDs
 */
#define ACC_SRV_UUID    0xFFA0
#define GYRO_SRV_UUID   0xFFB0
#define TEMP_SRV_UUID   0xFFE0
#define LED_SRV_UUID    0xFFD0

/**
 * Accelerometer service characteristics
 */
#define ACC_SRV_UUID_ENABLER    0xFFA1
#define ACC_SRV_UUID_RANGE_CHAR 0xFFA2
#define ACC_SRV_UUID_X_CHAR     0xFFA3
#define ACC_SRV_UUID_Y_CHAR     0xFFA4
#define ACC_SRV_UUID_Z_CHAR     0xFFA5
#define ACC_SRV_UUID_XYZ_CHAR   0xFFA6

/**
 * Gyro service characteristics
 */
#define GYRO_SRV_UUID_ENABLER    0xFFB1
#define GYRO_SRV_UUID_RANGE_CHAR 0xFFB2
#define GYRO_SRV_UUID_X_CHAR     0xFFB3
#define GYRO_SRV_UUID_Y_CHAR     0xFFB4
#define GYRO_SRV_UUID_Z_CHAR     0xFFB5
#define GYRO_SRV_UUID_XYZ_CHAR   0xFFB6

/**
 * Temperature service characteristics
 */
#define TEMP_SRV_UUID_TEMP_CHAR 0xFFE1

/**
 * LED service characteristics
 */
#define LED_SRC_UUID_RED_CHAR   0xFFD1
#define LED_SRC_UUID_GREEN_CHAR 0xFFD2
#define LED_SRC_UUID_BLUE_CHAR  0xFFD3
#define LED_SRC_UUID_RGB_CHAR   0xFFD4

#endif // UUIDS_H__

/** @} */
