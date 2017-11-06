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
 
/**
 * BMI160, Accelerometer
 */
 
 #ifndef BMI160_H__
 #define BMI160_H__

//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

//@} End of Includes


//////////////////////////////////////////////////////////////////////
/// @name Defines
//@{

#define ACC_DATA_SIZE		(12)

//@} End of Defines


//////////////////////////////////////////////////////////////////////
/// @name Typedefs
//@{

typedef enum {
    SENSOR_EVENT_SENSOR_DATA,
    SENSOR_EVENT_TEMPERATURE,
    SENSOR_EVENT_NO_MOTION
} sensor_event_type_t;

typedef struct {
} sensor_data_t;

typedef struct {
    sensor_event_type_t evt;

    union {
        struct {
            uint32_t sensor_time;
            int16_t acc_x;
            int16_t acc_y;
            int16_t acc_z;
            int16_t gyro_x;
            int16_t gyro_y;
            int16_t gyro_z;
        } sensor_data;
        int16_t temperature;
    } p;
} sensor_event_t;

typedef void (*sensor_event_handler_cb_t)(sensor_event_t evt);

//@} End of Typedefs


//////////////////////////////////////////////////////////////////////
/// @name Data
//@{
//@} End of Data


//////////////////////////////////////////////////////////////////////
/// @name Private Methods
//@{
//@} End of Private Methods


//////////////////////////////////////////////////////////////////////
/// @name Public Methods
//@{

/**@brief Initialize the bmi160 chip
 *
 * @param[in]   handler				event handler
 */
void sensor_init(sensor_event_handler_cb_t handler);

/**@brief Read chip ID
 *
 * @return      The ID register content of the sensor
 */
uint8_t sensor_read_chip_id();

/**@brief Read sensor power mode
 *
 * This function executes blocking communication with the sensor
 *
 * @return      The content of the power mode register (PMU_STATUS)
 */
uint8_t sensor_read_mode();

/**@brief Request sensor data
 *
 * This function schedules a twi transition to fetch sensor data. The data will
 * be delivered through the sensor_event_handler_cb.
 *
 * @return      NRF_SUCCESS upon success
 */
uint32_t sensor_get_sensor_data();

/**@brief Read accelerometer sensor data
 *
 * This function reads accelerometer sensor data from the MEMS sensor. The
 * operation is blocking.
 *
 * @param[in]   p_x             pointer to X axis buffer
 * @param[in]   p_y             pointer to Y axis buffer
 * @param[in]   p_z             pointer to Z axis buffer
 *
 * @return      NRF_SUCCESS upon success
 */
uint32_t sensor_read_acc_sensor_data(int16_t *p_x, int16_t *p_y, int16_t *p_z);

/**@brief Read gyro sensor data
 *
 * This function reads gyro sensor data from the MEMS sensor. The
 * operation is blocking.
 *
 * @param[in]   p_x             pointer to X axis buffer
 * @param[in]   p_y             pointer to Y axis buffer
 * @param[in]   p_z             pointer to Z axis buffer
 *
 * @return      NRF_SUCCESS upon success
 */
uint32_t sensor_read_gyro_sensor_data(int16_t *p_x, int16_t *p_y, int16_t *p_z);

/**@brief Request sensor temperature
 *
 * This function schedules a twi transition to fetch the sensor temperature.
 * The temperature data will be delivered to the sensor_event_handler_cb.
 *
 * @return      NRF_SUCCESS upon success
 */
uint32_t sensor_get_temperature();

/**@brief Get accelerometer range
 *
 * This function executes blocking communication with the sensor
 *
 * @return      The accelerometer range in -+g, e.g. 2 means +-2g
 */
uint8_t sensor_get_acc_range();

/**@brief Set accelerometer range
 *
 * This function executes blocking communication with the sensor
 *
 * @param[in]   The range to be used in g, valid values are 2, 4, 8, 1nd 16
 * @return      NRF_SUCCESS upon success
 *              NRF_ERROR_INVALID_PARAM bad range parameter
 */
uint32_t sensor_set_range(uint8_t range);

/**@brief Enable motion detect. When motion is detected, the sensor_event_handler will
 * get a SENSOR_EVENT_MOTION_DETECTED event.
 */
void sensor_enable_motion_detect(void);

void sensors_set_normal_mode(void);

//@} End of Public Methods

#endif //BMI160_H__
