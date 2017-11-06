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
 
#ifndef GYRO_SRV_H__
#define GYRO_SRV_H__

/*****************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/*****************************************************************************
 * Defines
 *****************************************************************************/


/*****************************************************************************
 * Typedefs
 *****************************************************************************/
typedef struct gyro_service_s gyro_service_t;

/**@brief Gyro Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_srv_cccd_security_mode_t  char_attr_md;                   /**< Initial security level for characteristics attribute */
    ble_gap_conn_sec_mode_t       report_read_perm;               /**< Initial security level for report read attribute */
} gyro_service_init_t;


/*****************************************************************************
 * Data
 *****************************************************************************/
/**@brief Gyro Service structure. This contains various status information for the service. */
struct gyro_service_s
{
    uint16_t                      service_handle;                 /**< Handle of Gyro Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      gyro_x;                         /**< Gyro X axis characteristic. */
    ble_gatts_char_handles_t      gyro_y;                         /**< Gyro Y axis characteristic. */
    ble_gatts_char_handles_t      gyro_z;                         /**< Gyro Z axis characteristic. */
    ble_gatts_char_handles_t      gyro_xyz;                       /**< Gyro axis aggregate characteristic */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
};


/*****************************************************************************
 * Public Methods
 *****************************************************************************/
/**@brief Function for initializing the Gyro Service.
 *
 * @param[out]  p_srv       Gyro Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_srv_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t gyro_srv_init(gyro_service_t * p_srv, const gyro_service_init_t * p_srv_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Gyro Service.
 *
 * @param[in]   p_srv      Gyro Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void gyro_srv_on_ble_evt(gyro_service_t * p_srv, ble_evt_t * p_ble_evt);

/**@brief Function for updating an axis, i.e. characteristic, in the Gyro Service
 *
 * @param[in]   p_srv       Gyro Service structure.
 * @param[in]   val         Gyro axis value
 * @param[in]   p_axis_char Pointer to characteristic handle
 */
uint32_t gyro_srv_notify_axis(gyro_service_t * p_srv, int8_t val, ble_gatts_char_handles_t *p_axis_char);

/**@brief Function for updating the axis aggregate characteristic
 *
 * @param[in]   p_srv       Gyro Service structure.
 * @param[in]   p_vals      Pointer to array of axis data
 * @param[in]   len         Length of array
 */
uint32_t gyro_srv_notify_axis_aggregate(gyro_service_t * p_srv, int8_t *p_vals, uint32_t len);

#endif // GYRO_SRV_H__

/** @} */
