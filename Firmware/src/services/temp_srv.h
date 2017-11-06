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
 
#ifndef TEMP_SRV_H_
#define TEMP_SRV_H_


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
typedef struct temp_service_s temp_service_t;

/**@brief Temperature Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    bool                          support_notification;           /**< TRUE if notification of temperature changes is supported. */
    ble_srv_cccd_security_mode_t  char_attr_md;                   /**< Initial security level for characteristics attribute */
    ble_gap_conn_sec_mode_t       report_read_perm;               /**< Initial security level for report read attribute */
    int8_t                        temp_init_val;                  /**< Initial temperature value of characteristic */
} temp_service_init_t;

/*****************************************************************************
 * Data
 *****************************************************************************/
/**@brief Temperature Service structure. This contains various status information for the service. */
struct temp_service_s
{
    uint16_t                      service_handle;                 /**< Handle of Temperature Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      temp;                           /**< Temperature characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
};

/*****************************************************************************
 * Public Methods
 *****************************************************************************/
/**@brief Function for initializing the Temperature Service.
 *
 * @param[out]  p_srv       Temperature Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_srv_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t temp_srv_init(temp_service_t * p_srv, const temp_service_init_t * p_srv_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Temperature Service.
 *
 * @param[in]   p_srv      Temperature Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void temp_srv_on_ble_evt(temp_service_t * p_srv, ble_evt_t * p_ble_evt);

/**@brief Function for updating the Temperature characteristics, i.e. the state of the two LEDs
 *
 * @param[in]   p_srv       Temperature Service structure.
 * @param[in]   temp        New temperature to update characteristic with
 */
uint32_t temp_srv_notify_change(temp_service_t *p_srv, int8_t new_temp);

#endif /* TEMP_SRV_H_ */
