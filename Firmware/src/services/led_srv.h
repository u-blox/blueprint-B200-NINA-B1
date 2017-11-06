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
 
#ifndef LED_SRV_H_
#define LED_SRV_H_


/*****************************************************************************
 * Includes 
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "leds.h"

/*****************************************************************************
 * Defines
 *****************************************************************************/


/*****************************************************************************
 * Typedefs
 *****************************************************************************/
/**@brief LED Service event type. */
typedef enum
{
    LED_SRV_EVT_LED_CHANGE,                                        /**< LED status change */
    LED_SRV_EVT_COLOR_CHANGE,                                      /**< RGB Color changed */
} led_service_evt_type_t;

/**@brief LED Service event. */
typedef struct
{
    led_service_evt_type_t evt_type;                               /**< Type of event. */

    union {
        struct {
            led_t led;
            bool  lit;
        } led_change_params;
        struct {
            uint8_t r;
            uint8_t g;
            uint8_t b;
        } color;
    } p;
} led_service_evt_t;

// Forward declaration of the ble_bas_t type.
typedef struct led_service_s led_service_t;

/**@brief LED Service event handler type. */
typedef void (*led_service_evt_handler_t) (led_service_t * p_srv, led_service_evt_t * p_evt);

/**@brief LED Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    led_service_evt_handler_t     evt_handler;                    /**< Event handler to be called for handling events in the LED service. */
    bool                          support_notification;           /**< TRUE if notification of LED state changes is supported. */
    ble_srv_cccd_security_mode_t  char_attr_md;                   /**< Initial security level for characteristics attribute */
    ble_gap_conn_sec_mode_t       report_read_perm;               /**< Initial security level for report read attribute */
    uint8_t                       red_init_val;                   /**< Red led init value of characteristic */
    uint8_t                       green_init_val;                 /**< Green led init value of characteristic */
    uint8_t                       blue_init_val;                  /**< Green led init value of characteristic */
    uint8_t                       rgb_init_val;                   /**< Green led init value of characteristic */
} led_service_init_t;

/*****************************************************************************
 * Data
 *****************************************************************************/
/**@brief LED Service structure. This contains various status information for the service. */
struct led_service_s
{
    led_service_evt_handler_t     evt_handler;                    /**< Event handler to be called for handling events in the LED Service. */
    uint16_t                      service_handle;                 /**< Handle of LED Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      red_led;                        /**< Red LED characteristic. */
    ble_gatts_char_handles_t      green_led;                      /**< Green LED characteristic. */
    ble_gatts_char_handles_t      blue_led;                      /**< Green LED characteristic. */
    ble_gatts_char_handles_t      rgb_led;                        /**< RGB LED characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of LED states is supported. */
};

/*****************************************************************************
 * Public Methods
 *****************************************************************************/
/**@brief Function for initializing the LED Service.
 *
 * @param[out]  p_srv       LED Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_srv_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t led_srv_init(led_service_t * p_srv, const led_service_init_t * p_srv_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the LED Service.
 *
 * @param[in]   p_srv      LED Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void led_srv_on_ble_evt(led_service_t * p_srv, ble_evt_t * p_ble_evt);

/**@brief Function for updating the LED characteristics, i.e. the state of the two LEDs
 *
 * @param[in]   p_srv       LED Service structure.
 * @param[in]   led_on      LED status
 * @param[in]   p_led_char  Pointer to characteristic handle
 */
uint32_t led_srv_notify_led_change(led_service_t * p_srv, bool led_on, ble_gatts_char_handles_t *p_led_char);

#endif /* LED_SRV_H_ */
