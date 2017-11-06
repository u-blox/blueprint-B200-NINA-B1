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
 
/*****************************************************************************
 * Includes 
 *****************************************************************************/
#include "led_srv.h"

#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "uuids.h"


/*****************************************************************************
 * Defines
 *****************************************************************************/
#define CHAR_DESCR_LED_RED      "LED Red"
#define CHAR_DESCR_LED_GREEN    "LED Green"
#define CHAR_DESCR_LED_BLUE     "LED Blue"
#define CHAR_DESCR_LED_RGB      "RGB LED"

/*****************************************************************************
 * Typedefs
 *****************************************************************************/

/*****************************************************************************
 * Data
 *****************************************************************************/

/***************************************************************************** 
 * Private method definitions
 *****************************************************************************/
/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bas       LED service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(led_service_t * p_srv, ble_evt_t * p_ble_evt)
{
    p_srv->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bas       LED service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(led_service_t * p_srv, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_srv->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_srv       LED Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(led_service_t * p_srv, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_srv->evt_handler != NULL) {
        if (p_evt_write->handle == p_srv->red_led.value_handle) {
            led_service_evt_t evt;

            evt.evt_type = LED_SRV_EVT_LED_CHANGE;
            evt.p.led_change_params.led = LED_RED;
            evt.p.led_change_params.lit = (p_evt_write->data[0] == 1);

            p_srv->evt_handler(p_srv, &evt);
        } else if (p_evt_write->handle == p_srv->green_led.value_handle) {
            led_service_evt_t evt;

            evt.evt_type = LED_SRV_EVT_LED_CHANGE;
            evt.p.led_change_params.led = LED_GREEN;
            evt.p.led_change_params.lit = (p_evt_write->data[0] == 1);

            p_srv->evt_handler(p_srv, &evt);
        } else if (p_evt_write->handle == p_srv->blue_led.value_handle) {
            led_service_evt_t evt;

            evt.evt_type = LED_SRV_EVT_LED_CHANGE;
            evt.p.led_change_params.led = LED_BLUE;
            evt.p.led_change_params.lit = (p_evt_write->data[0] == 1);

            p_srv->evt_handler(p_srv, &evt);
        } else if (p_evt_write->handle == p_srv->rgb_led.value_handle) {
            led_service_evt_t evt;

            evt.evt_type = LED_SRV_EVT_COLOR_CHANGE;
            evt.p.color.r = p_evt_write->data[0];
            evt.p.color.g = p_evt_write->data[1];
            evt.p.color.b = p_evt_write->data[2];
            p_srv->evt_handler(p_srv, &evt);
        }
    }
}

static uint32_t add_led_char(led_service_t * p_srv, const led_service_init_t * p_srv_init,
                              uint16_t uuid, ble_gatts_char_handles_t *p_char_handle,
                              uint8_t init_val, const char *p_descr) {

    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = (uint8_t*)p_descr;
    char_md.char_user_desc_max_size = strlen(p_descr);
    char_md.char_user_desc_size = strlen(p_descr);
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, uuid);

    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.read_perm  = p_srv_init->char_attr_md.read_perm;
    attr_md.write_perm = p_srv_init->char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;
    attr_char_value.p_value   = &init_val;

    err_code = sd_ble_gatts_characteristic_add(p_srv->service_handle, &char_md,
                                               &attr_char_value,
                                               p_char_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

static uint32_t add_rgb_char(led_service_t * p_srv, const led_service_init_t * p_srv_init) {

    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = (uint8_t*)CHAR_DESCR_LED_RGB;
    char_md.char_user_desc_max_size = strlen(CHAR_DESCR_LED_RGB);
    char_md.char_user_desc_size = strlen(CHAR_DESCR_LED_RGB);
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, LED_SRC_UUID_RGB_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.read_perm  = p_srv_init->char_attr_md.read_perm;
    attr_md.write_perm = p_srv_init->char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 4;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 4;
    attr_char_value.p_value   = 0;

    err_code = sd_ble_gatts_characteristic_add(p_srv->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_srv->rgb_led);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the LED characteristic.
 *
 * @param[in]   p_bas        LED Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t led_add_characteristics(led_service_t * p_srv, const led_service_init_t * p_srv_init)
{
    uint32_t err_code;

    err_code = add_led_char(p_srv, p_srv_init, LED_SRC_UUID_RED_CHAR, &p_srv->red_led, p_srv_init->red_init_val, CHAR_DESCR_LED_RED);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = add_led_char(p_srv, p_srv_init, LED_SRC_UUID_GREEN_CHAR, &p_srv->green_led, p_srv_init->green_init_val, CHAR_DESCR_LED_GREEN);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = add_led_char(p_srv, p_srv_init, LED_SRC_UUID_BLUE_CHAR, &p_srv->blue_led, p_srv_init->blue_init_val, CHAR_DESCR_LED_BLUE);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = add_rgb_char(p_srv, p_srv_init);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return NRF_SUCCESS;
}

/*****************************************************************************
 * Public Methods
 *****************************************************************************/
void led_srv_on_ble_evt(led_service_t * p_srv, ble_evt_t * p_ble_evt)
{
    if (p_srv == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_srv, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_srv, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_srv, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t led_srv_init(led_service_t * p_srv, const led_service_init_t * p_srv_init)
{
    if (p_srv == NULL || p_srv_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_srv->evt_handler               = p_srv_init->evt_handler;
    p_srv->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, LED_SRV_UUID);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_srv->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return led_add_characteristics(p_srv, p_srv_init);
}
