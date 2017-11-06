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
#include "acc_srv.h"

#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "uuids.h"


/*****************************************************************************
 * Defines
 *****************************************************************************/
#define CHAR_DESCR_RANGE "Accel Range"
#define CHAR_DESCR_X "Accel X-Coordinate"
#define CHAR_DESCR_Y "Accel Y-Coordinate"
#define CHAR_DESCR_Z "Accel Z-Coordinate"
#define CHAR_DESCR_AXIS_AGGREGATE "Accel XYZ-Coordinates"

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
 * @param[in]   p_srv       Accelerometer service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(acc_service_t * p_srv, ble_evt_t * p_ble_evt)
{
    p_srv->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_srv       Accelerometer service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(acc_service_t * p_srv, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_srv->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static uint32_t add_range_char(acc_service_t * p_srv, const acc_service_init_t * p_srv_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_srv_init->char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read   = 1;
    char_md.p_char_user_desc  = (uint8_t*)CHAR_DESCR_RANGE;
    char_md.char_user_desc_max_size = strlen(CHAR_DESCR_RANGE);
    char_md.char_user_desc_size = strlen(CHAR_DESCR_RANGE);
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, ACC_SRV_UUID_RANGE_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.read_perm  = p_srv_init->char_attr_md.read_perm;
    attr_md.write_perm = p_srv_init->char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 1;
    attr_char_value.p_value   = (uint8_t*)&p_srv_init->initial_range;

    err_code = sd_ble_gatts_characteristic_add(p_srv->service_handle, &char_md,
                                               &attr_char_value, &p_srv->acc_range);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

static uint32_t add_axis_char(acc_service_t *p_srv, const acc_service_init_t *p_srv_init,
                              uint16_t uuid, ble_gatts_char_handles_t *p_char_handle, const char *p_descr)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_srv_init->char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
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
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 1;
    attr_char_value.p_value   = 0;

    err_code = sd_ble_gatts_characteristic_add(p_srv->service_handle, &char_md,
                                               &attr_char_value,
                                               p_char_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

static uint32_t add_axis_aggregate(acc_service_t * p_srv, const acc_service_init_t * p_srv_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_srv_init->char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = (uint8_t*)CHAR_DESCR_AXIS_AGGREGATE;
    char_md.char_user_desc_max_size = strlen(CHAR_DESCR_AXIS_AGGREGATE);
    char_md.char_user_desc_size = strlen(CHAR_DESCR_AXIS_AGGREGATE);
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, ACC_SRV_UUID_XYZ_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.read_perm  = p_srv_init->char_attr_md.read_perm;
    attr_md.write_perm = p_srv_init->char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 6;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 6;
    attr_char_value.p_value   = 0;

    err_code = sd_ble_gatts_characteristic_add(p_srv->service_handle, &char_md,
                                               &attr_char_value, &p_srv->acc_xyz);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the Accelerometer characteristic.
 *
 * @param[in]   p_srv        Accelerometer Service structure.
 * @param[in]   p_srv_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t acc_add_characteristics(acc_service_t * p_srv, const acc_service_init_t * p_srv_init)
{
    uint32_t err_code;

    err_code = add_range_char(p_srv, p_srv_init);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = add_axis_char(p_srv, p_srv_init, ACC_SRV_UUID_X_CHAR, &p_srv->acc_x, CHAR_DESCR_X);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = add_axis_char(p_srv, p_srv_init, ACC_SRV_UUID_Y_CHAR, &p_srv->acc_y, CHAR_DESCR_Y);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = add_axis_char(p_srv, p_srv_init, ACC_SRV_UUID_Z_CHAR, &p_srv->acc_z, CHAR_DESCR_Z);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = add_axis_aggregate(p_srv, p_srv_init);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return NRF_SUCCESS;
}

/*****************************************************************************
 * Public Methods
 *****************************************************************************/
void acc_srv_on_ble_evt(acc_service_t * p_srv, ble_evt_t * p_ble_evt)
{
    if (p_srv == NULL || p_ble_evt == NULL) {
        return;
    }

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_srv, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_srv, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t acc_srv_init(acc_service_t * p_srv, const acc_service_init_t * p_srv_init)
{
    if (p_srv == NULL || p_srv_init == NULL) {
        return NRF_ERROR_NULL;
    }
    
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_srv->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, ACC_SRV_UUID);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_srv->service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return acc_add_characteristics(p_srv, p_srv_init);
}

uint32_t acc_srv_notify_axis(acc_service_t * p_srv, int8_t val, ble_gatts_char_handles_t *p_axis_char)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    if (p_srv == NULL) {
    	return NRF_ERROR_INVALID_PARAM;
    }

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = 1;
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&val;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_srv->conn_handle,
                                      p_axis_char->value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Send value if connected and notifying.
    if (p_srv->conn_handle != BLE_CONN_HANDLE_INVALID) {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_axis_char->value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_srv->conn_handle, &hvx_params);
    } else {
        err_code = NRF_ERROR_INVALID_STATE;
	}

    return err_code;
}

uint32_t acc_srv_notify_axis_aggregate(acc_service_t * p_srv, int8_t *p_vals, uint32_t len)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    if (p_srv == NULL) {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_vals == NULL) {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (len != 6) {
        return NRF_ERROR_INVALID_PARAM;
    }

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = 6;
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)p_vals;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_srv->conn_handle,
                                      p_srv->acc_xyz.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Send value if connected and notifying.
    if (p_srv->conn_handle != BLE_CONN_HANDLE_INVALID) {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_srv->acc_xyz.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_srv->conn_handle, &hvx_params);
    } else {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}
