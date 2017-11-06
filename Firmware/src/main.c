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
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "softdevice_handler_appsh.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "nrf_delay.h"
#include "app_scheduler.h"
#include "nrf_drv_gpiote.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "dfu_types.h"

#include "acc_srv.h"
#include "gyro_srv.h"
#include "led_srv.h"
#include "temp_srv.h"
#include "uuids.h"

#include "battery.h"
#include "leds.h"
#include <stdio.h>

#include "bmi160.h"

/*****************************************************************************
 * Defines
 *****************************************************************************/

#define BLE_ATTR_TABLE_SIZE              0x800                                      /**< The size of the attribute table used by the BLE stack. */

#define CENTRAL_LINK_COUNT               0                                          /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS       60                                         /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(20, UNIT_1_25_MS)           /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(40, UNIT_1_25_MS)           /**< Maximum acceptable connection interval. */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(sensor_event_t)                      /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. */

#define TEMP_SAMPLE_INTERVAL            50
#define SAMPLE_TIMER_TICKS              APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)

#define DEVICE_NAME_PREFIX  "NINA-B1-"                                              /**< Name prefix, 4 chars from the MAC will be added as well */
#define MANUFACTURER_NAME   "u-blox"                                                /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM_STR       "NINA-B1 Demo"
#define SERIAL_NUMBER_STR   "Serial Number"
#define HW_ID_STR           "0.1"
#ifndef FW_VERSION
#define FW_VERSION          "NO_VERSION"
#endif
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

//STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */

 /*****************************************************************************
  * Typedefs
  *****************************************************************************/

 /*****************************************************************************
  * Data
  *****************************************************************************/
static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_bas_t                         m_bas;                                     /**< Structure used to identify the battery service. */

static acc_service_t                     m_acc;                                     /**< Structure used to identify the accelerometer service. */
static gyro_service_t                    m_gyro;                                    /**< Structure used to identify the gyro service. */
static led_service_t                     m_led;                                     /**< Structure used to identify the LED service. */
static temp_service_t                    m_temp;                                    /**< Structure used to identify the Temperature service. */

APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */

static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

/**< We only advertise the fun services, i.e. Accelerometer, Gyro, Temp and Led. */
static ble_uuid_t m_adv_uuids[] = {{ACC_SRV_UUID,                        BLE_UUID_TYPE_BLE},
                                   {GYRO_SRV_UUID,                       BLE_UUID_TYPE_BLE},
                                   {LED_SRV_UUID,                        BLE_UUID_TYPE_BLE},
                                   {TEMP_SRV_UUID,                       BLE_UUID_TYPE_BLE}};

static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */

static ble_bas_init_t       bas_init;
static ble_dis_init_t       dis_init;
static acc_service_init_t   acc_init;
static gyro_service_init_t  gyro_init;
static led_service_init_t   led_init;
static temp_service_init_t  temp_init;

static char m_device_name[16];

/*****************************************************************************
 * Private method definitions
 *****************************************************************************/
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
        /* Debugger connected, break */
    	asm volatile ("bkpt 0");
    }

    NVIC_SystemReset();
}

/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    int8_t battery_level;

    UNUSED_PARAMETER(p_context);

    /* battery_level measure battery */
    battery_level = battery_sample();
    if (battery_level >= 0) {
        uint32_t err_code;
        err_code = ble_bas_battery_level_update(&m_bas, battery_level);
        printf("ble_bas_battery_level_update err_code=0x%X\r\n", (unsigned int)err_code);

        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != BLE_ERROR_NO_TX_PACKETS) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
        {
            APP_ERROR_HANDLER(err_code);
        }
    }

    /* Sample Temperature as well */
    sensor_get_temperature();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

static char get_char_from_nibble(uint8_t n)
{
    if (n > 9) {
        return 'A' + n - 10;
    } else {
        return '0' + n;
    }
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    size_t mac_start = strlen(DEVICE_NAME_PREFIX);
    strcpy(m_device_name, DEVICE_NAME_PREFIX);

    /* Append the last four nibbles of the MAC address to the device name */
    m_device_name[mac_start++] = get_char_from_nibble((NRF_FICR->DEVICEADDR[0] >> 12) & 0xF);
    m_device_name[mac_start++] = get_char_from_nibble((NRF_FICR->DEVICEADDR[0] >>  8) & 0xF);
    m_device_name[mac_start++] = get_char_from_nibble((NRF_FICR->DEVICEADDR[0] >>  4) & 0xF);
    m_device_name[mac_start++] = get_char_from_nibble( NRF_FICR->DEVICEADDR[0]        & 0xF);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)m_device_name,
                                          strlen(m_device_name));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void led_service_evt_handler(led_service_t * p_srv, led_service_evt_t * p_evt) {
    switch (p_evt->evt_type) {
        case LED_SRV_EVT_LED_CHANGE:
            leds_set_led(p_evt->p.led_change_params.led, p_evt->p.led_change_params.lit);
            break;

        case LED_SRV_EVT_COLOR_CHANGE:
            leds_set_color(p_evt->p.color.r, p_evt->p.color.g, p_evt->p.color.b);
            break;
    }
}

void set_dev_info(ble_dis_init_t *dis_init) {
    ble_srv_ascii_to_utf8(&dis_init->manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init->fw_rev_str, FW_VERSION);
    ble_srv_ascii_to_utf8(&dis_init->sw_rev_str, FW_VERSION);
    ble_srv_ascii_to_utf8(&dis_init->hw_rev_str, HW_ID_STR);
    ble_srv_ascii_to_utf8(&dis_init->serial_num_str, SERIAL_NUMBER_STR);
    ble_srv_ascii_to_utf8(&dis_init->model_num_str, MODEL_NUM_STR);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the services
 */
static void services_init(void)
{
    uint32_t       err_code;

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = battery_get_last_estimate();

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
    set_dev_info(&dis_init);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Accelerometer service
    memset(&acc_init, 0, sizeof(acc_init));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&acc_init.char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&acc_init.char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&acc_init.char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&acc_init.report_read_perm);

    acc_init.initial_range        = sensor_get_acc_range();

    err_code = acc_srv_init(&m_acc, &acc_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Gyro service
    memset(&gyro_init, 0, sizeof(gyro_init));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&gyro_init.char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&gyro_init.char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&gyro_init.char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&gyro_init.report_read_perm);

    err_code = gyro_srv_init(&m_gyro, &gyro_init);
    APP_ERROR_CHECK(err_code);

    // Initialize LED service
    memset(&led_init, 0, sizeof(led_init));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&led_init.char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&led_init.char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&led_init.char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&led_init.report_read_perm);

    led_init.evt_handler          = led_service_evt_handler;
    led_init.red_init_val         = leds_get(LED_RED) ? 1 : 0;
    led_init.green_init_val       = leds_get(LED_GREEN) ? 1 : 0;

    err_code = led_srv_init(&m_led, &led_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Temperature service
    memset(&temp_init, 0, sizeof(temp_init));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.report_read_perm);

    temp_init.temp_init_val = 0;

    err_code = temp_srv_init(&m_temp, &temp_init);
    APP_ERROR_CHECK(err_code);

    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_bas.battery_level_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_IDLE:
            sensor_enable_motion_detect();
            nrf_delay_ms(2);

            /**
             * Enter System Off power mode, the nRF will reset at motion event
             * from accelerometer.
             */
            APP_ERROR_CHECK(sd_power_system_off());
            break;
        default:
            break;
    }
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }

    return NRF_SUCCESS;
}

/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    int8_t        tx_power_level = 4;
    ble_advdata_t advdata;
    ble_advdata_t scan_response_data;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    memset(&scan_response_data, 0, sizeof(scan_response_data));

    /* Advertising data, uuids and flags */
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    advdata.p_tx_power_level        = &tx_power_level;

    /* Scan response, Device name */
    scan_response_data.name_type    = BLE_ADVDATA_FULL_NAME;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scan_response_data, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void notify_acc(sensor_event_t *p_evt) {
    uint32_t err_code;
    int8_t axis[6];

    /* Convert to 8-bit */
    axis[0] = (p_evt->p.sensor_data.acc_x / 256);
    axis[1] = (p_evt->p.sensor_data.acc_y / 256);
    axis[2] = (p_evt->p.sensor_data.acc_z / 256);
    axis[3] = (p_evt->p.sensor_data.sensor_time >> 16) & 0xFF;
    axis[4] = (p_evt->p.sensor_data.sensor_time >> 8) & 0xFF;
    axis[5] = p_evt->p.sensor_data.sensor_time & 0xFF;

    err_code = acc_srv_notify_axis(&m_acc, (uint8_t)axis[0], &m_acc.acc_x);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }

    err_code = acc_srv_notify_axis(&m_acc, (uint8_t)axis[1], &m_acc.acc_y);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }

    err_code = acc_srv_notify_axis(&m_acc, (uint8_t)axis[2], &m_acc.acc_z);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }

    err_code = acc_srv_notify_axis_aggregate(&m_acc, axis, 6);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }

    printf("a %d, %d, %d\r\n", p_evt->p.sensor_data.acc_x, p_evt->p.sensor_data.acc_y, p_evt->p.sensor_data.acc_z);
}

static void notify_gyro(sensor_event_t *p_evt) {
    uint32_t err_code;
    int8_t axis[6];

    /* Convert to 8-bit */
    axis[0] = (p_evt->p.sensor_data.gyro_x / 256);
    axis[1] = (p_evt->p.sensor_data.gyro_y / 256);
    axis[2] = (p_evt->p.sensor_data.gyro_z / 256);
    axis[3] = (p_evt->p.sensor_data.sensor_time >> 16) & 0xFF;
    axis[4] = (p_evt->p.sensor_data.sensor_time >> 8) & 0xFF;
    axis[5] = p_evt->p.sensor_data.sensor_time & 0xFF;

    err_code = gyro_srv_notify_axis(&m_gyro, axis[0], &m_gyro.gyro_x);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }

    err_code = gyro_srv_notify_axis(&m_gyro, axis[1], &m_gyro.gyro_y);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }

    err_code = gyro_srv_notify_axis(&m_gyro, axis[2], &m_gyro.gyro_z);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }

    err_code = gyro_srv_notify_axis_aggregate(&m_gyro, axis, 6);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }

    printf("g %d, %d, %d\r\n", p_evt->p.sensor_data.gyro_x, p_evt->p.sensor_data.gyro_y, p_evt->p.sensor_data.gyro_z);
}

static void handle_sensor_data(void * p_event_data, uint16_t event_size)
{
    sensor_event_t *evt = (sensor_event_t *) p_event_data;

    APP_ERROR_CHECK_BOOL(event_size == sizeof(sensor_event_t));

    notify_acc(evt);
    notify_gyro(evt);
}

static void handle_sensor_nomotion(void * p_event_data, uint16_t event_size)
{
    /* No action */
}

static void handle_temp(void * p_event_data, uint16_t event_size)
{
    uint32_t err_code;
    sensor_event_t *evt = (sensor_event_t *) p_event_data;
    int8_t temp_c;

    APP_ERROR_CHECK_BOOL(event_size == sizeof(sensor_event_t));

    /* Convert int16_t value to degrees Celsius */
    temp_c = (evt->p.temperature * 195) / 100000 + 23;

    err_code = temp_srv_notify_change(&m_temp, temp_c);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }
    printf("New temp: %d [%d]\r\n", temp_c, evt->p.temperature);
}

void sensor_event_handler_cb(sensor_event_t evt)
{
    uint32_t err_code;

    switch (evt.evt) {
        case SENSOR_EVENT_SENSOR_DATA:
            /* Post to scheduler, we cannot handle the event in this context */
            err_code = app_sched_event_put(&evt, sizeof(evt), handle_sensor_data);
            APP_ERROR_CHECK(err_code);
            break;

        case SENSOR_EVENT_TEMPERATURE:
            /* Post to scheduler, we cannot handle the event in this context */
            err_code = app_sched_event_put(&evt, sizeof(evt), handle_temp);
            APP_ERROR_CHECK(err_code);
            break;

        case SENSOR_EVENT_NO_MOTION:
            /* Post to scheduler, we cannot handle the event in this context */
            err_code = app_sched_event_put(&evt, sizeof(evt), handle_sensor_nomotion);
            APP_ERROR_CHECK(err_code);
            break;
    }
}

static void set_sensor_init_vals() {
    sensor_event_t sensor_vals;
    uint32_t err_code;

    /* Sample accelerometer and update values */
    err_code = sensor_read_acc_sensor_data(&sensor_vals.p.sensor_data.acc_x,
            &sensor_vals.p.sensor_data.acc_y, &sensor_vals.p.sensor_data.acc_z);
    APP_ERROR_CHECK(err_code);
    notify_acc(&sensor_vals);

    /* Sample gyro and update values */
    err_code = sensor_read_gyro_sensor_data(&sensor_vals.p.sensor_data.gyro_x,
            &sensor_vals.p.sensor_data.gyro_y, &sensor_vals.p.sensor_data.gyro_z);
    APP_ERROR_CHECK(err_code);
    notify_gyro(&sensor_vals);
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            sensors_set_normal_mode();
            leds_blink_stop();
            sensor_get_temperature();

            set_sensor_init_vals();
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            sensor_enable_motion_detect();
            nrf_delay_ms(2);

            /**
             * Enter System Off power mode, the nRF will reset at motion event
             * from accelerometer.
             */
            APP_ERROR_CHECK(sd_power_system_off());
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    acc_srv_on_ble_evt(&m_acc, p_ble_evt);
    gyro_srv_on_ble_evt(&m_gyro, p_ble_evt);
    led_srv_on_ble_evt(&m_led, p_ble_evt);
    temp_srv_on_ble_evt(&m_temp, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */

    on_ble_evt(p_ble_evt);

}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_enable_params.gatts_enable_params.service_changed = 1;
    ble_enable_params.gatts_enable_params.attr_tab_size = BLE_ATTR_TABLE_SIZE;

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/*****************************************************************************
 * Public Methods
 *****************************************************************************/
/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    NRF_POWER->DCDCEN = (POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos);

    APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);

    // Initialize.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    APP_ERROR_CHECK(nrf_drv_gpiote_init());
    leds_init();
    app_trace_init();
    timers_init();
    sensor_init(sensor_event_handler_cb);
    battery_init();

    ble_stack_init();
    device_manager_init(true);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();

    // Start execution.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    leds_blink_advertising();

    leds_set_led(LED_RED, true);
    nrf_delay_ms(200);
    leds_set_led(LED_RED, false);
    nrf_delay_ms(200);
    leds_set_led(LED_RED, true);
    nrf_delay_ms(200);
    leds_set_led(LED_RED, false);

    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}
