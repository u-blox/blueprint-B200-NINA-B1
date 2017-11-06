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
 * BMI160, Accelerometer 6-AXIS
 */

//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include "bmi160.h"

#include "pins.h"
#include "bmi160_reg_bits.h"
#include "app_twi.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"
#include "app_scheduler.h"
#include "nrf_delay.h"
#include "nrf_drv_config.h"

//@} End of Includes


//////////////////////////////////////////////////////////////////////
/// @name Defines
//@{

#define MAX_PENDING_TRANSACTIONS    5
#define BMI160_ADDRESS              0x68
#define DEVICE_ID                   0xD1
#define RX_BUFFER_SIZE              32

//@} End of Defines


//////////////////////////////////////////////////////////////////////
/// @name Typedefs
//@{

//@} End of Typedefs


//////////////////////////////////////////////////////////////////////
/// @name Data
//@{

/* ODR = 12.5 Hz */
static const uint8_t ACC_CONF_LP_MODE[]  = {ACC_CONF, 0x85};

/* Accelerometer Range = +-2g */
static const uint8_t ACC_RANGE_INIT[] = {ACC_RANGE, 0x03};
#define              DEFAULT_RANGE 2

/* Gyro Range = +-500deg/s */
static const uint8_t GYRO_RANGE_INIT[] = {GYR_RANGE, 0x02};

/* Set Accelerometer in Normal Mode */
static const uint8_t CMD_ACC_NORMAL_MODE[] = {CMD, CMD_ACC_PMU_NORMAL};

/* Set Accelerometer in Low power mode */
static const uint8_t CMD_ACC_LP_MODE[] = {CMD, CMD_ACC_PMU_LOW_PWR};

/* Set Accelerometer in Suspended power mode */
static const uint8_t CMD_ACC_SUSPEND_MODE[] = {CMD, CMD_ACC_PMU_SUSPEND};

/* Set Gyro in Normal Mode */
static const uint8_t CMD_GYRO_NORMAL_MODE[] = {CMD, CMD_GYR_PMU_NORMAL};

/* Set Gyro in Suspended Mode */
static const uint8_t CMD_GYRO_SUSPEND_MODE[] = {CMD, CMD_GYR_PMU_SUSPEND};

/* Initialization sequence */
static const app_twi_transfer_t const bmi160_init_transfers[] = {
        APP_TWI_WRITE(BMI160_ADDRESS, ACC_CONF_LP_MODE, sizeof(ACC_CONF_LP_MODE), APP_TWI_NO_STOP),
        APP_TWI_WRITE(BMI160_ADDRESS, ACC_RANGE_INIT, sizeof(ACC_RANGE_INIT), APP_TWI_NO_STOP),
        APP_TWI_WRITE(BMI160_ADDRESS, GYRO_RANGE_INIT, sizeof(GYRO_RANGE_INIT), APP_TWI_NO_STOP),
        APP_TWI_WRITE(BMI160_ADDRESS, CMD_ACC_SUSPEND_MODE, sizeof(CMD_ACC_SUSPEND_MODE), APP_TWI_NO_STOP),
        APP_TWI_WRITE(BMI160_ADDRESS, CMD_GYRO_SUSPEND_MODE, sizeof(CMD_GYRO_SUSPEND_MODE), 0)
};

/**
 * Map Anymotion interrupt to INT1 output pin
 */
static const uint8_t MOTION_DETECT_MAP_INTERRUPT[] = {INT_MAP_0, 0x04 };

/**
 * Enable Anymotion interrupt on all axises
 */
static const uint8_t MOTION_DETECT_ANY_MOTION_INT_EN[] = {INT_EN_0, 0x07 };

/**
 * Enable Anymotion interrupt on all axises
 * int1_output_en = 1, int1_od = 0, int1_lvl = 1, int1_edge_ctrl = 1
 */
static const uint8_t MOTION_DETECT_INT_OUTPUT_EN[] = {INT_OUT_CTRL, 0x0B };

/**
 * Enable temporary interrupt output, a 10ms pulse
 * 0x06 => 10ms pulse on interrupt
 * 0x0F => Interrupt is latched
 *
 * In the Anymoton detect case we want to use the Latched option, since this
 * will wake up the nRF from Power Off.
 */
static const uint8_t MOTION_DETECT_INT_LATCH[] = {INT_LATCH, 0x0F };

/**
 *  Duration is the number of data points above Motion TH until interrupt are generated
 *  Duration = int_anym_dur + 1, int_anym_dur = 1 => 2 samples above th to generate interrupt.
 */
static const uint8_t MOTION_DETECT_DURATION[] = {INT_MOTION_0, 0x00 };

/**
 * ANY_MOTION_TH is range dependant and is calculated :
 *  th_mg =  3.91 * int_anym_th (2G)
 *  th_mg =  7.81 * int_anym_th (4G)
 *  th_mg = 15.63 * int_anym_th (8G)
 *  th_mg = 31.25 * int_anym_th (16G)
 *
 *  Default: 0x14:
 *  3.91 * 0x14 = 78.2 mG threshold
 *
 *  3.91 * 0x80 = ~0.5 G
 */
static const uint8_t MOTION_DETECT_TH[] = {INT_MOTION_1, 0x14 };

/**
 * Anymotion duration = 0
 * Nomotion duration = 0b001111 => 20.48s
 */
static const uint8_t NORMAL_MOTION_DETECT_DURATION[] = {INT_MOTION_0, 0x3C };

/**
 * AnyMotion Threshold
 *
 * 0x04 => 3.91 * 4 ~= 15 mG.
 * 0x02 => 3.91 * 2  = 7,82 mG.
 */
static const uint8_t NORMAL_MOTION_DETECT_TH[] = {INT_MOTION_1, 0x02 };

/**
 * NoMotion Threshold
 *
 * 0x04 => 3.91 * 4 ~= 15 mG.
 */
static const uint8_t NORMAL_NO_MOTION_DETECT_TH[] = {INT_MOTION_2, 0x04 };

/**
 * Select No motion interrupt
 */
static const uint8_t NORMAL_INT_MOTION_3[] = {INT_MOTION_3, 0x01 };

/* ODR = 50 Hz */
static const uint8_t NORMAL_ACC_CONF[]  = {ACC_CONF, 0x07};

/* Gyro ODR 25 Hz */
static const uint8_t NORMAL_GYRO_CONF[] = {GYR_CONF, 0x26};

/**
 * In the DataReady case, we use a 10ms pulse to notify the nRF that data is
 * ready
 */
static const uint8_t NORMAL_INT_LATCH[] = {INT_LATCH, 0x06 };

/**
 * Enable No-motion detect on all axises
 */
static const uint8_t NORMAL_INT_EN_2[] = {INT_EN_2, 0x07 };

/**
 * Nomotion interrupt on INT2
 */
static const uint8_t NORMAL_INT_MAP_2[] = {INT_MAP_2, 0x08};

/**
 * int1_output_en = 1, int1_od = 0, int1_lvl = 1, int1_edge_ctrl = 1
 */
static const uint8_t NORMAL_INT_OUTPUT_EN[] = {INT_OUT_CTRL, 0xBB };

/**
 * No motion triggers gyro_sleep_trigger = 0b00000100
 * ANymotion triggers gyro_wakup_trigger = 0b00010000
 */
static const uint8_t NORMAL_PMU_TRIGGER[] = {PMU_TRIGGER, 0x14 };

/* MEMS chip Soft reset command */
static const uint8_t CMD_SOFT_RESET[] = {CMD, CMD_SOFTRESET};

static uint8_t TEMP_REQ = TEMPERATURE_0;

static sensor_event_handler_cb_t    m_handler;
static app_twi_t                    m_app_twi = APP_TWI_INSTANCE(0);
static uint8_t                      m_data_buffer[RX_BUFFER_SIZE];
static uint8_t                      m_req_buffer[2];

static uint8_t                      m_temp_buffer[2];
static uint8_t                      m_range = DEFAULT_RANGE;

//@} End of Data


//////////////////////////////////////////////////////////////////////
/// @name Private Methods
//@{

static void read_sensor_data_handler(ret_code_t result, void * p_user_data)
{
    if (m_handler != NULL) {
        sensor_event_t e;

        if (result == NRF_SUCCESS) {
            e.evt = SENSOR_EVENT_SENSOR_DATA;

            e.p.sensor_data.gyro_x = m_data_buffer[0] | (m_data_buffer[1] << 8);
            e.p.sensor_data.gyro_y = m_data_buffer[2] | (m_data_buffer[3] << 8);
            e.p.sensor_data.gyro_z = m_data_buffer[4] | (m_data_buffer[5] << 8);
            e.p.sensor_data.acc_x  = m_data_buffer[6] | (m_data_buffer[7] << 8);
            e.p.sensor_data.acc_y  = m_data_buffer[8] | (m_data_buffer[9] << 8);
            e.p.sensor_data.acc_z  = m_data_buffer[10] | (m_data_buffer[11] << 8);

            e.p.sensor_data.sensor_time = m_data_buffer[12] | (m_data_buffer[13] << 8) | (m_data_buffer[14] << 16);

            m_handler(e);
        } else {
            //APP_ERROR_HANDLER(result);
        }
    }
}

static void get_temp_handler(ret_code_t result, void * p_user_data)
{
    if (m_handler != NULL) {
        sensor_event_t e;

        if (result == NRF_SUCCESS) {
            e.evt = SENSOR_EVENT_TEMPERATURE;
            e.p.temperature = m_temp_buffer[0] | (m_temp_buffer[1] << 8);
            m_handler(e);
        } else {
            //APP_ERROR_HANDLER(result);
        }
    }
}

static void read_on_motion(void * p_event_data, uint16_t event_size)
{
    APP_ERROR_CHECK(sensor_get_sensor_data());
}

static void motion_detect_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t err_code;

    /**
     * Motion detected in normal mode, read data registers. However, this can
     * not be done in this context so lets do it in the app scheduler context
     */
    err_code = app_sched_event_put(NULL, 0, read_on_motion);
    APP_ERROR_CHECK(err_code);
}

static void nomotion_detect_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    /**
     * No Motion detected in normal mode, the Gyro is now in Fast Startup mode.
     */
    if (m_handler != NULL) {
        sensor_event_t e;

        e.evt = SENSOR_EVENT_NO_MOTION;
        m_handler(e);
    }
}

static void twi_init(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = PIN_SENSOR_I2C_SCL,
       .sda                = PIN_SENSOR_I2C_SDA,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
}

static uint8_t read_error()
{
    m_req_buffer[0] = ERR_REG;

    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, m_req_buffer, 1, APP_TWI_NO_STOP),
            APP_TWI_READ(BMI160_ADDRESS, m_data_buffer, 1, 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, transfers,
            sizeof(transfers) / sizeof(transfers[0]), NULL));

    return m_data_buffer[0];
}

static void chip_reset()
{
    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, CMD_SOFT_RESET, sizeof(CMD_SOFT_RESET), 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, transfers,
            sizeof(transfers) / sizeof(transfers[0]), NULL));
}

uint8_t read_reg(uint8_t reg)
{
    m_req_buffer[0] = reg;

    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, m_req_buffer, 1, APP_TWI_NO_STOP),
            APP_TWI_READ(BMI160_ADDRESS, m_data_buffer, 1, 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, transfers,
            sizeof(transfers) / sizeof(transfers[0]), NULL));

    return m_data_buffer[0];
}

//@} End of Private Methods


//////////////////////////////////////////////////////////////////////
/// @name Public Methods
//@{

void sensor_init(sensor_event_handler_cb_t handler)
{
    uint8_t chip_id;
    uint8_t err_code;

    m_handler = handler;

    twi_init();

    /* Start with a chip reset to make sure we have a known state */
    chip_reset();

    /* Verify Chip ID */
    chip_id = sensor_read_chip_id();
    APP_ERROR_CHECK_BOOL(chip_id == DEVICE_ID);

	/* Initialize sensor */
    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, bmi160_init_transfers,
            sizeof(bmi160_init_transfers) / sizeof(bmi160_init_transfers[0]), NULL));

    /* Check for errors */
    err_code = read_error();
    APP_ERROR_CHECK_BOOL(err_code == 0);
}

void sensors_set_normal_mode(void)
{
    uint32_t err_code;

    /* Set Gyro and Acc in Normal mode, sampling rate so something useful */
    /* Set undersampling acc_us = 0, acc_bwp = 0b010 */
    static app_twi_transfer_t const drdy_transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_ACC_CONF, sizeof(NORMAL_ACC_CONF), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_GYRO_CONF, sizeof(NORMAL_GYRO_CONF), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_INT_MAP_2, sizeof(NORMAL_INT_MAP_2), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_INT_LATCH, sizeof(NORMAL_INT_LATCH), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_INT_EN_2, sizeof(NORMAL_INT_EN_2), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_INT_OUTPUT_EN, sizeof(NORMAL_INT_OUTPUT_EN), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_INT_MOTION_3, sizeof(NORMAL_INT_MOTION_3), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_NO_MOTION_DETECT_TH, sizeof(NORMAL_NO_MOTION_DETECT_TH), 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, drdy_transfers,
            sizeof(drdy_transfers) / sizeof(drdy_transfers[0]), NULL));

    /* Configure Motion detect */
    static app_twi_transfer_t const motion_detect_transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, MOTION_DETECT_MAP_INTERRUPT, sizeof(MOTION_DETECT_MAP_INTERRUPT), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, MOTION_DETECT_ANY_MOTION_INT_EN, sizeof(MOTION_DETECT_ANY_MOTION_INT_EN), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_MOTION_DETECT_TH, sizeof(NORMAL_MOTION_DETECT_TH), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_MOTION_DETECT_DURATION, sizeof(NORMAL_MOTION_DETECT_DURATION), 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, motion_detect_transfers,
            sizeof(motion_detect_transfers) / sizeof(motion_detect_transfers[0]), NULL));

    /* Configure No-motion detect so it will turn Gyro to sleep */
    static app_twi_transfer_t const no_motion_detect_transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, NORMAL_PMU_TRIGGER, sizeof(NORMAL_PMU_TRIGGER), APP_TWI_NO_STOP),
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, no_motion_detect_transfers,
            sizeof(no_motion_detect_transfers) / sizeof(no_motion_detect_transfers[0]), NULL));

    /* Set sensors to Normal Mode */
    static app_twi_transfer_t const set_acc_mode_transfers[] = {
        APP_TWI_WRITE(BMI160_ADDRESS, CMD_ACC_NORMAL_MODE, sizeof(CMD_ACC_NORMAL_MODE), 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, set_acc_mode_transfers,
            sizeof(set_acc_mode_transfers) / sizeof(set_acc_mode_transfers[0]), NULL));

    nrf_delay_ms(2);

    /* Set sensors to Normal Mode */
    static app_twi_transfer_t const set_gyro_mode_transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, CMD_GYRO_NORMAL_MODE, sizeof(CMD_GYRO_NORMAL_MODE), 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, set_gyro_mode_transfers,
            sizeof(set_gyro_mode_transfers) / sizeof(set_gyro_mode_transfers[0]), NULL));

    /* Configure pin change interrupt on INT1, Any motion detected */
    const nrf_drv_gpiote_in_config_t int1_cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    nrf_gpio_cfg_input(PIN_SENSOR_INT_1, NRF_GPIO_PIN_NOPULL);
    nrf_drv_gpiote_in_init(PIN_SENSOR_INT_1, &int1_cfg, motion_detect_handler);
    nrf_drv_gpiote_in_event_enable(PIN_SENSOR_INT_1, true);

    /* Configure pin change interrupt on INT2, No motion detected */
    const nrf_drv_gpiote_in_config_t int2_cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    nrf_gpio_cfg_input(PIN_SENSOR_INT_2, NRF_GPIO_PIN_NOPULL);
    nrf_drv_gpiote_in_init(PIN_SENSOR_INT_2, &int2_cfg, nomotion_detect_handler);
    nrf_drv_gpiote_in_event_enable(PIN_SENSOR_INT_2, true);

    err_code = read_error();
    APP_ERROR_CHECK_BOOL(err_code == 0);
}

uint8_t sensor_read_chip_id()
{
    uint32_t err_code;

    m_req_buffer[0] = CHIP_ID;

    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, m_req_buffer, 1, APP_TWI_NO_STOP),
            APP_TWI_READ(BMI160_ADDRESS, m_data_buffer, 1, 0)
    };

    err_code = app_twi_perform(&m_app_twi, transfers,
            sizeof(transfers) / sizeof(transfers[0]), NULL);

    APP_ERROR_CHECK(err_code);

    return m_data_buffer[0];
}

uint8_t sensor_read_mode()
{
    m_req_buffer[0] = PMU_STATUS;

    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, m_req_buffer, 1, APP_TWI_NO_STOP),
            APP_TWI_READ(BMI160_ADDRESS, m_data_buffer, 1, 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, transfers,
            sizeof(transfers) / sizeof(transfers[0]), NULL));

    return m_data_buffer[0];
}

uint32_t sensor_get_sensor_data()
{
    m_req_buffer[0] = DATA_8;
    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, m_req_buffer, 1, APP_TWI_NO_STOP),
            APP_TWI_READ(BMI160_ADDRESS, m_data_buffer, 15, 0)
    };
    static app_twi_transaction_t const transaction = {
            .callback = read_sensor_data_handler,
            .p_user_data = NULL,
            .p_transfers = transfers,
            .number_of_transfers = 2
    };
    return app_twi_schedule(&m_app_twi, &transaction);
}

uint32_t sensor_read_acc_sensor_data(int16_t *p_x, int16_t *p_y, int16_t *p_z)
{
    uint32_t err_code;

    m_req_buffer[0] = DATA_14;
    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, m_req_buffer, 1, APP_TWI_NO_STOP),
            APP_TWI_READ(BMI160_ADDRESS, m_data_buffer, 6, 0)
    };

    err_code = app_twi_perform(&m_app_twi, transfers,
            sizeof(transfers) / sizeof(transfers[0]), NULL);
    if (err_code == NRF_SUCCESS) {
        *p_x = m_data_buffer[0] | (m_data_buffer[1] << 8);
        *p_y = m_data_buffer[2] | (m_data_buffer[3] << 8);
        *p_z = m_data_buffer[4] | (m_data_buffer[5] << 8);
    }

    return err_code;
}

uint32_t sensor_read_gyro_sensor_data(int16_t *p_x, int16_t *p_y, int16_t *p_z)
{
    uint32_t err_code;

    m_req_buffer[0] = DATA_8;
    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, m_req_buffer, 1, APP_TWI_NO_STOP),
            APP_TWI_READ(BMI160_ADDRESS, m_data_buffer, 6, 0)
    };

    err_code = app_twi_perform(&m_app_twi, transfers,
            sizeof(transfers) / sizeof(transfers[0]), NULL);
    if (err_code == NRF_SUCCESS) {
        *p_x = m_data_buffer[0] | (m_data_buffer[1] << 8);
        *p_y = m_data_buffer[2] | (m_data_buffer[3] << 8);
        *p_z = m_data_buffer[4] | (m_data_buffer[5] << 8);
    }

    return err_code;
}

uint32_t sensor_get_temperature()
{
    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, &TEMP_REQ, 1, APP_TWI_NO_STOP),
            APP_TWI_READ(BMI160_ADDRESS, m_temp_buffer, 2, 0)
    };
    static app_twi_transaction_t const transaction = {
            .callback = get_temp_handler,
            .p_user_data = NULL,
            .p_transfers = transfers,
            .number_of_transfers = 2
    };

    return app_twi_schedule(&m_app_twi, &transaction);
}

uint8_t sensor_get_acc_range()
{
    return m_range;
}

uint32_t sensor_set_range(uint8_t range)
{
    uint8_t range_setting;
    uint32_t err_code;

    if (range == 2) range_setting = 3;
    else if (range == 4) range_setting = 5;
    else if (range == 8) range_setting = 8;
    else if (range == 16) range_setting = 12;
    else {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_req_buffer[0] = ACC_RANGE;
    m_req_buffer[1] = range_setting;

    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, m_req_buffer, 2, APP_TWI_NO_STOP),
    };

    err_code = app_twi_perform(&m_app_twi, transfers,
            sizeof(transfers) / sizeof(transfers[0]), NULL);

    if (err_code == NRF_SUCCESS) {
        m_range = range;
    }

    return err_code;
}

void sensor_enable_motion_detect(void)
{
    /**
     * Configure AnyMotion detection
     */
    static app_twi_transfer_t const transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, MOTION_DETECT_DURATION, sizeof(MOTION_DETECT_DURATION), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, MOTION_DETECT_TH, sizeof(MOTION_DETECT_TH), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, MOTION_DETECT_MAP_INTERRUPT, sizeof(MOTION_DETECT_MAP_INTERRUPT), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, MOTION_DETECT_INT_OUTPUT_EN, sizeof(MOTION_DETECT_INT_OUTPUT_EN), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, MOTION_DETECT_INT_LATCH, sizeof(MOTION_DETECT_INT_LATCH), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, MOTION_DETECT_ANY_MOTION_INT_EN, sizeof(MOTION_DETECT_ANY_MOTION_INT_EN), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, ACC_CONF_LP_MODE, sizeof(ACC_CONF_LP_MODE), APP_TWI_NO_STOP),
            APP_TWI_WRITE(BMI160_ADDRESS, CMD_ACC_LP_MODE, sizeof(CMD_ACC_LP_MODE), 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, transfers,
            sizeof(transfers) / sizeof(transfers[0]), NULL));

    /* Set sensors to Normal Mode */
    static app_twi_transfer_t const set_gyro_mode_transfers[] = {
            APP_TWI_WRITE(BMI160_ADDRESS, CMD_GYRO_SUSPEND_MODE, sizeof(CMD_GYRO_SUSPEND_MODE), 0)
    };

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, set_gyro_mode_transfers,
            sizeof(set_gyro_mode_transfers) / sizeof(set_gyro_mode_transfers[0]), NULL));

    /* Configure Sense on INT1 input */
    nrf_gpio_cfg_sense_input(PIN_SENSOR_INT_1, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
}

//@} End of Public Methods
