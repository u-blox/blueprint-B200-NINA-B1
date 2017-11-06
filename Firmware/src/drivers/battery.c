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
#include "battery.h"

#include "app_util_platform.h"
#include "nrf_drv_config.h"
#include "nrf_drv_saadc.h"

/*****************************************************************************
 * Defines
 *****************************************************************************/
#define SAMPLE_BUFFER_LENGTH 10

#define INTERNAL_REFERENCE_mV 600

/**
 * The remaining battery estimate is divided into three intervals,
 * 100 - 90% => 3.0V  > Voltage > 2.6V
 *  90 - 15% => 2.6V  > Voltage > 2.35V
 *  15 - 0 % => 2.35V > Voltage > 2.0V
 *
 *  In these intervals the relation capacity in percent / battery voltage is
 *  assumed to be linear.
 */
#define FULL_RANGE_L_mV 3000
#define HI_RANGE_L_mV   2600
#define MID_RANGE_L_mV  2350
#define LOW_RANGE_L_mV  2000

/*****************************************************************************
 * Typedefs
 *****************************************************************************/

/*****************************************************************************
 * Data
 *****************************************************************************/
static uint16_t m_sample_buffer[SAMPLE_BUFFER_LENGTH];
static uint32_t m_sample_idx;
static uint8_t   m_last_estimate;

/***************************************************************************** 
 * Private method definitions
 *****************************************************************************/

static void nrf_drv_saadc_event_handler(nrf_drv_saadc_evt_t const * p_event) {

}

static uint16_t sample_batt()
{
    nrf_saadc_value_t val;
    ret_code_t error_code;

    error_code = nrf_drv_saadc_sample_convert(0, &val);
    APP_ERROR_CHECK(error_code);

    return val;
}

static uint8_t get_estimate(uint16_t adc_value)
{
    int8_t estimate;
    uint32_t vbat_mV;

    vbat_mV = ((INTERNAL_REFERENCE_mV * adc_value) / 0x3FF) * 6;

    if (vbat_mV > FULL_RANGE_L_mV) {
        estimate = 100;
    } else if (vbat_mV > HI_RANGE_L_mV) {
        /* 100 - 90% => 3.0V > Voltage > 2.6V */
        estimate = ((vbat_mV - HI_RANGE_L_mV) * 10) / 400 + 90;
    } else if (vbat_mV > MID_RANGE_L_mV) {
        /* 90 - 15% => 2.6V > Voltage > 2.35V */
        estimate = ((vbat_mV - MID_RANGE_L_mV) * 75) / 250 + 15;
    } else if (vbat_mV > LOW_RANGE_L_mV) {
        /* 15 - 0 % => 2.35V > Voltage > 2.0V */
        estimate = ((vbat_mV - LOW_RANGE_L_mV) * 15) / 350;
    } else {
        estimate = 0;
    }

    return estimate;
}

/*****************************************************************************
 * Public Methods
 *****************************************************************************/
void battery_init()
{
    ret_code_t error_code;
    uint16_t vbat;

    nrf_drv_saadc_config_t config = {
            .resolution = SAADC_CONFIG_RESOLUTION,
            .oversample = SAADC_CONFIG_OVERSAMPLE,
            .interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY
    };
    nrf_saadc_channel_config_t ch_config = {
            .resistor_p = SAADC_CH_CONFIG_RESP_Bypass,
            .resistor_n = SAADC_CH_CONFIG_RESP_Bypass,
            .gain = NRF_SAADC_GAIN1_6,
            .reference = NRF_SAADC_REFERENCE_INTERNAL,
            .acq_time = NRF_SAADC_ACQTIME_10US,
            .mode = NRF_SAADC_MODE_SINGLE_ENDED,
            .pin_p = NRF_SAADC_INPUT_VDD,
            .pin_n = NRF_SAADC_INPUT_DISABLED
    };

    error_code = nrf_drv_saadc_init(&config, nrf_drv_saadc_event_handler);
    APP_ERROR_CHECK(error_code);

    error_code = nrf_drv_saadc_channel_init(0, &ch_config);
    APP_ERROR_CHECK(error_code);

    m_sample_idx = 0;

    vbat = sample_batt();
    m_last_estimate = get_estimate(vbat);
}

int8_t battery_sample()
{
    int8_t estimate = -1;

    m_sample_buffer[m_sample_idx++] = sample_batt();

    printf("New battery sample: %d\r\n", m_sample_buffer[m_sample_idx - 1]);

    if (m_sample_idx == SAMPLE_BUFFER_LENGTH) {
        uint32_t tot = 0;
        uint32_t mean;

        m_sample_idx = 0;

        /* Calculate mean value */
        for (int i = 0; i < SAMPLE_BUFFER_LENGTH; i++) {
            tot += m_sample_buffer[0];
        }
        mean = tot / SAMPLE_BUFFER_LENGTH;

        estimate = get_estimate(mean);
        m_last_estimate = estimate;

        printf("New estimate: %d [adval=%d]\r\n", estimate, (int)mean);
    }

    return estimate;
}

uint8_t battery_get_last_estimate()
{
    return m_last_estimate;
}
