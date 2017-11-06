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
#include "leds.h"

#include "nrf.h"
#include "nrf_gpio.h"
#include "led_softblink.h"
#include "app_error.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_config.h"

/*****************************************************************************
 * Defines
 *****************************************************************************/
/**
 * @brief LED softblink config for advertising
 */
#define LED_SB_INIT_PARAMS_ADVERTISING(mask)    \
{                                               \
    .active_high        = false,                \
    .duty_cycle_max     = 220,                  \
    .duty_cycle_min     = 0,                    \
    .duty_cycle_step    = 5,                    \
    .off_time_ticks     = 0x3FFF,               \
    .on_time_ticks      = 0,                    \
    .leds_pin_bm        = (1 << PIN_LED_GREEN)  \
}

/*****************************************************************************
 * Typedefs
 *****************************************************************************/

/*****************************************************************************
 * Data
 *****************************************************************************/
static nrf_drv_pwm_t m_pwm = NRF_DRV_PWM_INSTANCE(0);
static bool m_pwm_enabled;

/***************************************************************************** 
 * Private method definitions
 *****************************************************************************/

/*****************************************************************************
 * Public Methods
 *****************************************************************************/
void leds_init(void)
{
    /* Config as outputs */
    nrf_gpio_cfg_output(PIN_LED_RED);
    nrf_gpio_cfg_output(PIN_LED_GREEN);
    nrf_gpio_cfg_output(PIN_LED_BLUE);

    /* Turn off LEDs at start */
    nrf_gpio_pin_set(PIN_LED_RED);
    nrf_gpio_pin_set(PIN_LED_GREEN);
    nrf_gpio_pin_set(PIN_LED_BLUE);

    m_pwm_enabled = false;
}

void leds_set_led(led_t led, bool lit)
{
    uint32_t pin;

    switch (led) {
        case LED_RED:
            pin = PIN_LED_RED;
            break;

        case LED_GREEN:
            pin = PIN_LED_GREEN;
            break;

        case LED_BLUE:
            pin = PIN_LED_BLUE;
            break;

        default:
            APP_ERROR_CHECK_BOOL(false);
            return;
    }

    if (lit) {
        nrf_gpio_pin_clear(pin);
    } else {
        nrf_gpio_pin_set(pin);
    }
}

bool leds_get(led_t led)
{
    uint32_t pin;

    switch (led) {
        case LED_RED:
            pin = PIN_LED_RED;
            break;

        case LED_GREEN:
            pin = PIN_LED_GREEN;
            break;

        case LED_BLUE:
            pin = PIN_LED_BLUE;
            break;

        default:
            APP_ERROR_CHECK_BOOL(false);
            return false;
    }

    return (NRF_GPIO->OUT & (1 << pin)) == 0;
}

void leds_blink_advertising(void)
{
    led_sb_init_params_t params = LED_SB_INIT_PARAMS_ADVERTISING((1 << PIN_LED_GREEN));
    led_softblink_init(&params);
    led_softblink_start((1 << PIN_LED_GREEN));
}

void leds_blink_connected(void)
{
    /* Increase the Off time */
    led_softblink_off_time_set(0x7FFF);
}

void leds_blink_stop(void)
{
    led_softblink_stop();
}

void leds_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    uint32_t err_code;

    nrf_drv_pwm_config_t const config =
    {
            .output_pins =
            {
                    PIN_LED_RED,
                    PIN_LED_GREEN,
                    PIN_LED_BLUE,
                    NRF_DRV_PWM_PIN_NOT_USED
            },
            .base_clock = NRF_PWM_CLK_125kHz,
            .count_mode = NRF_PWM_MODE_UP,
            .top_value  = 0xFF,
            .load_mode  = NRF_PWM_LOAD_INDIVIDUAL,
            .step_mode  = NRF_PWM_STEP_AUTO
    };

    if (m_pwm_enabled) {
        nrf_drv_pwm_uninit(&m_pwm);

        m_pwm_enabled = false;
    }

    if (red == 0 && green == 0 && blue == 0) {
        /* Stop using the PWM, but make sure all LEDs are off */
        leds_set_led(LED_RED, false);
        leds_set_led(LED_GREEN, false);
        leds_set_led(LED_BLUE, false);

        return;
    }

    err_code = nrf_drv_pwm_init(&m_pwm, &config, NULL);
    APP_ERROR_CHECK(err_code);

    static nrf_pwm_values_individual_t seq_values;
    seq_values.channel_0 = red;
    seq_values.channel_1 = green;
    seq_values.channel_2 = blue;
    seq_values.channel_3 = 0;

    nrf_pwm_sequence_t const seq =
    {
            .values.p_individual = &seq_values,
            .length = NRF_PWM_VALUES_LENGTH(seq_values),
            .repeats = 0,
            .end_delay = 0
    };

    nrf_drv_pwm_simple_playback(&m_pwm, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);

    m_pwm_enabled = true;
}
