/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#define NRF_LOG_BACKEND_UART_ENABLED 0  
#define NRF_PWR_MGMT_CONFIG_CPU_USAGE_MONITOR_ENABLED 0
#define ENABLE_DEBUG_LOG_SUPPORT 0
#define RTC_ENABLED 1
#define DEVICE_NAME "Window"

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "app_timer.h"
#include "boards.h"

#include "nrf_delay.h"

#include "nrfx_rtc.h"
//#include "counter.h"

#define APP_TIMER_MAX_TIMERS 3

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_ADV_TIMEOUT_IN_SECONDS      10   
#define APP_ADV_INTERVAL                40      

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x4                             /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x3                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */


#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define PACKET_NAME                   'W'

#define BUTTON                          BSP_BUTTON_0   
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define REER_SWITCH_0 31

#define BTN 12
#define LED 14
#define PIN_OUT 13
#define PIN_IN 11
#define EVENTS_PER_TIME_LIMIT 1000//3
#define TIME_LIMIT_FOR_EVENTS 36000000 // 1h
#define TIME_LIMIT_FOR_STILL_ALIVE 1000//86400000 // 1D
#define NUMBER_OF_REPEATS 3

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

struct Timer_context_st {
    app_timer_id_t  timer_id;
    unsigned repeat;
} Timer_context;

APP_TIMER_DEF(Sip_timer);
APP_TIMER_DEF(Clear_event_timer);
APP_TIMER_DEF(Still_alive_event_timer);

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};

static bool state_change = 1;
static unsigned event_counter = 0;
static bool last_state = 0;

const uint8_t Sensor_ID = 1;
const uint8_t Window_open = 0;

uint8_t m_beacon_info[4] =                    /**< Information advertised by the Beacon. */
/*{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};*/

{
    PACKET_NAME, Sensor_ID, Window_open, 0
};



/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = 0xFFFF;

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    ble_gap_conn_sec_mode_t sec_mode;
    sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_FULL_NAME;//BLE_ADVDATA_NO_NAME;

    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 100; // 100 ms

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    //sd_ble_gap_tx_power_set(0);

    /*err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);*/
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


void clear_event_counter(void * p_context){
    event_counter = 0;
    NRF_LOG_INFO("Clear");
    if (last_state != nrf_gpio_pin_read(REER_SWITCH_0)){
        
        event_counter += 1;
        last_state = nrf_gpio_pin_read(REER_SWITCH_0);
        if (nrf_gpio_pin_read(REER_SWITCH_0)){
            m_beacon_info[3] = 1;
        }else{
            m_beacon_info[3] = 0;
        }
        sd_ble_gap_adv_stop(m_adv_handle);
        advertising_init();
        advertising_start();
    }
    
}

void still_alive_hendlar(void* p_context)
{       
    if (event_counter < EVENTS_PER_TIME_LIMIT){
    event_counter += 1;
    last_state = nrf_gpio_pin_read(REER_SWITCH_0);
    if (nrf_gpio_pin_read(REER_SWITCH_0)){
        m_beacon_info[3] = 1;
    }else{
        m_beacon_info[3] = 0;
    }
    sd_ble_gap_adv_stop(m_adv_handle);
    advertising_init();
    advertising_start();
    }
      
}

void send_add(void * p_context)
{
    struct Timer_context_st * pt_context =  p_context;

    if (pt_context->repeat){
        pt_context->repeat = pt_context->repeat-1;
        send_add(p_context);
    }
    last_state = nrf_gpio_pin_read(REER_SWITCH_0);
    if (nrf_gpio_pin_read(REER_SWITCH_0)){
        m_beacon_info[3] = 1;
    }else{
        m_beacon_info[3] = 0;
    }
    sd_ble_gap_adv_stop(m_adv_handle);
    advertising_init();
    advertising_start();

    ret_code_t err_code;
    err_code = app_timer_stop(pt_context->timer_id);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&Sip_timer, APP_TIMER_MODE_REPEATED, send_add);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&Clear_event_timer,APP_TIMER_MODE_REPEATED, clear_event_counter);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&Still_alive_event_timer,APP_TIMER_MODE_REPEATED, still_alive_hendlar);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    //sd_app_evt_wait();
    if (NRF_LOG_PROCESS() == false)
    {
        //nrf_log_backend_disable(&uart_log_backend);
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
/*static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    switch (pin_no)
    {
        case BUTTON:
            NRF_LOG_INFO("Send button state change.");
            break;
        default:
            APP_ERROR_HANDLER(pin_no);
            //NRF_LOG_INFO("1");
            break;
    }
}*/

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    state_change = 1;
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(REER_SWITCH_0, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(REER_SWITCH_0, true);
}


/**@brief Function for initializing the button handler module.
 */
/*static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}*/

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timers_init();
    gpio_init();
    //counter_init();
    //buttons_init();
    power_management_init();
    ble_stack_init();
    advertising_init();
   
    //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    // Start 32 kHz clock
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    // Start execution.
    NRF_LOG_INFO("Beacon example started.");
    state_change = 0;
    // Enter main loop.
    
    ret_code_t err_code;
    err_code = app_timer_start(Clear_event_timer, APP_TIMER_TICKS(TIME_LIMIT_FOR_EVENTS), NULL); //1000 ms= 1 s
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(Still_alive_event_timer, APP_TIMER_TICKS(TIME_LIMIT_FOR_STILL_ALIVE), NULL); //1000 ms= 1 s
    APP_ERROR_CHECK(err_code);
    

    //nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    for (;; )
    {   
        if(state_change){
            
            if (event_counter < EVENTS_PER_TIME_LIMIT){
                //ret_code_t err_code;
                struct Timer_context_st context = {Sip_timer,3};
                event_counter += 1;
                err_code = app_timer_start(Sip_timer, APP_TIMER_TICKS(500), &context); //1000 ms= 1 s
                APP_ERROR_CHECK(err_code);
            }
            
            state_change = 0;
        }else{
            idle_state_handle();
        }
    }
}


/**
 * @}
 */
