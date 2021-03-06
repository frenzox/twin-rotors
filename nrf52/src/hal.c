//
//  hal.c
//  TwinRotors
//
//  Created on 9/6/16.
//      André Luiz Luppi
//      Felipe Hartcopp Betoni
//

#include "hal.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "app_pwm.h"
#include "SEGGER_RTT.h"


#define MAIN_ROTOR_ADC_CHANNEL              1
#define TAIL_ROTOR_ADC_CHANNEL              2
#define ADC_PRE_SCALING_COMPENSATION        6
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS       600
#define ADC_RES_10BIT                       1024
#define ADC_BUTTON                          16
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

static SemaphoreHandle_t              m_ble_event_ready;
static TaskHandle_t				      m_ble_stack_task;
static TimerHandle_t                  debounce_timer;
static TimerHandle_t                  pwm_duty_timer;
static uint32_t                        sine_count;
static bool                           sampling;
static volatile bool                  ready_flag;            // A flag indicating PWM status.

const uint8_t sin_table[] = {0, 0,1,2,4,6,9,12,16,20,24,29,35,40,   46, 53, 59, 66, 74, 81, 88, 96, 104,112,120,128,136,144,152,160,168,175,182,190,197,203,210,216,221,227,
               232,236,240,244,247,250,252,254,255,255,255,255,255,254,252,250,247,244,240,236,232,227,221,216,210,203,197,190,182,175,168,160,152,144,136,128,120,112,104,
               96,88,81,74,66,59,   53, 46, 40, 35, 29,24,  20, 16, 12, 9,  6,  4,  2,1,0};            

void ble_stack_task(void * vParm)
{   
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.js
    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    while (1)
    {
        /* Wait for event from SoftDevice */
        while(pdFALSE == xSemaphoreTake(m_ble_event_ready, portMAX_DELAY))
        {
            // Just wait again in the case when INCLUDE_vTaskSuspend is not enabled
        }

        intern_softdevice_events_execute();
        vTaskDelay(15);
    }
}

static void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

static void pwm_init() {
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(100, 5, 6); 
    
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    
    /* Initialize and enable PWM. */
    uint32_t err_code = app_pwm_init(&PWM1, &pwm1_cfg, pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    
}

static void update_duty() {
    sine_count = (sine_count+1) % 100;
    uint8_t duty = (sin_table[sine_count]*100/255);

    while(app_pwm_channel_duty_set(&PWM1, 0, duty) == NRF_ERROR_BUSY);
    while(app_pwm_channel_duty_set(&PWM1, 1, duty) == NRF_ERROR_BUSY);

}


static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    // Unused but needed fr SAADC config
    // Called when sampling finishes
}

static void adc_configure(void)
{
    uint32_t err_code;
    nrf_saadc_channel_config_t main_rotor_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    nrf_saadc_channel_config_t tail_rotor_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(MAIN_ROTOR_ADC_CHANNEL,&main_rotor_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(TAIL_ROTOR_ADC_CHANNEL,&tail_rotor_config);
    APP_ERROR_CHECK(err_code);
}


static void adc_button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    nrf_drv_gpiote_in_event_disable(ADC_BUTTON);
    adc_button_toggle();

    if(pdPASS != xTimerStart(debounce_timer, 2))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

static void gpiote_init(){
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    //in_config.hi_accuracy = true;

    err_code = nrf_drv_gpiote_in_init(ADC_BUTTON, &in_config, adc_button_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(ADC_BUTTON, true);
}

static void debounce() {
    xTimerStop(debounce_timer, 2);
    nrf_drv_gpiote_in_event_enable(ADC_BUTTON, true);
}


void adc_button_toggle(){
    sampling = !sampling;

    if(sampling) {
        sine_count = 0;
        app_pwm_enable(&PWM1);
        xTimerStart(pwm_duty_timer, 2);
    } else {
        app_pwm_disable(&PWM1);
        xTimerStop(pwm_duty_timer, 2);
    }
}

void adc_sample() {
    uint32_t err_code;
    static nrf_saadc_value_t main_rotor_value;
    static nrf_saadc_value_t tail_rotor_value;

    err_code = nrf_drv_saadc_sample_convert(MAIN_ROTOR_ADC_CHANNEL, &main_rotor_value);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_sample_convert(TAIL_ROTOR_ADC_CHANNEL, &tail_rotor_value);
    APP_ERROR_CHECK(err_code);

    main_rotor_value = ADC_RESULT_IN_MILLI_VOLTS(main_rotor_value);
    main_rotor_value_update((uint16_t*)&main_rotor_value);

    tail_rotor_value = ADC_RESULT_IN_MILLI_VOLTS(tail_rotor_value);
    tail_rotor_value_update((uint16_t*)&tail_rotor_value);
}

void main_rotor_value_update(uint16_t *value) {
    main_rotor_characteristic_update(&m_rs, value);
}

void tail_rotor_value_update(uint16_t *value) {
    tail_rotor_characteristic_update(&m_rs, value);
}

bool is_sampling() {
    return sampling;
}

void set_sampling_indicator(bool on) {
    if (on)
        nrf_gpio_pin_clear(20);
    else
        nrf_gpio_pin_set(20);
}

void init_hal() {

    sampling = false;
    ready_flag = false;
    debounce_timer = xTimerCreate("DEBOUNCE_TIMER", 300, pdTRUE, NULL, debounce);
    pwm_duty_timer = xTimerCreate("PWM_TIMER", 50, pdTRUE, NULL, update_duty);

    adc_configure();
    gpiote_init();
    pwm_init();
    
    // Init a semaphore for the BLE thread.
    m_ble_event_ready = xSemaphoreCreateBinary();
    if(NULL == m_ble_event_ready)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Start execution.
    if(pdPASS != xTaskCreate(ble_stack_task, "BLE", 256, NULL, 1, &m_ble_stack_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}
