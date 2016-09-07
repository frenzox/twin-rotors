//
//  hal.c
//  TwinRotors
//
//  Created on 9/6/16.
//  Contributors:
//      Guilherme Felipe da Silva
//      André Luiz Luppi
//      Felipe Hartcopp Betoni
//

#include "hal.h"

static SemaphoreHandle_t        m_ble_event_ready;   // Semaphore raised if there is a new event to be processed in the BLE thread.
static TaskHandle_t				m_ble_stack_task;  // Bluetooth Low Energy Stack task

/**@brief Function for application main entry.
 */
void ble_stack_task(void * vParm)
{   
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
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

        // This function gets events from the SoftDevice and processes them by calling the function
        // registered by softdevice_ble_evt_handler_set during stack initialization.
        // In this code ble_evt_dispatch would be called for every event found.
        intern_softdevice_events_execute();
    }
}

void init_hal() {
	// Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

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