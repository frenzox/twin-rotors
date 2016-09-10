//
//  ble_main.h
//  TwinRotors
//
//  Created on 9/6/16.
//  Contributors:
//      Guilherme Felipe da Silva
//      André Luiz Luppi
//      Felipe Hartcopp Betoni
//

#ifndef BLE_MAIN_H
#define BLE_MAIN_H

#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble_rs.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

// Declare a service structure for the application
ble_rs_t m_rs;

void timers_init(void);
void buttons_leds_init(bool * erase_bonds);
void ble_stack_init(void);
void device_manager_init(bool erase_bonds);
void gap_params_init(void);
void services_init(void);
void advertising_init(void);
void conn_params_init(void);

#endif