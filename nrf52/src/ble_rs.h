//
//  ble_rs.h
//  TwinRotors
//
//  Created on 9/6/16.
//  Contributors:
//      Guilherme Felipe da Silva
//      André Luiz Luppi
//      Felipe Hartcopp Betoni
//

#ifndef BLE_RS_H
#define BLE_RS_H

#include <stdint.h>
#include <string.h>

#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "app_error.h"

// FROM_SERVICE_TUTORIAL: Defining 16-bit service and 128-bit base UUIDs
#define BLE_UUID_RS_BASE_UUID              {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define BLE_UUID_RS_SERVICE_UUID                0xAAA0

// ALREADY_DONE_FOR_YOU: Defining 16-bit characteristic UUID
#define BLE_UUID_MAIN_ROTOR_CHARACTERISTC_UUID                  0xAAA1
#define BLE_UUID_TAIL_ROTOR_CHARACTERISTC_UUID                  0xAAA2
#define BLE_UUID_MAIN_ROTOR_CONTROL_CHARACTERISTC_UUID          0xAAA3
#define BLE_UUID_TAIL_ROTOR_CONTROL_CHARACTERISTC_UUID          0xAAA4


/* @brief Data structure for [Main Rotor] characteristic */
typedef struct {
    uint16_t value;        // Which GPIO is being controlled, for now, only GPIO0 is valid option
} main_rotor_char_value_t;

/* @brief Data structure for [Tail Rotor] characteristic */
typedef struct {
	uint16_t value;
} tail_rotor_char_value_t;

/* @brief Data structure for [Main Rotor Control] characteristic */
typedef struct {
    uint32_t value;
} main_rotor_control_char_value_t;

/* @brief Data structure for [Tail Rotor Control] characteristic */
typedef struct {
    uint32_t value;
} tail_rotor_control_char_value_t;

/*@brief Service event type for Notification CCCD */
typedef enum
{
    BLE_RS_EVT_MAIN_ROTOR,
    BLE_RS_EVT_TAIL_ROTOR,
    BLE_RS_EVT_MAIN_ROTOR_CONTROL,
    BLE_RS_EVT_TAIL_ROTOR_CONTROL
} ble_rs_evt_type_t;

typedef struct {
    ble_rs_evt_type_t evt_type;
    main_rotor_char_value_t main_rotor;
    tail_rotor_char_value_t tail_rotor;
    main_rotor_control_char_value_t main_rotor_control;
    tail_rotor_control_char_value_t tail_rotor_control;
} ble_rs_evt_t;

// Forward declaration of ble_ios_t
typedef struct ble_rs_s ble_rs_t;

/*@brief IO service event handler type
 *@detail This handler will be created in application layer to handle
 *        events passed from the service if necessary
 */
typedef void (*ble_rs_evt_handler_t) (ble_rs_t* p_rs, ble_rs_evt_t* p_evt);


// This structure contains various status information for our service. 
// The name is based on the naming convention used in Nordics SDKs. 
// 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
// ‘os’ is short for Our Service). 
typedef struct ble_rs_s
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    // OUR_JOB: Step 2.D, Add handles for the characteristic attributes to our struct
    ble_rs_evt_handler_t evt_handler;
    ble_gatts_char_handles_t rs_main_rotor_handles;
    ble_gatts_char_handles_t rs_tail_rotor_handles;
    ble_gatts_char_handles_t rs_main_rotor_control_handles;
    ble_gatts_char_handles_t rs_tail_rotor_control_handles;
}ble_rs_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_rs       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_rs_on_ble_evt(ble_rs_t * p_rs, ble_evt_t * p_ble_evt);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_rs       Pointer to Our Service structure.
 */
void ble_rs_init(ble_rs_t * p_rs, ble_rs_evt_handler_t evt_handler);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_rs                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void main_rotor_characteristic_update(ble_rs_t *p_rs, uint16_t *value);
void tail_rotor_characteristic_update(ble_rs_t *p_rs, uint16_t *value);

#endif  /* BLE_RS_H */
