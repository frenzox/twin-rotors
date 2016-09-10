//
//  ble_rs.c
//  TwinRotors
//
//  Created on 9/6/16.
//  Contributors:
//      Guilherme Felipe da Silva
//      André Luiz Luppi
//      Felipe Hartcopp Betoni
//

#include "ble_rs.h"

/**@brief Function for handling BLE GATTS EVENTS
 * 
 * This function prints out data that is received when you try to write to your characteristic or CCCD. 
 * In general it is a bad idea to to so much printf stuff and UART transfer inside the BLE handler,
 * but this is just for demonstrate purposes.
 *
 * @param[in]   p_rs        Our Service structure.
 * @param[in]   p_ble_evt            BLE event passed from BLE stack
 *
 */
static void on_ble_write(ble_rs_t * p_rs, ble_evt_t * p_ble_evt)
{
    // Decclare buffer variable to hold received data. The data can only be 32 bit long.
    uint32_t data_buffer;
    // Pupulate ble_gatts_value_t structure to hold received data and metadata.
    ble_gatts_value_t rx_data;
    rx_data.len = sizeof(uint32_t);
    rx_data.offset = 0;
    rx_data.p_value = (uint8_t*)&data_buffer;

    ble_rs_evt_t evt;
    memset(&evt,0,sizeof(evt));
    
    if(p_ble_evt->evt.gatts_evt.params.write.handle == p_rs->rs_main_rotor_control_handles.value_handle)
    {
        // Get data
        sd_ble_gatts_value_get(p_rs->conn_handle, p_rs->rs_main_rotor_control_handles.value_handle, &rx_data);
        evt.evt_type = BLE_RS_EVT_MAIN_ROTOR_CONTROL;
        evt.main_rotor_control.value = *((uint32_t*)rx_data.p_value);

    }
    else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_rs->rs_tail_rotor_control_handles.value_handle)
    {
        // Get data
        sd_ble_gatts_value_get(p_rs->conn_handle, p_rs->rs_tail_rotor_control_handles.value_handle, &rx_data);
        evt.evt_type = BLE_RS_EVT_TAIL_ROTOR_CONTROL;
        evt.tail_rotor_control.value = *((uint32_t*)rx_data.p_value);
    }

    p_rs->evt_handler(p_rs, &evt);
}

// ALREADY_DONE_FOR_YOU: Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_rs_on_ble_evt(ble_rs_t * p_rs, ble_evt_t * p_ble_evt)
{

    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->header.evt_id)
    {        
        case BLE_GATTS_EVT_WRITE:
            on_ble_write(p_rs, p_ble_evt);
            break;
        case BLE_GAP_EVT_CONNECTED:
            p_rs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_rs->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_rs        Our Service structure.
 *
 */
static uint32_t main_rotor_char_add(ble_rs_t * p_rs)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_RS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_MAIN_ROTOR_CHARACTERISTC_UUID;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 0;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 2;
    attr_char_value.init_len    = 2;
    uint8_t initial_value[2]    = {0x00, 0x00};
    attr_char_value.p_value     = initial_value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_rs->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_rs->rs_main_rotor_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_rs        Our Service structure.
 *
 */
static uint32_t tail_rotor_char_add(ble_rs_t * p_rs)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_RS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_TAIL_ROTOR_CHARACTERISTC_UUID;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 0;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 2;
    attr_char_value.init_len    = 2;
    uint8_t initial_value[2]    = {0x00, 0x00};
    attr_char_value.p_value     = initial_value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_rs->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_rs->rs_tail_rotor_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_rs        Our Service structure.
 *
 */
static uint32_t main_rotor_control_char_add(ble_rs_t * p_rs)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_RS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_MAIN_ROTOR_CONTROL_CHARACTERISTC_UUID;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;

    
    // // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    // ble_gatts_attr_md_t cccd_md;
    // memset(&cccd_md, 0, sizeof(cccd_md));
    // BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    // BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    // cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    // char_md.p_cccd_md           = &cccd_md;
    // char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 4;
    attr_char_value.init_len    = 4;
    uint8_t initial_value[4]    = {0x00, 0x00, 0x00, 0x00};
    attr_char_value.p_value     = initial_value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_rs->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_rs->rs_main_rotor_control_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_rs        Our Service structure.
 *
 */
static uint32_t tail_rotor_control_char_add(ble_rs_t * p_rs)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    // Add a custom characteristic UUID
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_RS_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_TAIL_ROTOR_CONTROL_CHARACTERISTC_UUID;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;

    // Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    
    // Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));        
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // Set characteristic length in number of bytes
    attr_char_value.max_len     = 4;
    attr_char_value.init_len    = 4;
    uint8_t initial_value[4]    = {0x00, 0x00, 0x00, 0x00};
    attr_char_value.p_value     = initial_value;

    // Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_rs->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_rs->rs_tail_rotor_control_handles);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_rs        Our Service structure.
 *
 */
void ble_rs_init(ble_rs_t * p_rs, ble_rs_evt_handler_t evt_handler)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    // Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_RS_BASE_UUID;
    service_uuid.uuid = BLE_UUID_RS_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    
    
    // Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    p_rs->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_rs->evt_handler = evt_handler;

    // Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_rs->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    // Call the function to add our new characteristic to the service. 
    main_rotor_char_add(p_rs);
    tail_rotor_char_add(p_rs);
    main_rotor_control_char_add(p_rs);
    tail_rotor_control_char_add(p_rs);

    nrf_gpio_pin_dir_set(18, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(19, NRF_GPIO_PIN_DIR_OUTPUT);
}

// Function to be called when updating characteristic value
void main_rotor_characteristic_update(ble_rs_t *p_rs, uint16_t *value)
{
    // Update characteristic value
    if (p_rs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t len      = 2;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_rs->rs_main_rotor_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)value;  

        sd_ble_gatts_hvx(p_rs->conn_handle, &hvx_params);
    }   
}

// Function to be called when updating characteristic value
void tail_rotor_characteristic_update(ble_rs_t *p_rs, uint16_t *value)
{
    // Update characteristic value
    if (p_rs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t      len = 2;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_rs->rs_tail_rotor_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)value;  

        sd_ble_gatts_hvx(p_rs->conn_handle, &hvx_params);
    }   
}
