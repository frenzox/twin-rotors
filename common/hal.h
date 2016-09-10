//
//  hal.h
//  TwinRotors
//
//  Created on 9/6/16.
//  Contributors:
//      Guilherme Felipe da Silva
//      André Luiz Luppi
//      Felipe Hartcopp Betoni
//

#ifndef HAL_H
#define HAL_H

#include "ble_main.h"

void ble_stack_task(void * vParm);
void adc_sample();
void main_rotor_value_update(uint16_t *value);
void tail_rotor_value_update(uint16_t *value);
void init_hal();
bool is_sampling();
void adc_button_toggle();
void set_sampling_indicator(bool on);

#endif //HAL_H
