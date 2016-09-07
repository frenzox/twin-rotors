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
void init_hal();

#endif //HAL_H
