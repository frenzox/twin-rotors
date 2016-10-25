#ifndef ADC_SM_H
#define ADC_SM_H

#include "sm.h"
#include "hal.h"

StateMachine sampling_sm;

STATE(IDLE);
STATE(SAMPLING);

#endif // ADC_SM_H