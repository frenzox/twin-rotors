#ifndef PID_SM_H
#define PID_SM_H

#include "sm.h"
#include "hal.h"

StateMachine pid_sm;

STATE(IDLE);
STATE(CONTROLING);

#endif // PID_SM_H
