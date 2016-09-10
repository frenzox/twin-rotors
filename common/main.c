//
//  main.c
//  TwinRotors
//
//  Created on 9/6/16.
//  Contributors:
//      Guilherme Felipe da Silva
//      André Luiz Luppi
//      Felipe Hartcopp Betoni
//

#include "hal.h"
#include "sm.h"

#define ADC_SAMPLING_PERIOD	20

static TimerHandle_t	adc_sampling_timer;
static state_machine 	sampling_sm;

STATE(IDLE);
STATE(SAMPLING);

STATE(IDLE) {
	if(FIRST)
		set_sampling_indicator(false);
	if(is_sampling())
		NEXT_STATE(SAMPLING);
}

STATE(SAMPLING) {
	if(FIRST)
		set_sampling_indicator(true);
	if(!is_sampling())
		NEXT_STATE(IDLE);
}

void adc_sampling(TimerHandle_t xTimer) {
	if(COMPARE(sampling_sm, SAMPLING))
		adc_sample();
}

void start_timers() {
    if(pdPASS != xTimerStart(adc_sampling_timer, 2))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

void vApplicationIdleHook() {
	EXEC(sampling_sm);
}

int main(void) {
	adc_sampling_timer = xTimerCreate("ADC_TIMER", ADC_SAMPLING_PERIOD, pdTRUE, NULL, adc_sampling);

	init_hal();
	INIT(sampling_sm, IDLE);
	start_timers();
	vTaskStartScheduler();

	while(1){
		//Infinite loop
	};

	return 0;
}