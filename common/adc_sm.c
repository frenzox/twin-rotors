#include "adc_sm.h"

#define ADC_SAMPLING_PERIOD	100

static TimerHandle_t adc_sampling_timer = NULL;

STATE(IDLE) {
	if(FIRST) {
		if(adc_sampling_timer)
			xTimerStop(adc_sampling_timer, 2);
		set_sampling_indicator(false);
	}

	if(is_sampling())
		NEXT_STATE(SAMPLING);
}

STATE(SAMPLING) {
	if(FIRST) {
		 if(!adc_sampling_timer)
		 	adc_sampling_timer = xTimerCreate(
		 		"ADC_TIMER",
		 		pdMS_TO_TICKS(ADC_SAMPLING_PERIOD),
		 		pdTRUE,
		 		NULL,
		 		adc_sample
		 	);
		xTimerStart(adc_sampling_timer, 2);
		set_sampling_indicator(true);
	}

	if(!is_sampling()) {
		NEXT_STATE(IDLE);
	}
}