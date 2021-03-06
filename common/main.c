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

#include "adc_sm.h"

void state_machine_task(void *pParam){
        while(1){
	        EXEC(sampling_sm);
                vTaskDelay(20);
	}
}

int main(void) {

	init_hal();
	INIT(sampling_sm, IDLE);
	xTaskCreate(state_machine_task, "SM_TASK", 50, NULL, 1, NULL);
	vTaskStartScheduler();

	while(1){};

	return 0;
}
