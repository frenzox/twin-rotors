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

int main(void) {
	init_hal();
	vTaskStartScheduler();

	while(1){
		//Infinite loop
	};

	return 0;
}