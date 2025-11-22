#include "Aim_Task.h"

void Aim_Task(){
	static portTickType currentTime;		 
		for(;;){
			currentTime = xTaskGetTickCount();
			if(SystemState == SYSTEM_RUNNING) {
				/* ×ÔÃé»ð¿Ø */
				Aim_Shoot();
			}
			vTaskDelayUntil(&currentTime, 1);		 
	   }
}
