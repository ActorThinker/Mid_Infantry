#include "Serialport.h"
#include "Gimbal.h"
void Serial_Send(void);
void SerialportSend();
void Vofa_JustFloat(float *Data, uint8_t Num);
unsigned char temp_end[4] = {0, 0, 0x80, 0x7F};

float a=1;

void Serialport_Task(){
     static portTickType currentTime;		 
	   for(;;){
		   currentTime = xTaskGetTickCount();
		   
            vTaskSuspendAll();	
            SerialportSend();		   
            xTaskResumeAll();

            vTaskDelayUntil(&currentTime, 1000);		 
	   }
}
void SerialportSend(){
		printf("%.6f,%.6f\n",Serialport[0],Serialport[1]);
}
