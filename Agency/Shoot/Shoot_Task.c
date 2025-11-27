#include "Shoot_Task.h"



/*  TODO:增益补偿
    TODO:裁判系统反馈信息 弹速和热量
    TODO:
*/
void Shoot_Task(void *pvParameters)
{
	static portTickType currentTime;
	 
	for(;;)
	{
		currentTime = xTaskGetTickCount(); 
		ShootCtrl_Decide();
		Aim_Shoot();
		Detect_Shoot();
		ShootHeat_Limit();
		ShootRef_Set();
		Shoot_Console();
		Shoot_Send();
		Shoot_SendDown();
		
		vTaskDelayUntil (&currentTime,1);
	}
}
