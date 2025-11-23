#include "Init_Task.h"
WatchDog_TypeDef Gimbal_Dog[GIMBAL_SUM], Shoot_Dog[FRIC_SUM], Pluck_Dog, Down_Dog, PC_Dog,Referee_Dog;
void Init_Task(){
	taskENTER_CRITICAL();

	CanFilter_Init(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	CanFilter_Init(&hcan2);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

	remote_control_init();

	WatchDog_Init(&Remote_Dog, 20);
	WatchDog_Init(&IMU_Dog, 20);
	WatchDog_Init(&Gimbal_Dog[YAW], 10);
	WatchDog_Init(&Gimbal_Dog[PITCH], 10);
	WatchDog_Init(&Shoot_Dog[LEFT], 10); 
	WatchDog_Init(&Shoot_Dog[RIGHT], 10);
	WatchDog_Init(&Pluck_Dog, 10);
	WatchDog_Init(&Down_Dog, 50);
	WatchDog_Init(&PC_Dog, 100);
	WatchDog_Init(&Referee_Dog, 50);

	HAL_TIM_Base_Start_IT(&htim3); 

	xTaskCreate((TaskFunction_t)MainCtrl_Task,    "MainCtrl_Task",     256,   NULL, 7, &MainCtrl_Task_handle);
	xTaskCreate((TaskFunction_t)Shoot_Task,		   	"Shoot_Task",        256,   NULL, 4, &Shoot_Task_handle);		
	xTaskCreate((TaskFunction_t)usb_task,         "usb_task",          256 *2,NULL, 4, &usb_task_handle);				
	xTaskCreate((TaskFunction_t)Gimbal_Task,      "Gimbal_Task",       256,   NULL, 4, &Gimbal_Task_handle);
	xTaskCreate((TaskFunction_t)Plotter_Task,   	"Plotter_Task",			 128,   NULL, 4, &Plotter_Task_handle);
	xTaskCreate((TaskFunction_t)Chassis_Task,     "Chassis_Task",      256,   NULL, 4, &Chassis_Task_handle);
	xTaskCreate((TaskFunction_t)Music_Task,       "Music_Task",        256,   NULL, 4, &Music_Task_handle);

	taskEXIT_CRITICAL();
	vTaskDelete(NULL);
}
