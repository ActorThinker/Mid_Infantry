#include "MainCtrl_Task.h"

void MainCtrl_Task(){
	static portTickType currentTime;
	for (;;){
    static uint8_t cnt=0;
		currentTime = xTaskGetTickCount();
		if(DeviceState.Remote_State != Device_Online){
			osThreadSuspend(Chassis_Task_handle);
			osThreadSuspend(Gimbal_Task_handle);
			osThreadSuspend(Shoot_Task_handle);
			RemoteClear();
      for(int i = 0;i<4;i++)Key_ch[i] = 0;
			SystemState = SYSTEM_STARTING;
			GimbalInitFlag = 0;
		} else {
			osThreadResume(Chassis_Task_handle);
			osThreadResume(Shoot_Task_handle);
			osThreadResume(Gimbal_Task_handle);
		}
		if(RC_CtrlData.key.Z){
		Gimbal_action.Key = 1;//Z键--Key=1刷新UI Ctrl&&Shift--Key=2解除功率限制
		}
	 	else if(RC_CtrlData.key.Ctrl){
		Gimbal_action.Key = 2;//Z键--Key=1刷新UI Ctrl&&Shift--Key=2解除功率限制	
		}else{
		Gimbal_action.Key = 0;//Z键--Key=1刷新UI Ctrl&&Shift--Key=2解除功率限制			
		}
	  switch(cnt++){
      case 0:CAN_Send_StdDataFrame(&hcan2, 0x120, (uint8_t *)&Gimbal_action);			 break;
      case 1:CAN_Send_StdDataFrame(&hcan2, 0x130, (uint8_t *)&Gimbal_data);cnt = 0;break;
    }
		WatchDog_Polling();
		vTaskDelayUntil(&currentTime, 15);
	}	   
}