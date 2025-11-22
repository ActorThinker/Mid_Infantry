#include "Gimbal_Task.h"

void Gimbal_Task(){
	static portTickType currentTime;

	PID_init(&Gimbal_Place_pid_Yaw[INIT],2.0f*Pi,2.0f*Pi,0,     0,0,0,0,		0.001,0,0,0,0,Integral_Limit);	
	PID_init(&Gimbal_Speed_pid_Yaw[INIT],				3,3,0,  	    	0,0,0,0,		0.001,0,0,0,0,Integral_Limit);
	PID_init(&Gimbal_Speed_pid_Pitch[INIT],2.0f*Pi,2.0f*Pi,0,   0,0,0,0,		0.001,0,0,0,0,Integral_Limit);	
	PID_init(&Gimbal_Place_pid_Pitch[INIT],			3,3,0,  				0,0,0,0,		0.001,0,0,0,0,Integral_Limit);
	for(;;){
		currentTime = xTaskGetTickCount();			 
		if(SystemState != SYSTEM_RUNNING){
			if(GimbalInitFlag == 0) GimbalInit();
			MedianInit();
#if !GIMBAL_RUN
            SystemState = SYSTEM_RUNNING;
#endif				
		}else{
			GimbalCtrl_Decide();
			GimbalRef_Update();
			GimbalReal_Update();
			Gimbal_Pid();	
			Gimbal_Send();				
			Gimbal_SendDown();

//			Serialport[0] = IMU.Angle_Yaw;
//			Serialport[0] = Gimbal.Angle[Gyro].ContinuousYaw;
//			Serialport[1] = Gimbal.Ref[Gyro].Yaw ;
//			Serialport[0] = IMU.Angle_Pitch ;
//			Serialport[1] = Gimbal.Ref[Mech].Pitch;
			}
        vTaskDelayUntil(&currentTime, 1);		 
		}
}
