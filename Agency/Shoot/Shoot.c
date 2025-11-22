#include "Shoot.h"
#include "Time.h"
#include "Function.h"
#include "Time.h"
#include "Gimbal.h"

PID Shoot_Speed_PID[FRIC_SUM] = {{.Kp = 15, .Ki = 0, .Kd = 0, .limit = 5000},
								 {.Kp = 15, .Ki = 0, .Kd = 0, .limit = 5000}};

PID_Smis Pluck_Place_PIDS = {.Kp = 30, .Ki = 0, .Kd = - 0.8, .limit = 5000}; 
PID Pluck_Speed_PID 	  = {.Kp = 10, .Ki = 0, .Kd = 0, .limit = 5000};                   
PID Pluck_Continue_PID    = {.Kp = 20, .Ki = 0, .Kd = 0, .limit = 5000};               

int16_t Can1Send_Shoot[4] ={0};

struct SHOOT
{	
	int16_t Ref_3508[FRIC_SUM];
	int16_t Ref_2006;
	int16_t Ref_2006_Angle;
	float   Angle_DEG;
	enum{
		SHOOT_STOP = 0,
		SHOOT_READY = 1,
		SHOOT_NORMAL = 2,
		SHOOT_RUNNING = 3,
		SHOOT_STUCKING = 4,
	}Action;
}SHOOT;

RM3508_TypeDef Shoot_Motor[FRIC_SUM];
M2006_TypeDef  Pluck_Motor;

uint8_t Add_Angle_Flag = 0;
uint8_t Lose_Angle_Flag = 0;
uint8_t	Running_Flag = 0;
float 	Angle_Target = 0;
float 	RAMP_Angle_Target = 0;
uint16_t tim = 0;
uint16_t Stuck_time = 0;

void ShootCtrl_Decide(){
	   
		if(DeviceState.Remote_State == Device_Online){
		 RemoteMode == REMOTE_INPUT ? Shoot_Rc_Ctrl():
     RemoteMode == KEY_MOUSE_INPUT ? Shoot_Key_Ctrl() :
		 Shoot_Stop();
		}else Shoot_Close();
}
void Shoot_Rc_Ctrl(){
	if(GimbalCtrl != gAim){
#if RobotID == 0
    switch (RC_CtrlData.rc.s1){
        case 1:
							SHOOT.Action = SHOOT_RUNNING;
            break;
        case 3:
							SHOOT.Action = SHOOT_STOP;
            break;
        case 2:
							SHOOT.Action = SHOOT_STOP;
            break;
    }
#elif RobotID == 1
    switch (RC_CtrlData.rc.s1){
        case 2:
							SHOOT.Action = SHOOT_RUNNING;
            break;
        case 3:
							SHOOT.Action = SHOOT_STOP;
            break;
        case 1:
							SHOOT.Action = SHOOT_STOP;
            break;
    }
#endif
	}else{
		switch(Aim_Data.AimShoot){
			case AimFire:
							SHOOT.Action = SHOOT_RUNNING;
			break;
			case AimReady:
							SHOOT.Action = SHOOT_READY;
			break;
			case AimStop:
							SHOOT.Action = SHOOT_STOP;
			break;
   	}
	}
}

void Shoot_Key_Ctrl(){
    static char mouse_r_flag = 0;
    static uint16_t normal_time = 0,shoot_tim = 0;
	if(GimbalCtrl != gAim){
		if(RC_CtrlData.key.Q){
			shoot_tim = 0;
			SHOOT.Action = SHOOT_READY;						
		}							
		if(RC_CtrlData.mouse.press_l == 1){
			SHOOT.Action = SHOOT_READY;							
			shoot_tim = 0;
			SHOOT.Action = SHOOT_RUNNING;  
		}
		if(SHOOT.Action == SHOOT_RUNNING &&RC_CtrlData.mouse.press_l == 0){
			SHOOT.Action = SHOOT_READY;											   
		}
			shoot_tim ++;					
		if(SHOOT.Action != SHOOT_STUCKING && shoot_tim > 3000){
			SHOOT.Action = SHOOT_STOP;								
		}
	 }else {
		switch(Aim_Data.AimShoot){
			case AimFire:
				if(RC_CtrlData.mouse.press_l)
					SHOOT.Action = SHOOT_RUNNING;
				else SHOOT.Action = SHOOT_READY;
					break;
			case AimReady:
				SHOOT.Action = SHOOT_READY;
					break;
			case AimStop:
				SHOOT.Action = SHOOT_STOP;
					break;
   	}
	if(Gimbal.LastCtrl == gAim && GimbalCtrl != gAim) 
		SHOOT.Action = SHOOT_STOP;
	}
}
void Shoot_Stop(){
	SHOOT.Action = SHOOT_STOP;
		SHOOT.Ref_3508[LEFT]    =  0;
    SHOOT.Ref_3508[RIGHT]   =  0; 
		SHOOT.Ref_2006          =  0;
	PID_Control(Pluck_Motor.Speed, SHOOT.Ref_2006, &Pluck_Speed_PID);
	PID_Control(Shoot_Motor [LEFT].Speed, SHOOT.Ref_3508[LEFT], &Shoot_Speed_PID [LEFT]);
	PID_Control(Shoot_Motor [RIGHT].Speed, SHOOT.Ref_3508[RIGHT], &Shoot_Speed_PID [RIGHT]);
	limit(Pluck_Speed_PID.pid_out, PLUCK_SPEED, -PLUCK_SPEED);
    limit(Shoot_Speed_PID[LEFT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
    limit(Shoot_Speed_PID[RIGHT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
    Can1Send_Shoot[0] = (int16_t)Pluck_Speed_PID.pid_out;       
    Can1Send_Shoot[2] = (int16_t)Shoot_Speed_PID [LEFT].pid_out;
    Can1Send_Shoot[1] = (int16_t)Shoot_Speed_PID [RIGHT].pid_out; 
#if SHOOT_RUN
    MotorSend(&hcan1, 0X200, Can1Send_Shoot);
#endif
}
void Shoot_Close(){
     Can1Send_Shoot[0] = 0;       
     Can1Send_Shoot[1] = 0;
     Can1Send_Shoot[2] = 0; 

#if SHOOT_RUN
            MotorSend(&hcan1, 0X200, Can1Send_Shoot);
#endif

}
void Shoot_SendDown(){
	   if(DeviceState.Pluck_State != Device_Online || DeviceState.Shoot_State[LEFT] != Device_Online || DeviceState.Shoot_State[RIGHT] != Device_Online)
       Gimbal_action.shoot_status = shoot_offline;
		 else Gimbal_action.shoot_status = shoot_online;
		 if(SHOOT.Action == SHOOT_STOP)Gimbal_action.shoot_mode = shoot_mode_stop;
		 else if(SHOOT.Action == SHOOT_RUNNING && GimbalCtrl != gAim) Gimbal_action.shoot_mode = shoot_mode_fire;
		 else if(GimbalCtrl == gAim) Gimbal_action.shoot_mode = shoot_mode_follow;
		 if(SHOOT.Action == SHOOT_STUCKING)Gimbal_action.shoot_mode = shoot_mode_stucking;
		 

} 
void ShootRef_Set(){
	   
     SHOOT.Ref_3508[LEFT]    =  SHOOT_SPEED;
     SHOOT.Ref_3508[RIGHT]   = -SHOOT_SPEED; 
	   SHOOT.Angle_DEG         =  (Pluck_Motor.r * 8192 + Pluck_Motor.MchanicalAngle) * 0.0439453125f;	 
    switch (SHOOT.Action)
    {
    case SHOOT_STOP:
     Shoot_Stop();
		 Time.Single             =  0;
		 Angle_Target            = SHOOT.Angle_DEG;
     SHOOT.Ref_2006_Angle    = SHOOT.Angle_DEG;

     break;
    case SHOOT_READY:
     SHOOT.Ref_2006          =  0;
	   Add_Angle_Flag			     =  1;
		 Running_Flag            =  0;
		 Pluck_Motor.r           =  0;
		 Time.Single             =  0;
		 Angle_Target            = SHOOT.Angle_DEG;
     SHOOT.Ref_2006_Angle    = SHOOT.Angle_DEG;
        break;
    case SHOOT_NORMAL:
		if(Add_Angle_Flag == 1){

		 SHOOT.Ref_2006_Angle    =  SHOOT.Angle_DEG;
		 Angle_Target            = SHOOT.Angle_DEG + PLUCK_MOTOR_ONE *2;
			}
		 Add_Angle_Flag = 0;
			break;
    case SHOOT_RUNNING:

						if(Referee_data_Rx.game_state_robot_color / 10 == 1){
							break;
						}
				    else {
						pluck_speed =  4000;
						SHOOT.Ref_2006 =  4000;
						}


        break;
    case SHOOT_STUCKING:
		    SHOOT.Ref_2006 = -PLUCK_SPEED;
        break;
    }
}

void Shoot_Console(){
    if((SHOOT.Action == SHOOT_NORMAL)){
			if(SHOOT.Action == SHOOT_NORMAL)SHOOT.Ref_2006_Angle = RAMP_float (Angle_Target, SHOOT.Ref_2006_Angle, 100);
			PID_Control_Smis(SHOOT.Angle_DEG, SHOOT.Ref_2006_Angle, &Pluck_Place_PIDS, Pluck_Motor.Speed);
			SHOOT.Ref_2006 = Pluck_Place_PIDS.pid_out;
			 if(Time.Single > 300 ){
					SHOOT.Ref_2006          =  Pluck_Place_PIDS.pid_out;
					Time.Single             =  0;
				  Angle_Target            =  0;
			 }
			}
			PID_Control(Pluck_Motor.Speed, SHOOT.Ref_2006, &Pluck_Speed_PID);
			PID_Control(Shoot_Motor [LEFT].Speed, SHOOT.Ref_3508[LEFT], &Shoot_Speed_PID [LEFT]);
			PID_Control(Shoot_Motor [RIGHT].Speed, SHOOT.Ref_3508[RIGHT], &Shoot_Speed_PID [RIGHT]);
			limit(Pluck_Speed_PID.pid_out, M2006_LIMIT, -M2006_LIMIT);
			limit(Shoot_Speed_PID[LEFT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
			limit(Shoot_Speed_PID[RIGHT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
}
void Shoot_Send(){
        Can1Send_Shoot[0] = (int16_t)Pluck_Speed_PID.pid_out;       
        Can1Send_Shoot[2] = (int16_t)Shoot_Speed_PID [LEFT].pid_out;
        Can1Send_Shoot[1] = (int16_t)Shoot_Speed_PID [RIGHT].pid_out; 
#if SHOOT_RUN
	        if(SHOOT.Action != SHOOT_STOP){
            MotorSend(&hcan1, 0X200, Can1Send_Shoot);					
}

#endif
}
void Detect_Shoot(){
    float Angle_Target;
	  uint8_t Last_Action;
    static uint16_t  tim = 0;
    if(DeviceState.Pluck_State != Device_Online || DeviceState.Shoot_State[LEFT] != Device_Online || DeviceState.Shoot_State[RIGHT] != Device_Online)
         SHOOT.Action = SHOOT_STOP;
    if(SHOOT.Action != SHOOT_STOP){

				 if(SHOOT.Action == SHOOT_RUNNING){
         if(ABS( Pluck_Motor.Speed ) <= 15)
             Stuck_time++;
	 }
     if(Stuck_time >= 50){
         SHOOT.Action = SHOOT_STUCKING;
         Stuck_time++;
     }
		 if( Stuck_time > 100){
         Stuck_time = 0;
         SHOOT.Action = SHOOT_RUNNING;
		 }
	}

}
        float Recover,Remain,Consumption,shoot_speed,K = 2;
	      uint16_t ShootTime,shoot_time;
				uint8_t reduction_ratio = 72,caliper = 12;
void ShootHeat_Limit(){
	      Recover = (float)Referee_data_Rx.heat_limit_recover;
	      Remain  = (float)Referee_data_Rx.heat_limit_remain;
	      if(Remain > 2000){
				 Remain  = 0;
				}
				Consumption = 10.0;
//					if(Referee_data_Rx.game_state == 1){
								if(shoot_time == 0){
										ShootTime = (Remain + K * Recover) * 10;// 单位 /100ms
										
										//分级射速
									if(Remain > 100){
											shoot_speed = (15 * Remain - Recover - 5 * Consumption) / (Consumption * ShootTime/100.0) + Recover / Consumption;
									}else{
											shoot_speed = (15 * Remain - Recover - 4 * Consumption) / (Consumption * ShootTime / 100.0) + Recover / Consumption;
										}
										
								}
								else if(0 < shoot_time && Remain > 250 ){
								SHOOT.Ref_2006 = 9000;
								}
								else if(0 < shoot_time && shoot_time < ShootTime){
											SHOOT.Ref_2006 = shoot_speed * reduction_ratio * 60 / caliper ;//期望=期望弹频 * 减速比 * 60秒 / 拨尺数
								}else{
											SHOOT.Ref_2006 = Recover / Consumption * reduction_ratio * 60 / caliper;//期望 = 冷却速率 / 单发消耗热量 * 减速比 * 60秒 / 拨尺数
							
								}
								
								if(shoot_time < ShootTime){
									shoot_time++;
								}

								//未射击

								if(SHOOT.Action != SHOOT_RUNNING){
									shoot_time = 0;

									shoot_speed = 0;				
								}

	
//	}
					
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    uint16_t CAN1_ID = CAN_Receive_DataFrame(&hcan1, CAN1_buff);
    switch (CAN1_ID)
    {
        
        case 0x201: M2006_Receive(&Pluck_Motor, CAN1_buff);
                    Feed_Dog(&Pluck_Dog);
                    break;
        
        case 0x203: RM3508_Receive(&Shoot_Motor[LEFT], CAN1_buff); 
                    Feed_Dog(&Shoot_Dog[LEFT]);
                    break;   
        
        case 0x202: RM3508_Receive(&Shoot_Motor[RIGHT], CAN1_buff);
                    Feed_Dog(&Shoot_Dog[RIGHT]);
                    break; 
        default:    break;
    }
  }
}
