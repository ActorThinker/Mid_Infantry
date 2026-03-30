#include "Gimbal.h"
#include "Time.h"
#include "USB_Task.h"
#include "VT03.h"

eGimbal Gimbal;
eGimbalPidMode GimbalPidMode;
eGimbalCtrl GimbalCtrl;

PID_TypeDef Gimbal_Speed_pid_Yaw[GIMBAL_MODE];
PID_TypeDef Gimbal_Place_pid_Yaw[GIMBAL_MODE];
PID_TypeDef Gimbal_Speed_pid_Pitch[GIMBAL_MODE];
PID_TypeDef Gimbal_Place_pid_Pitch[GIMBAL_MODE];

int16_t Can2Send_Gimbal[4]={0};
int16_t Can2Send[4]={0};
uint8_t RecodeGimbal = 0;

void GimbalInit(){
	GimbalCtrl = gNormal;
	GimbalInitFlag = 1;
	RecodeGimbal =   0;
	Time.GimbalInit = 0;
	
	PID_init(&Gimbal_Place_pid_Pitch[INIT],25000,0,0,   10,0,0,0 );	
	PID_init(&Gimbal_Speed_pid_Pitch[INIT],25000,0,0,  	15,0,0,0 );
	PID_init(&Gimbal_Place_pid_Yaw[INIT],25000,0,0,     5,0,100,0 );	
	PID_init(&Gimbal_Speed_pid_Yaw[INIT],25000,0,0,  	  10,0,0,0 );

	PID_init(&Gimbal_Place_pid_Pitch[GYRO],25000,0,0,  10,0,100,0 );	
	PID_init(&Gimbal_Speed_pid_Pitch[GYRO],25000,0,0,  1000,0,10,0 );
	PID_init(&Gimbal_Place_pid_Yaw[GYRO],25000,0,0,  3,0,70,0 );	
	PID_init(&Gimbal_Speed_pid_Yaw[GYRO],25000,0,0,  -1000,0,0,0 );

	PID_init(&Gimbal_Speed_pid_Pitch[AIM],25000,0,0, 1200,0,10,0 );	
	PID_init(&Gimbal_Place_pid_Pitch[AIM],25000,0,0, 7,0,120,0 );	
	PID_init(&Gimbal_Place_pid_Yaw[AIM],25000,0,0,   6,0,100,0 );	
	PID_init(&Gimbal_Speed_pid_Yaw[AIM],25000,0,0,   -1250,0,0,0 );	
}
void GimbalCtrl_Decide(){
	if(DeviceState.Remote_State == Device_Online){		
		RemoteMode == REMOTE_INPUT ? Gimbal_RC_Ctrl():
		RemoteMode == KEY_MOUSE_INPUT ? Gimbal_Key_Ctrl() :
		Gimbal_Stop();
	}else Gimbal_Stop();
}
void Gimbal_RC_Ctrl(){
	switch (RC_CtrlData.rc.s1){
		case 1:
//			GimbalCtrl = gAim;
//			if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
//			GimbalPidMode = AIM;
							GimbalCtrl = gNormal;
					    if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
							Gimbal.Mode = Gyro;
							GimbalPidMode = GYRO;

			break;
		case 3:
			GimbalCtrl = gAim;
			if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
			GimbalPidMode = AIM;

//			GimbalCtrl = gNormal;
//			if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
//			Gimbal.Mode = Gyro;
//			GimbalPidMode = GYRO;
			break;
		case 2:
			if(GimbalCtrl != gNormal || Gimbal.Mode != Gyro) RecodeGimbal = 0;
			GimbalCtrl = gNormal;
			Gimbal.Mode = Gyro;
			GimbalPidMode = GYRO;
			break;
    }	
}
void Gimbal_Key_Ctrl(){
    static char Key_Q_flag = 0,Key_F_flag = 0;
    static float Speed_K = 0.5;
    static char mouse_r_flag = 0;
	
    if(GimbalCtrl != gAim){
        if (RC_CtrlData.key.F == 1 && Key_F_flag == 0){
			  Gimbal.Ref[Gyro].Yaw    += 180;
            Key_F_flag = 1;
        }
        if (RC_CtrlData.key.F == 0)
            Key_F_flag = 0;
					GimbalCtrl = gNormal;
					if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
					Gimbal.Mode = Gyro;
					GimbalPidMode = GYRO;
		}

        if(RC_CtrlData.mouse.press_r == 1 && mouse_r_flag == 0 ){
					    if(GimbalCtrl != gAim || Gimbal.Mode != Gyro) RecodeGimbal = 0;
							Gimbal.Mode = Gyro;
							GimbalPidMode = AIM;
							if(GimbalCtrl != gAim)GimbalCtrl = gAim;
					    else GimbalCtrl = gNormal;
					    mouse_r_flag = 1;
        }
        if (RC_CtrlData.mouse.press_r == 0)
            mouse_r_flag = 0;		
}
void GimbalRef_Update(){
	if(GimbalCtrl == gAim){
		float yaw_diff = 0;
		yaw_diff = ReceiveVisionData.data.Ref_Yaw - IMU.Angle_Yaw;
		if(yaw_diff > 180.0f){
			yaw_diff -= 360.0f;
		}else if (yaw_diff < -180.0f){
			yaw_diff += 360.0f;
		}
		IMU.VisionAngle = IMU.Angle_Yawcontinuous + yaw_diff;
	}
 	if(RecodeGimbal == 0){
		Gimbal.Ref[Gyro].Yaw 	=	IMU.Angle_Yawcontinuous;
		Gimbal.Ref[Gyro].Pitch 	=	IMU.Angle_Pitch;
		Gimbal.Ref[Mech].Yaw 	=	Gimbal.Angle[Mech].ContinuousYaw;
		Gimbal.Ref[Mech].Pitch 	= IMU.Angle_Pitch ;
		RecodeGimbal ++; 
	}
switch(GimbalCtrl){
	case gNormal:
		if(RemoteMode == REMOTE_INPUT){
			Gimbal.increase[YAW]   = Key_ch[2] * 0.3f;
			Gimbal.increase[PITCH] = Key_ch[3] * 0.1f;
		}else if(RemoteMode == KEY_MOUSE_INPUT){
			Gimbal.increase[YAW]    = Mouse_ch[0] * 0.25;
			Gimbal.increase[PITCH]  = Mouse_ch[1] * 0.2;
		}
		if(RC_CtrlData.key.E){
			Gimbal.increase[YAW]    *= 0.2;
			Gimbal.increase[PITCH]  *= 0.2;
		}
		Gimbal.Ref[Mech].Yaw 	+= 	Gimbal.increase[YAW];
		Gimbal.Ref[Gyro].Pitch 	-= 	Gimbal.increase[PITCH];
		Gimbal.Ref[Gyro].Yaw  	-= 	Gimbal.increase[YAW];	
	
		limit(Gimbal.Ref[Mech].Pitch,P_ADD_limit,P_LOSE_limit);		 
		break;
	case gAim:
		if(DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
			Gimbal.increase[YAW]   = 0;
			Gimbal.increase[PITCH] = 0;
			Gimbal.Ref[Gyro].Yaw   = IMU.VisionAngle;
			Gimbal.Ref[Gyro].Pitch = ReceiveVisionData.data.Ref_Pitch;
		} else {
			if(RemoteMode == REMOTE_INPUT){
				Gimbal.increase[YAW]   = Key_ch[2] * 0.1f;
				Gimbal.increase[PITCH] = Key_ch[3] * 0.1f;
			}
			else if(RemoteMode == KEY_MOUSE_INPUT){
				Gimbal.increase[YAW]    = Mouse_ch[0] * 0.1;
				Gimbal.increase[PITCH]  = Mouse_ch[1] * 0.02;
			}		
			Gimbal.Ref[Gyro].Yaw   -= (Gimbal.increase[YAW]);
			Gimbal.Ref[Gyro].Pitch -= (Gimbal.increase[PITCH]);
			Gimbal.Ref[Mech].Pitch -= (Gimbal.increase[PITCH]);
		}
		Gimbal.LastCtrl = gAim;
		break;
	default :
			Gimbal.Ref[Gyro].Yaw   = IMU.Angle_Yawcontinuous;
			Gimbal.Ref[Gyro].Pitch = Gimbal.Angle[Mech].Pitch;
			Gimbal.Ref[Mech].Pitch = Gimbal.Angle[Mech].Pitch;
		break;
  }
}
void GimbalReal_Update(){
	Gimbal.Angle[Gyro].Pitch           	= IMU.Angle_Pitch;
	Gimbal.Angle[Gyro].ContinuousYaw	  = IMU.Angle_Yawcontinuous;
	Gimbal.Speed[Gyro].Pitch            = IMU.Gyro_Pitch;
	Gimbal.Speed[Gyro].Yaw              = IMU.Gyro_Yaw;

	Gimbal.Angle[Mech].Pitch            = Gimbal_Motor[PITCH].Angle_DEG;
	Gimbal.Speed[Mech].Pitch            = IMU.Gyro_Pitch;
}	
void Gimbal_Stop(){
	Gimbal.Ref[Gyro].Yaw   = Gimbal.Angle[Gyro].ContinuousYaw;
	Gimbal.Ref[Gyro].Pitch = IMU.Angle_Pitch;
	Gimbal.Ref[Mech].Pitch = Gimbal.Angle[Mech].Pitch;
	Can2Send_Gimbal[YAW]   = 0;	
	Can2Send_Gimbal[PITCH] = 0;
	MotorSend(&hcan2, 0x1FF, Can2Send_Gimbal);
}
void Detect_Gimbal(){
	static uint16_t RefYaw,Yaw,RefPitch,Pitch;
	RefYaw    = Gimbal.Ref[Gyro].Yaw;
	Yaw       = Gimbal.Angle[Gyro].ContinuousYaw;
	RefPitch  = Gimbal.Ref[Gyro].Pitch;
	Pitch     = Gimbal.Angle[Gyro].Pitch;
	if(ABS(RefYaw - Yaw) > STD_Angle * 0.45f || ABS(RefPitch - Pitch) > STD_Angle * 0.45f){
		Gimbal_Stop();
	}
}
float Gimbal_Offset = -2.0f;
void Gimbal_Pid(){
	limit(ReceiveVisionData.data.Ref_Pitch,P_ADD_limit,P_LOSE_limit);	
	if (GimbalCtrl == gAim){
		if(ReceiveVisionData.data.dis > 0.1f && DeviceState.PC_State == 1){ 
			if(Gimbal_action.move_status == rotate){
			PID_Calc(&Gimbal_Place_pid_Yaw[AIM],IMU.Angle_Yawcontinuous,IMU.VisionAngle + Gimbal_Offset);
			PID_Calc(&Gimbal_Speed_pid_Yaw[AIM],IMU.Gyro_Yaw ,Gimbal_Place_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_Vyaw);			
			} else {
			PID_Calc(&Gimbal_Place_pid_Yaw[AIM],IMU.Angle_Yawcontinuous,IMU.VisionAngle);
			PID_Calc(&Gimbal_Speed_pid_Yaw[AIM],IMU.Gyro_Yaw ,Gimbal_Place_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_Vyaw);
			
			}
			PID_Calc(&Gimbal_Place_pid_Pitch[AIM],IMU.Angle_Pitch,ReceiveVisionData.data.Ref_Pitch);
			PID_Calc(&Gimbal_Speed_pid_Pitch[AIM],IMU.Gyro_Pitch ,Gimbal_Place_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_Vpitch);
		}else{
			PID_Calc(&Gimbal_Place_pid_Yaw[AIM],IMU.Angle_Yawcontinuous,Gimbal.Ref[Gyro].Yaw);
			PID_Calc(&Gimbal_Speed_pid_Yaw[AIM],IMU.Gyro_Yaw,Gimbal_Place_pid_Yaw[AIM].Output);
			
			PID_Calc(&Gimbal_Place_pid_Pitch[GYRO],IMU.Angle_Pitch,Gimbal.Ref[Gyro].Pitch);
			PID_Calc(&Gimbal_Speed_pid_Pitch[GYRO],IMU.Gyro_Pitch,Gimbal_Place_pid_Pitch[GYRO].Output);;
		}
   }else{
			PID_Calc(&Gimbal_Place_pid_Yaw[GYRO],IMU.Angle_Yawcontinuous,Gimbal.Ref[Gyro].Yaw);
			PID_Calc(&Gimbal_Speed_pid_Yaw[GYRO],IMU.Gyro_Yaw,Gimbal_Place_pid_Yaw[GYRO].Output);
		
			PID_Calc(&Gimbal_Place_pid_Pitch[GYRO],IMU.Angle_Pitch,Gimbal.Ref[Gyro].Pitch);
			PID_Calc(&Gimbal_Speed_pid_Pitch[GYRO],IMU.Gyro_Pitch,Gimbal_Place_pid_Pitch[GYRO].Output); 						
   }
}
void Gimbal_Send(){
	if(GimbalCtrl == gAim){
		if(ReceiveVisionData.data.dis > 0.1f && DeviceState.PC_State == 1){ 
			Can2Send_Gimbal[PITCH] = (int16_t)(Gimbal_Speed_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_aPitch);
			Can2Send_Gimbal[YAW]   = (int16_t)(Gimbal_Speed_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_aYaw);
		}else{
			Can2Send_Gimbal[PITCH] = (int16_t)(Gimbal_Speed_pid_Pitch[GYRO].Output);
			Can2Send_Gimbal[YAW]   = (int16_t)(Gimbal_Speed_pid_Yaw[AIM].Output);				
		}
	}else{
		Can2Send_Gimbal[PITCH] = (int16_t)(Gimbal_Speed_pid_Pitch[GYRO].Output);
		Can2Send_Gimbal[YAW]   = (int16_t)(Gimbal_Speed_pid_Yaw[GYRO].Output);	
	}
	limit(Can2Send_Gimbal[GIMBAL_SUM],GM6020_LIMIT,-GM6020_LIMIT);
#if GIMBAL_RUN
		 if(RemoteMode != STOP)
     MotorSend(&hcan2, 0x1FF, Can2Send_Gimbal);
#endif
}

void MedianInit(){
     static float Expect_PitchInit = 0;
     static float Expect_YawInit = 0;

     uint16_t Expect_YawRamp   = Gimbal_Motor[YAW].MchanicalAngle;
     uint16_t Expect_PitchRamp = Gimbal_Motor[PITCH].MchanicalAngle;

	if (Time.GimbalInit < 100){
#if   Yaw_Mid_Right < Yaw_Mid_Left
			if ((Gimbal_Motor[YAW].MchanicalAngle  <= Yaw_Mid_Left) && (Gimbal_Motor[YAW].MchanicalAngle  >= Yaw_Mid_Right))
#elif Yaw_Mid_Right > Yaw_Mid_Left
			if ((Gimbal_Motor[YAW].MchanicalAngle  <= Yaw_Mid_Left) || (Gimbal_Motor[YAW].MchanicalAngle  >= Yaw_Mid_Right))
#endif
				MidMode = FRONT;	else	MidMode = BACK;
	}else{
		if (MidMode == FRONT) 	Expect_YawInit = QuickCentering(Gimbal_Motor[YAW].MchanicalAngle,Yaw_Mid_Front);
				else	Expect_YawInit = QuickCentering(Gimbal_Motor[YAW].MchanicalAngle,Yaw_Mid_Back);

		Expect_YawRamp = RAMP_float(Expect_YawInit, Expect_YawRamp,200); 
		PID_Calc(&Gimbal_Place_pid_Yaw[INIT], Gimbal_Motor[YAW].MchanicalAngle, Expect_YawRamp);
		PID_Calc(&Gimbal_Speed_pid_Yaw[INIT], Gimbal_Motor[YAW].Speed, Gimbal_Place_pid_Yaw[INIT].Output);

		Expect_PitchInit = QuickCentering(Gimbal_Motor[PITCH].MchanicalAngle,Pitch_Mid);
		Expect_PitchRamp = RAMP_float(Expect_PitchInit, Expect_PitchRamp, 50); 
		PID_Calc(&Gimbal_Place_pid_Pitch[INIT], Gimbal_Motor[PITCH].MchanicalAngle, Expect_PitchRamp);
		PID_Calc(&Gimbal_Speed_pid_Pitch[INIT], Gimbal_Motor[PITCH].Speed,Gimbal_Place_pid_Pitch[INIT].Output);

		Can2Send[YAW]   = (int16_t)(Gimbal_Speed_pid_Yaw[INIT].Output);
		Can2Send[PITCH] = (int16_t)(Gimbal_Speed_pid_Pitch[INIT].Output);
		limit(Can2Send[GIMBAL_SUM],GM6020_LIMIT,-GM6020_LIMIT);
#if GIMBAL_RUN
        MotorSend(&hcan2, 0x1FF, Can2Send);
#endif
    }         
	if(Time.GimbalInit >= 1000){
		Time.GimbalInit = 0;
		GimbalInitFlag  = 0;

		Gimbal.Ref[Mech].Pitch           = Gimbal_Motor[PITCH].Angle_DEG;	
		Gimbal.Angle[Mech].Pitch         = Gimbal_Motor[PITCH].Angle_DEG;
		Gimbal.YawInit                   = Expect_YawInit;
		Gimbal.MidMode                   = MidMode;

		Gimbal.Angle[Gyro].ContinuousYaw = IMU.Angle_Yawcontinuous;
		Gimbal.Angle[Gyro].Pitch         = IMU.Angle_Pitch;
		Gimbal.Ref[Gyro].Pitch           = IMU.Angle_Pitch;
		Gimbal.Ref[Gyro].Yaw             = IMU.Angle_Yawcontinuous;		   
		Gimbal.increase[PITCH] = 0;
		Gimbal.increase[YAW]   = 0;

		SystemState = SYSTEM_RUNNING;
    }
}

void Gimbal_SendDown(){
	if(DeviceState.Gimbal_State[PITCH] == Device_Online) Gimbal_action.Gimbal_status.Pitch = Gimbal_online;
		else Gimbal_action.Gimbal_status.Pitch 	= 	Gimbal_offline;
	if( DeviceState.Gimbal_State[YAW]  == Device_Online)  Gimbal_action.Gimbal_status.Yaw  = Gimbal_online;
		else Gimbal_action.Gimbal_status.Yaw 	= 	Gimbal_offline; 
       
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if (hcan->Instance == CAN2){
    uint16_t CAN2_ID = CAN_Receive_DataFrame(&hcan2, CAN2_buff);
    switch (CAN2_ID){
        case 0x101:
						Referee_data_Rx.game_state = CAN2_buff[0];
						Referee_data_Rx.robot_color = CAN2_buff[1];
						Referee_data_Rx.heat_limit = (uint8_t)CAN2_buff[2];
						Referee_data_Rx.heat_cooling = (uint8_t)CAN2_buff[3];
						Referee_data_Rx.heat_now = (uint16_t)(CAN2_buff[4] << 8 | CAN2_buff[5]);
						Referee_data_Rx.bullet_speed  = (int16_t)(CAN2_buff[6] << 8 | CAN2_buff[7]);
                    Feed_Dog(&Down_Dog);
					break;
        case 0x205: GM6020_Receive( &Gimbal_Motor[PITCH], CAN2_buff); 
                    Feed_Dog(&Gimbal_Dog[PITCH]);
          break;
        case 0x206: GM6020_Receive( &Gimbal_Motor[YAW], CAN2_buff); 
                    Feed_Dog(&Gimbal_Dog[YAW]);
					break;
        default:    break;
    }
  }
}
