#include "Gimbal.h"
#include "Time.h"
#include "usb_task.h"

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
	Gimbal.Mode = Gyro;
	GimbalCtrl = gNormal;
	GimbalInitFlag = 1;
	RecodeGimbal =   0;
	Time.GimbalInit = 0;
	
	PID_init(&Gimbal_Speed_pid_Pitch[INIT],2.0f*Pi,2.0f*Pi,0,   0,0,0,0,		0.001,0,0,0,0,Integral_Limit);	
	PID_init(&Gimbal_Place_pid_Pitch[INIT],			3,3,0,  				0,0,0,0,		0.001,0,0,0,0,Integral_Limit);
	PID_init(&Gimbal_Place_pid_Yaw[INIT],2.0f*Pi,2.0f*Pi,0,     0,0,0,0,		0.001,0,0,0,0,Integral_Limit);	
	PID_init(&Gimbal_Speed_pid_Yaw[INIT],				3,3,0,  	    	0,0,0,0,		0.001,0,0,0,0,Integral_Limit);

	PID_init(&Gimbal_Speed_pid_Pitch[MECH],6.0f*Pi,6.0f*Pi,0, 0.5,0,0,0,  0.001,0,0,0,0,Integral_Limit&&ChangingIntegralRate);	
	PID_init(&Gimbal_Place_pid_Pitch[MECH],			3,3,0,   		  0.5,0,0,0,  0.001,0,0,0,0,Integral_Limit&&ChangingIntegralRate);
	PID_init(&Gimbal_Place_pid_Yaw[GYRO],2.0f*Pi,2.0f*Pi,0,     1 ,0,0,0,  0.001,0,0,0,0,Integral_Limit);	
	PID_init(&Gimbal_Speed_pid_Yaw[GYRO],			  3,3,0,  			-1.5,0,0,0,  0.001,0,0,0,0,Integral_Limit);

	PID_init(&Gimbal_Place_pid_Pitch[AIM],6.0f*Pi,6.0f*Pi,0, 	0.5,0,0,0,  0.001,0,0,0,0,Integral_Limit);	
	PID_init(&Gimbal_Speed_pid_Pitch[AIM],			3,1.5f,0,  		0.5,0,0,0,  0.001,0,0,0,0,Integral_Limit);	
	PID_init(&Gimbal_Place_pid_Yaw[AIM], 2.0f*Pi,2.0f*Pi,0,     1 ,0,0,0,  0.001,0,0,0,0,Integral_Limit);	
	PID_init(&Gimbal_Speed_pid_Yaw[AIM],				3,0.7f,0,   	-1.5,0,0,0,  0.001,0,0,0,0,Integral_Limit);	
}

void GimbalCtrl_Decide(){
	if(DeviceState.Remote_State == Device_Online){
		RemoteMode == REMOTE_INPUT ? Gimbal_RC_Ctrl():
		RemoteMode == KEY_MOUSE_INPUT ? Gimbal_Key_Ctrl() :
		Gimbal_Stop();
	}else Gimbal_Stop();
}

float PitchOffset;

void GimbalRef_Update(){

	if(RecodeGimbal == 0){
		Gimbal.Ref[Gyro].Yaw 	=	IMU.Angle_Yawcontinuous;
		Gimbal.Ref[Gyro].Pitch 	=	IMU.Angle_Pitch;
		Gimbal.Ref[Mech].Yaw 	=	Gimbal.Angle[Mech].ContinuousYaw;
		Gimbal.Ref[Mech].Pitch 	=	Gimbal.Angle[Mech].Pitch;
		PitchOffset = 0;
		if(Gimbal.Mode == gAim){
		Gimbal.Ref[Mech].Pitch = Gimbal.Angle[Mech].Pitch;
	}else{
		Gimbal.Ref[Gyro].Pitch = IMU.Angle_Pitch;	 
	 }
		RecodeGimbal ++; 
		}
switch(GimbalCtrl){
	case gNormal:
		if(Gimbal.Mode == Gyro){
		if(RemoteMode == REMOTE_INPUT){
			Gimbal.increase[YAW]   = Key_ch[2] * 0.3f;
			Gimbal.increase[PITCH] = Key_ch[3] * 0.1f;
		}else if(RemoteMode == KEY_MOUSE_INPUT){
			Gimbal.increase[YAW]    = Mouse_ch[0] * 0.25;
			Gimbal.increase[PITCH]  = Mouse_ch[1] * 0.2;
			}
		}
		else if(Gimbal.Mode == Mech){
		if(RemoteMode == REMOTE_INPUT){
			Gimbal.increase[YAW]   = Key_ch[2] * 1.5;
			Gimbal.increase[PITCH] = Key_ch[3] * 0.02f;		
		}else if(RemoteMode == KEY_MOUSE_INPUT){
			Gimbal.increase[YAW]    = Mouse_ch[0] * 8.0;
			Gimbal.increase[PITCH]  = Mouse_ch[1] * 0.1;	
			
			}
		}
		if(RC_CtrlData.key.E){
			Gimbal.increase[YAW]    *= 0.2;
			Gimbal.increase[PITCH]  *= 0.2;
		}
		Gimbal.Ref[Mech].Yaw 	+= 	Gimbal.increase[YAW];
		Gimbal.Ref[Mech].Pitch 	-= 	Gimbal.increase[PITCH];
		Gimbal.Ref[Gyro].Yaw  	-= 	Gimbal.increase[YAW];	
	
		limit(Gimbal.Ref[Mech].Pitch,P_ADD_limit,P_LOSE_limit);		 
	break;
	case gAim:
		if(Gimbal.Mode == Gyro && ReceiveVisionData.data.dis != -1){
			PitchOffset -= Mouse_ch[1] * 0.01;
			Gimbal.Ref[Gyro].Yaw   = ReceiveVisionData.data.Ref_Yaw;
			Gimbal.Ref[Gyro].Pitch = ReceiveVisionData.data.Ref_Pitch;   		
			Gimbal.increase[YAW]   = 0;
			Gimbal.increase[PITCH] = 0;
		}else{
			if(RemoteMode == REMOTE_INPUT){
				Gimbal.increase[YAW]   = Key_ch[2] * 0.1f;
				Gimbal.increase[PITCH] = Key_ch[3] * 0.1f;
			}
			else if(RemoteMode == KEY_MOUSE_INPUT){
				Gimbal.increase[YAW]    = Mouse_ch[0] * 0.1;
				Gimbal.increase[PITCH]  = Mouse_ch[1] * 0.02;
		}
		Gimbal.Ref[Gyro].Yaw   -= Gimbal.increase[YAW];
		Gimbal.Ref[Gyro].Pitch -= Gimbal.increase[PITCH];
		Gimbal.Ref[Mech].Pitch -= Gimbal.increase[PITCH];
		}
		Gimbal.LastCtrl = gAim;
	break;
	case gFllow:
		Gimbal.increase[YAW]   = 0;
		Gimbal.increase[PITCH] = Key_ch[3] * 0.1f;
		limit(Gimbal.Ref[Mech].Pitch,P_ADD_limit,P_LOSE_limit);		 
	break;
	
	case gTest:
		Gimbal.increase[YAW]   	= 0;
		Gimbal.increase[PITCH] 	= 0;
		Gimbal.Ref[Gyro].Yaw  	+= 0.1;
		if(Gimbal.Ref[Gyro].Yaw >= 51)	Gimbal.Ref[Gyro].Yaw = 0;
	break;

	default :
			Gimbal.Ref[Gyro].Yaw   = 0;
			Gimbal.Ref[Gyro].Pitch = 0 + PitchOffset;   		
	
	break;
  }
}

void GimbalReal_Update(){
			Gimbal.Angle[Gyro].Pitch           	= IMU.Angle_Pitch;
			Gimbal.Angle[Gyro].ContinuousYaw	  = IMU.Angle_Yawcontinuous;
			Gimbal.Speed[Gyro].Pitch            = IMU.Gyro_Pitch;
			Gimbal.Speed[Gyro].Yaw              = IMU.Gyro_Yaw;

			Gimbal.Angle[Mech].ContinuousYaw    = Gimbal_Motor[YAW].Angle;
			Gimbal.Angle[Mech].Yaw              = Gimbal_Motor[YAW].MchanicalAngle;
			Gimbal.Speed[Mech].Yaw              = Gimbal_Motor[YAW].Speed;
			Gimbal.Angle[Mech].Pitch            = Gimbal_Motor[PITCH].Angle_DEG;
			Gimbal.Speed[Mech].Pitch            = IMU.Gyro_Pitch;
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
        //			if(NormalModeFlag != 0 && GimbalCtrl != gAim && GyroscopeModeFlag != 1){
//			   if(Gimbal.Mode != Mech) RecodeGimbal = 0;
//					GimbalCtrl = gNormal;
//					Gimbal.Mode = Mech;
//					GimbalPidMode = MECH;
//			}else {
					GimbalCtrl = gNormal;
					if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
					Gimbal.Mode = Gyro;
					GimbalPidMode = GYRO;
//      }
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

void Gimbal_RC_Ctrl(){
	switch (RC_CtrlData.rc.s1){
		case 2:
			GimbalCtrl = gAim;
			if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
			GimbalPidMode = AIM;
			GimbalCtrl = gAim;
//							GimbalCtrl = gNormal;
//					    if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
//							Gimbal.Mode = Gyro;
//							GimbalPidMode = GYRO;

		break;
		case 3:

			GimbalCtrl = gNormal;
			if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
			Gimbal.Mode = Gyro;
			GimbalPidMode = GYRO;

//							GimbalCtrl = gNormal
//				    	if(Gimbal.Mode != Mech) RecodeGimbal = 0;;
//							Gimbal.Mode = Mech;
//							GimbalPidMode = MECH;
		break;
		case 1:
//				        if(Gimbal.Mode != Mech) RecodeGimbal = 0;
//							GimbalCtrl = gNormal;
//							Gimbal.Mode = Mech;
//							GimbalPidMode = MECH;

			if(GimbalCtrl != gNormal || Gimbal.Mode != Gyro) RecodeGimbal = 0;
			GimbalCtrl = gNormal;
			Gimbal.Mode = Gyro;
			GimbalPidMode = GYRO;
		break;
    }	
}	

void Gimbal_Stop(){
	Gimbal.Ref[Gyro].Yaw   = Gimbal.Angle[Gyro].ContinuousYaw;
	Gimbal.Ref[Gyro].Pitch = Gimbal.Angle[Gyro].Pitch;
	Gimbal.Ref[Mech].Yaw   = Gimbal.Angle[Mech].ContinuousYaw;
	Gimbal.Ref[Mech].Pitch = Gimbal.Angle[Mech].Pitch;
	Can2Send_Gimbal[YAW]   = 0;	
	Can2Send_Gimbal[PITCH] = 0;

	MotorSend(&hcan2, 0x1FE, Can2Send_Gimbal);
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

void Gimbal_Pid(){
	if (GimbalCtrl == gAim){
		if(ReceiveVisionData.data.dis != -1){
			PID_Calc(&Gimbal_Place_pid_Yaw[AIM],IMU.Angle_Yaw,ReceiveVisionData.data.Ref_Yaw);
			PID_Calc(&Gimbal_Speed_pid_Yaw[AIM],IMU.Gyro_Yaw ,Gimbal_Place_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_Vyaw);

			PID_Calc(&Gimbal_Place_pid_Pitch[AIM],IMU.Angle_Pitch,ReceiveVisionData.data.Ref_Pitch);
			PID_Calc(&Gimbal_Speed_pid_Pitch[AIM],IMU.Gyro_Pitch ,Gimbal_Place_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_Vpitch);
			}else{
			PID_Calc(&Gimbal_Place_pid_Yaw[AIM],Gimbal.Angle[Gyro].ContinuousYaw,Gimbal.Ref[Gyro].Yaw);
			PID_Calc(&Gimbal_Speed_pid_Yaw[AIM],Gimbal.Speed[Gyro].Yaw,Gimbal_Place_pid_Yaw[AIM].Output);
			
			PID_Calc(&Gimbal_Place_pid_Pitch[AIM],Gimbal.Angle[Mech].Pitch,Gimbal.Ref[Mech].Pitch);
			PID_Calc(&Gimbal_Speed_pid_Pitch[AIM],Gimbal.Speed[Mech].Pitch,Gimbal_Place_pid_Pitch[AIM].Output);;
			}
    }else{
			PID_Calc(&Gimbal_Place_pid_Yaw[GYRO],Gimbal.Angle[Gyro].ContinuousYaw,Gimbal.Ref[Gyro].Yaw);
			PID_Calc(&Gimbal_Speed_pid_Yaw[GYRO],Gimbal.Speed[Gyro].Yaw,Gimbal_Place_pid_Yaw[GYRO].Output);
		
			PID_Calc(&Gimbal_Place_pid_Pitch[MECH],Gimbal.Angle[Mech].Pitch,Gimbal.Ref[Mech].Pitch);
			PID_Calc(&Gimbal_Speed_pid_Pitch[MECH],Gimbal.Speed[Mech].Pitch,Gimbal_Place_pid_Pitch[MECH].Output); 						
    }
}

void Gimbal_Send(){
	if(GimbalCtrl == gAim){
		Can2Send_Gimbal[YAW]   = ((int16_t)((Gimbal_Speed_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_aYaw)	* current_to_out));
		Can2Send_Gimbal[PITCH] = ((int16_t)((Gimbal_Speed_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_aPitch)* current_to_out));
	}else{
		Can2Send_Gimbal[PITCH] = (int16_t)(Gimbal_Speed_pid_Pitch[MECH].Output * current_to_out);
		Can2Send_Gimbal[YAW]   = (int16_t)(Gimbal_Speed_pid_Yaw[GYRO].Output * current_to_out);	
	}
	limit(Can2Send_Gimbal[GIMBAL_SUM],GM6020_Limit,-GM6020_Limit);
#if GIMBAL_RUN
		 if(RemoteMode != STOP)
     MotorSend(&hcan2, 0x1FE, Can2Send_Gimbal);
#endif
}

void MedianInit(){
		int16_t Can2Send[4] = {0};
    static int16_t Expect_PitchInit = 0.0f;
    static int16_t Expect_YawInit   = 0.0f;
    static float   Expect_YawRamp   = 0.0f;
    static float   Expect_PitchRamp = 0.0f;

    uint16_t CurrentYaw   = Gimbal_Motor[YAW].MchanicalAngle;  
    uint16_t CurrentPitch = Gimbal_Motor[PITCH].MchanicalAngle;

    if (Time.GimbalInit < 100){
#if   Yaw_Mid_Right < Yaw_Mid_Left
        if ((CurrentYaw <= Yaw_Mid_Left) && (CurrentYaw >= Yaw_Mid_Right))
#elif Yaw_Mid_Right > Yaw_Mid_Left
        if ((CurrentYaw <= Yaw_Mid_Left) || (CurrentYaw >= Yaw_Mid_Right))
#endif
            MidMode = FRONT;
        else
            MidMode = BACK;
    } else {
        if (MidMode == FRONT) 	Expect_YawInit = QuickCentering(CurrentYaw, Yaw_Mid_Front);
						else	Expect_YawInit = QuickCentering(CurrentYaw, Yaw_Mid_Back);

        Expect_YawRamp = RAMP_float((float)Expect_YawInit, Expect_YawRamp, 200); 
        PID_Calc(&Gimbal_Place_pid_Yaw[INIT], CurrentYaw, Expect_YawRamp);
        PID_Calc(&Gimbal_Speed_pid_Yaw[INIT], Gimbal_Motor[YAW].Speed, Gimbal_Place_pid_Yaw[INIT].Output);

        Expect_PitchInit = QuickCentering(CurrentPitch, Pitch_Mid);
        Expect_PitchRamp = RAMP_float((float)Expect_PitchInit, Expect_PitchRamp, 50); 
        PID_Calc(&Gimbal_Place_pid_Pitch[INIT], CurrentPitch, Expect_PitchRamp);
        PID_Calc(&Gimbal_Speed_pid_Pitch[INIT], Gimbal.Speed[Mech].Pitch, Gimbal_Place_pid_Pitch[INIT].Output);

        Can2Send[YAW]   = (int16_t)(Gimbal_Speed_pid_Yaw[INIT].Output  * current_to_out);
        Can2Send[PITCH] = (int16_t)(Gimbal_Speed_pid_Pitch[INIT].Output* current_to_out);
		
		limit(Can2Send[GIMBAL_SUM],GM6020_Limit,-GM6020_Limit);

#if GIMBAL_RUN
        MotorSend(&hcan2, 0x1FE, Can2Send);
#endif
    }         

    if(Time.GimbalInit >= 1000){
        Time.GimbalInit = 0;
        GimbalInitFlag  = 0;

        // 初始化参考
        Gimbal.Ref[Mech].Pitch           = Gimbal_Motor[PITCH].Angle_DEG;	
        Gimbal.Ref[Mech].Yaw             = Expect_YawInit;
        Gimbal.Angle[Mech].ContinuousYaw = Gimbal_Motor[YAW].Angle; // 连续角度
        Gimbal.Angle[Mech].Pitch         = Gimbal_Motor[PITCH].Angle_DEG;
        Gimbal.YawInit                   = Expect_YawInit;
        Gimbal.MidMode                   = MidMode;

        // IMU 模式参考
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

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN2)
  {
    uint16_t CAN2_ID = CAN_Receive_DataFrame(&hcan2, CAN2_buff);
    switch (CAN2_ID)
    {
        case 0x101: memcpy(&Referee_data_Rx, CAN2_buff, sizeof(Referee_data_Rx));
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
