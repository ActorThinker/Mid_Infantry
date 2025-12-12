/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "WatchDog.h"
#include "spi.h"
#include "bsp_dwt.h"

INS_t INS;
IMU_t IMU;
PID_t TempCtrl = {0};
WatchDog_TypeDef IMU_Dog;

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
float RefTemp = 45.0;

void INS_Init(void){
	float init_quaternion[4] = {1,0,0,0};
	IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0.98f);  
	// imu heat init
	PID_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0,0,0,0,0);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	INS.AccelLPF = 0.008;
}
void INS_Task(void){
	static uint32_t count = 0;
	const float gravity[3] = {0, 0, 9.7997f};
	dt = DWT_GetDeltaT(&INS_DWT_Count);
	t += dt;
	// ins update
	BMI088_Read(&BMI088);
	INS.Accel[0] = BMI088.Accel[0];
	INS.Accel[1] = BMI088.Accel[1];
	INS.Accel[2] = BMI088.Accel[2];
	INS.Gyro[0] = BMI088.Gyro[0];
	INS.Gyro[1] = BMI088.Gyro[1];
	INS.Gyro[2] = BMI088.Gyro[2];
	// 核心函数,EKF更新四元数
	IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2], INS.Accel[0], INS.Accel[1], INS.Accel[2], dt);
	memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
	// 获取最终数据
	INS.Yaw = QEKF_INS.Yaw;
	INS.Pitch = QEKF_INS.Pitch;
	INS.Roll = QEKF_INS.Roll;
	INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
	INS.VisionAngle = QEKF_INS.VisionAngle;
	IMU_Rx();
	Feed_Dog(&IMU_Dog);
	if ((count % 2) == 0){			// 500hz
		IMU_Temperature_Ctrl();
	}
	count++;
}
void IMU_Rx(){
	IMU.Angle_Roll          = INS.Roll;
	IMU.Angle_Pitch         = INS.Pitch;
	IMU.Angle_Yaw           = INS.Yaw;
	IMU.Angle_Yawcontinuous = INS.YawTotalAngle;
	IMU.VisionAngle 				= INS.VisionAngle;
	IMU.Gyro_Roll           = INS.Gyro[1];
	IMU.Gyro_Pitch          = INS.Gyro[0];
	IMU.Gyro_Yaw            = INS.Gyro[2];
	IMU.r = QEKF_INS.YawRoundCount;
	for(int i = 0;i < 4;i++)IMU.q[i] = INS.q[i];
}
/**
 * @brief 温度控制
 * 
 */
void IMU_Temperature_Ctrl(void){
	PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);
	TIM_Set_PWM(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT16_MAX));
}
