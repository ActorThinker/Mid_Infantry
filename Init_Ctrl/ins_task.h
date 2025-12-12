#pragma once

#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"

typedef struct{
	float q[4]; // 四元数估计值
	float Gyro[3];  // 角速度
	float Accel[3]; // 加速度
	float AccelLPF; // 加速度低通滤波系数
	// 位姿
	float Roll;
	float Pitch;
	float Yaw;
	float YawTotalAngle;
	float VisionAngle;
} INS_t;
extern INS_t INS;
typedef struct{
	float Gyro_Roll;
	float Gyro_Pitch;
	float Gyro_Yaw;
	float Angle_Roll;
	float Angle_Pitch;    
	float Angle_Yaw;
	float Angle_Yawcontinuous;
	float VisionAngle;
	float q[4];
	int r;
}IMU_t;
extern IMU_t IMU;
extern void IMU_Rx();
extern void INS_Init(void);
extern void INS_Task(void);
extern void IMU_Temperature_Ctrl(void);
