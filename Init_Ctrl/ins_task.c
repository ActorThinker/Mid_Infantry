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
#include "IMU.h"
#include "WatchDog.h"
#include "spi.h"

INS_t INS;
IMU_Param_t IMU_Param;
PID_t TempCtrl = {0};
WatchDog_TypeDef IMU_Dog;
void IMU_Rx();

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
float RefTemp = 45.0;
void InitQuaternion(float *init_q4);

void Cross3d(float *v1, float *v2, float *res);
float Dot3d(float *v1, float *v2);
float NormOf3d(float *v);
float *Norm3d(float *v);

void INS_Init(void)
{
    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0.98f);
    
    // imu heat init
    PID_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

//    INS.AccelLPF = 0.0085;
		INS.AccelLPF = 0.008;
}

void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;
    // ins update
    if ((count % 1) == 0)
    {
			BMI088_Read(&BMI088);

			INS.Accel[X] = BMI088.Accel[X];
			INS.Accel[Y] = BMI088.Accel[Y];
			INS.Accel[Z] = BMI088.Accel[Z];
			INS.Gyro[X] = BMI088.Gyro[X];
			INS.Gyro[Y] = BMI088.Gyro[Y];
			INS.Gyro[Z] = BMI088.Gyro[Z];
      // 核心函数,EKF更新四元数
      IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

      memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
			// 获取最终数据
			INS.Yaw = QEKF_INS.Yaw;
			INS.Pitch = QEKF_INS.Pitch;
			INS.Roll = QEKF_INS.Roll;
			INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
			IMU_Rx();
			Feed_Dog(&IMU_Dog);
    }
    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
    }
    count++;
}
// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);
        acc_init[X] += BMI088.Accel[X];
        acc_init[Y] += BMI088.Accel[Y];
        acc_init[Z] += BMI088.Accel[Z];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

void IMU_Rx(){
		 IMU.Angle_Pitch         = INS.Pitch;
		 IMU.Angle_Yaw           = INS.Yaw;
	   IMU.Angle_Roll          = INS.Roll;
	   IMU.Angle_Yawcontinuous = INS.YawTotalAngle;
	   IMU.Gyro_Pitch          = INS.Gyro[X];
	   IMU.Gyro_Roll           = INS.Gyro[Y];
	   IMU.Gyro_Yaw            = INS.Gyro[Z];
		 IMU.r = QEKF_INS.YawRoundCount;
	   for(int i = 0;i < 4;i++)IMU.q[i] = INS.q[i];
}
/**
 * @brief 温度控制
 * 
 */
void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT16_MAX));
}
// 三维向量归一化
float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

