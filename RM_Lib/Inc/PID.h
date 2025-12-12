#pragma once
#include <math.h>
#include "stdint.h"
//错误类型枚举，目前只有两种：无错误和电机堵转。
typedef enum errorType_e{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef struct{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;
typedef struct _PID_TypeDef
{
  float Target;
	float Last_Target;
  float LastNoneZeroTarget;
	float Kp;
	float Ki;
	float Kd;
	float Kf;
	float dt;
	
  float Measure;
  float Last_Measure;
  float Err;
  float Last_Err;
	
  float Pout;
  float Iout;
  float Dout;
	float Fout;
  float ITerm;

  float Output;
  float Last_Output;
  float Last_Dout;

  float MaxOut;
  float IntegralLimit;
  float DeadBand;
	
  PID_ErrorHandler_t ERRORHandler;
} PID_TypeDef;
static void f_Output_Limit(PID_TypeDef *pid);
void PID_init(
	PID_TypeDef *pid,
	uint16_t max_out,
	float intergral_limit,
	float deadband,

	float kp,
	float ki,
	float kd,
	float kf,
	float dt
);
float PID_Calc(PID_TypeDef *pid, float measure, float target);
/**
 * @brief 标准位置式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float limit;        //!<@brief 积分限幅
    float error_thre;   //!<@brief 误差阈值，用于抗积分饱和
    float error_now;    //!<@brief 当前误差
    float error_inter;  //!<@brief 误差积分
    float DeadBand;     //!<@brief 误差死区
    float pid_out;      //!<@brief PID输出
    float error_last;   //!<@brief 上一次误差
} PID;
/**
 * @brief 带史密斯预估器的位置式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float limit;        //!<@brief 积分限幅
    float error_thre;   //!<@brief 积分分离，用于抗积分饱和
    float error_now;    //!<@brief 当前误差
    float error_inter;  //!<@brief 误差积分
    float DeadBand;     //!<@brief 误差死区
    float pid_out;      //!<@brief PID输出
} PID_Smis;
/**
 * @brief 前馈控制
 */
typedef struct{
	  float K1;
	  float K2;
	  float Last_DeltIn;
	  float Now_DeltIn;
	  float Out;
	  float OutMax;
}FeedForward_Typedef;
/**
 * @brief 标准位置式PID
 * @param[in] current 实际值
 * @param[in] expected 期望值
 * @param[in] parameter PID参数
 */
void PID_Control(float current, float expected, PID *data);
/**
 * @brief 带史密斯预估器的位置式PID
 * @param[in] current 实际值
 * @param[in] expected 期望值
 * @param[in] parameter PID参数
 * @param[in] speed 实际速度
 */
void PID_Control_Smis(float current, float expected, PID_Smis *data, float speed);
/* 前馈 */
float FeedForward_Calc(FeedForward_Typedef *FF);
