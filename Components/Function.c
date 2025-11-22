/*!
* @file     Function.c
* @brief    全局调用功能函数
*/
#include "Function.h"

#define ENCODER_RANGE 8192
#define HALF_RANGE (ENCODER_RANGE / 2)

/* 设备状态检测函数 */
osStatus_t RM3508_Motor_Temp(RM3508_TypeDef *dst)
{
	if (dst->temp >80)
		return osError;
	else
		return osOK;
}

osStatus_t GM6020_Motor_Temp(GM6020_TypeDef *dst)
{
	if (dst->temp >80)
		return osError;
	else
		return osOK;
}

osStatus_t REMOTE_IfDataError( void )
{
if ( (RC_CtrlData.rc.s1 != 1 && RC_CtrlData.rc.s1 != 3 && RC_CtrlData.rc.s1 != 2)
|| (RC_CtrlData.rc.s2 != 1 && RC_CtrlData.rc.s2 != 3 && RC_CtrlData.rc.s2 != 2)
|| (RC_CtrlData.rc.ch0 > 1684 || RC_CtrlData.rc.ch0 < 364)
|| (RC_CtrlData.rc.ch1 > 1684 || RC_CtrlData.rc.ch1 < 364)
|| (RC_CtrlData.rc.ch2 > 1684 || RC_CtrlData.rc.ch2 < 364)
|| (RC_CtrlData.rc.ch3 > 1684 || RC_CtrlData.rc.ch3 < 364) )
    return osError;
else
    return osOK;
}

osStatus_t IMU_IfDataError( void )
{

    if(fabs(IMU.Angle_Pitch)>180||fabs (IMU.Angle_Roll)>180||fabs (IMU.Angle_Yaw )>180
        ||(IMU.Angle_Pitch ==0&&IMU.Angle_Roll==0&&IMU.Angle_Yaw))
        return osError;
    else
        return osOK;
}

/* 斜坡函数（float） */
float RAMP_float( float final, float now, float ramp )
{
    float	buffer =final - now;
    if (buffer > 0){
        if (buffer > ramp)  
                now += ramp;  
        else
                now += buffer;
    } else {
        if (buffer < -ramp)
                now += -ramp;
        else
                now += buffer;
    }
    return now;
}

/**
 * @brief 将电机机械角度(0~8191)展开为连续角度，避免跨零跳变
 * @param cur_raw 当前机械角度(0~8191)
 * @param id 电机编号（区分Yaw/Pitch）
 * @return 连续角度（可正负无限累加）
 */
float GetContinuousAngle(uint16_t cur_raw, uint8_t id)
{
    static int32_t last_raw[2] = {0};    // 上一次的原始值
    static int32_t round_cnt[2] = {0};   // 圈数记录
    static int32_t continuous_angle[2] = {0};

    int32_t diff = (int32_t)cur_raw - last_raw[id];

    // 跨零点处理
    if (diff > ENCODER_HALF) {
        round_cnt[id]--;
    } else if (diff < -ENCODER_HALF) {
        round_cnt[id]++;
    }

    continuous_angle[id] = (int32_t)cur_raw + round_cnt[id] * ENCODER_MAX;
    last_raw[id] = cur_raw;

    return (float)continuous_angle[id];
}
