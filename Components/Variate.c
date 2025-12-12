#include "Variate.h"
int StuckFlag = 0;
int16_t pluck_speed;

uint8_t GimbalInitFlag = 0;

Gimbal_data_t Gimbal_data;
Gimbal_action_t Gimbal_action;

GM6020_TypeDef Gimbal_Motor[GIMBAL_SUM];

AIM_Typedef Aim_Data;

uint8_t NormalModeFlag = 0,GyroscopeModeFlag = 0;
Chassis_board_send_t Referee_data_Rx;
eSystemState SystemState;
DeviceStates DeviceState;
eAimAction AimAction         = AIM_STOP;
eMidMode MidMode             = FRONT;
