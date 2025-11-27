#ifndef __VARIATE_H
#define __VARIATE_H
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "tim.h"
#include "remote_control.h"
#include "remote.h"
#include "motor.h"
#include "CANDrive.h"
#include "WatchDog.h"
#include "PID.h"
#include "IMU.h"
#include "Data_Exchange.h"

#include "stdbool.h"
#include "Attribute_Typedef.h"
#include "Function.h"

#define CHASSIS_RUN 1
#define GIMBAL_RUN  1
#define SHOOT_RUN   1

#define current_to_out 16384.0f / 3.0f

#define RobotID 1 	//0 1ç™½0é»‘
#define Pimu_ADD_limit 10
#define Pimu_LOSE_limit 34

#define SHOOT_SPEED 6000
#define PLUCK_SPEED 8500
#define PLUCK_MOTOR_ONE 1360

#define Pi 3.14159265358979f
#if RobotID == 0
#define Yaw_Mid_Front 1179
#define Pitch_Mid 4333
#define P_ADD_limit 210
#define P_LOSE_limit 167
#elif RobotID == 1
#define Yaw_Mid_Front 4670
#define Pitch_Mid 1285
#define P_ADD_limit 80
#define P_LOSE_limit 30

#endif

#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;           \
    if (IN > MAX)           \
        IN = MAX

enum{
	YAW = 1,
	PITCH = 0,
	GIMBAL_SUM = 2,
};
enum{
    LEFT = 1,
    RIGHT= 0,
    FRIC_SUM = 2,
};
#if (Yaw_Mid_Front + 2048) > 8191
#define Yaw_Mid_Left Yaw_Mid_Front - 6143
#else
#define Yaw_Mid_Left Yaw_Mid_Front + 2048
#endif

#if (Yaw_Mid_Left + 2048) > 8191
#define Yaw_Mid_Back Yaw_Mid_Left - 6143
#else
#define Yaw_Mid_Back Yaw_Mid_Left + 2048
#endif

#if (Yaw_Mid_Back + 2048) > 8191
#define Yaw_Mid_Right Yaw_Mid_Back - 6143
#else
#define Yaw_Mid_Right Yaw_Mid_Back + 2048
#endif
/** @brief µç»ú²Î¿¼ËÙ¶È */
#define STD_Speed  2406		// ±ê×¼ËÙ¶È1m/s
#define STD_Omega  6142		// ±ê×¼ËÙ¶È1rpm/s
#define STD_Angle  0.36f	// ½Ç¶ÈÖÆ1rpm/s
#define STD_MAngle 8.192	// »úÐµ½Ç¶ÈÖÆ1rpm/s

#define IMU_NAME "imu_data"


/* ¿¨µ¯±êÖ¾Î» */
extern int StuckFlag;
/* ÔÆÌ¨³õÊ¼»¯±êÖ¾Î» */
extern uint8_t GimbalInitFlag;

/* ÏµÍ³×´Ì¬ */
typedef enum{
	SYSTEM_STARTING = 0,     //!< @brief ÕýÔÚÆô¶¯
	SYSTEM_RUNNING  = 1,	 //!< @brief ÕýÔÚÔËÐÐ
} eSystemState;
/*Éè±¸×´Ì¬ */
typedef enum{
	Device_Offline = 0,     //!< @brief Éè±¸ÀëÏß
	Device_Online  = 1,	    //!< @brief Éè±¸ÔÚÏß
	Device_Error   = 2	    //!< @brief Éè±¸´íÎó
} eDeviceState;
typedef struct {
	eDeviceState Remote_State, IMU_State, Gimbal_State[GIMBAL_SUM], Shoot_State[FRIC_SUM], Pluck_State, Down_State, PC_State,Referee_State;
}DeviceStates;
extern DeviceStates DeviceState;
extern eSystemState SystemState;

/* µç»ú */
extern GM6020_TypeDef Gimbal_Motor[GIMBAL_SUM];
extern RM3508_TypeDef Shoot_Motor[FRIC_SUM];
extern M2006_TypeDef  Pluck_Motor;

/* ÔÆÌ¨¹éÖÐÎ»ÖÃ */
typedef enum{
	FRONT = 0,       //!< @brief   Ç°·½
	BACK  = 1,       //!< @brief   ºó·½
} eMidMode;
extern eMidMode MidMode;

/* ×ÔÃé×´Ì¬ */
typedef enum{
	AIM_STOP = 0,     //!< @brief   ¹Ø±Õ×ÔÃé
	AIM_AID  = 1,     //!< @brief   ×ÔÃé²»×Ô¶¯Éä»÷
	AIM_AUTO = 2,     //!< @brief   ×ÔÃé+×Ô¶¯Éä»÷ ¾Ím
} eAimAction;
extern eAimAction AimAction;

/* ÊÓ¾õ */
typedef struct{
	uint8_t Flag;
	float Ref_Yaw;
	float Ref_Pitch;
	float HorizontalDistance;
	float id;
	enum {
	 AimStop  = 0,  
	 AimReady = 1,
	 AimFire  = 2,
	}AimShoot;
}AIM_Typedef;
extern AIM_Typedef Aim_Data;
extern int AimAllow;

/* ²ÃÅÐÏµÍ³ */
typedef struct  {
//	int8_t  robot_color;     		// »úÆ÷ÈËÑÕÉ«
	uint16_t heat_limit_remain; 	// Ê£ÓàÈÈÁ¿
	uint16_t heat_limit_recover;	// ÀäÈ´ËÙÂÊ
	int16_t ChassisSpeed; 			//µ×ÅÌ½ÇËÙ¶È
	int8_t game_state_robot_color;	//±ÈÈü×´Ì¬    --0 Î´¿ªÊ¼ --1 ¿ªÊ¼
} Chassis_board_send_t;


/* ·¢ËÍµ×ÅÌÊý¾Ý */
typedef enum {
	Gimbal_offline = 0,    
	Gimbal_online  = 1,     
}Gimbal_status_e;

typedef enum {
	shoot_offline = 0,     
	shoot_online  = 1,      
}shoot_status_e;

typedef enum {
	stop = 0,    
	normal  = 1,   
  rotate = 2,
  fllow = 3,
  grasp = 4
}move_status_e;

typedef enum {
	vision_offline = 0,   
	vision_online  = 1,    
}vision_status_e;


typedef enum {
	shoot_mode_stop = 0,   
	shoot_mode_ready  = 1,   
  shoot_mode_fire = 2,
  shoot_mode_follow = 3,
  shoot_mode_stucking = 4,
}shoot_mode_e;

typedef struct  {
    Gimbal_status_e Pitch: 1;
    Gimbal_status_e Yaw : 1;
} Gimbal_status_t;

typedef struct  {
	uint16_t vision_distance;  
	int16_t Pitch_angle;
	int16_t Yaw_angle;
	uint16_t Offset_Angle;
} Gimbal_data_t;

typedef struct{
    Gimbal_status_t Gimbal_status;         
    shoot_status_e shoot_status : 1;          
    move_status_e move_status : 4;     
    vision_status_e vision_status : 1;          
    shoot_mode_e shoot_mode;				           
	uint8_t Key; 
	uint8_t vision_number; 
}Gimbal_action_t;
/* ÉÏÏÂ°åÍ¨ÐÅ·¢ËÍ */
extern Gimbal_data_t Gimbal_data;
extern Gimbal_action_t Gimbal_action;

/** @briefn */
/* Ò£¿ØÆ÷ */
extern float Key_ch[4], Mouse_ch[3];
extern InputMode_e RemoteMode;
/* ÈÎÎñ¾ä±ú */
extern TaskHandle_t  Chassis_Task_handle, Gimbal_Task_handle, Shoot_Task_handle, Ins_Task_handle,Music_Task_handle,usb_task_handle;
/* ¿´ÃÅ¹· */
extern WatchDog_TypeDef Remote_Dog, IMU_Dog, Gimbal_Dog[GIMBAL_SUM], Shoot_Dog[FRIC_SUM], Pluck_Dog, Down_Dog, PC_Dog,Referee_Dog;
/* ²ÃÅÐÏµÍ³ */
extern Chassis_board_send_t Referee_data_Rx;      // ÉÏÏÂ°åÍ¨ÐÅ·¢ËÍ

extern uint8_t NormalModeFlag,GyroscopeModeFlag;

extern int16_t pluck_speed;

#endif
