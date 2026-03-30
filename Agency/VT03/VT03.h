#ifndef __VT03_H__
#define __VT03_H__

#include "main.h"
#include "stdbool.h"
#include "usart.h"

#define VT03_BUFF_SIZE 255

typedef struct{
  float yaw;
  float roll;
  float pitch;
  float roll2;
  float x;
  float y;
  float z;
  bool button;
  uint8_t reserved;  //保留位
}__PACKED RefereeCustomData_t; 
typedef struct {
  bool yaw_fdb_on;
  bool roll_fdb_on;
  bool pitch_fdb_on;
  bool roll2_fdb_on;
  bool x_fdb_on;
  bool y_fdb_on;
  bool z_fdb_on;
  uint8_t reserved[23];  //保留位 23位
}__PACKED RefereeRobotData_t;

typedef struct{
  uint8_t sof_1;
  uint8_t sof_2;
  uint64_t ch_0 : 11;
  uint64_t ch_1 : 11;
  uint64_t ch_2 : 11;
  uint64_t ch_3 : 11;
  uint64_t mode_sw : 2;
  uint64_t pause : 1;
  uint64_t fn_1 : 1;
  uint64_t fn_2 : 1;
  uint64_t wheel : 11;
  uint64_t trigger : 1;
  uint64_t padding1 : 3;  

  int16_t mouse_x;  
  int16_t mouse_y;  
  int16_t mouse_z;  

  uint8_t mouse_left : 2;   
  uint8_t mouse_right : 2;   
  uint8_t mouse_middle : 2;  
  uint8_t padding2 : 2;      

  uint16_t keys;   
  uint16_t crc16;  
}__PACKED VT03RemoteData_t;
typedef struct{
  float vx;  // 取值范围: [-1, 1]
  float vy;  // 取值范围: [-1, 1]
  float vz;  // 取值范围: [-1, 1], 鼠标滚轮
  bool left;
  bool middle;  //中键
  bool right;
}VT03MouseData_t;
typedef struct{
  bool W;
  bool S;
  bool A;
  bool D;
  bool Shift;
  bool Ctrl;
  bool Q;
  bool E;
  bool R;
  bool F;
  bool G;
  bool Z;
  bool X;
  bool C;
  bool V;
  bool B;
}__PACKED VT03KeysData_t;
typedef struct{
	enum VT03MODE{
	C = 0,
	N = 1,
	S = 2
	}mode;
	float ch_rx;
	float ch_ry;
	float ch_lx;
	float ch_ly;
	float wheel;
	float fn_l;
	float fn_r;
	float pause;
	float trigger;
	RefereeCustomData_t custom;
	RefereeRobotData_t robot; 
	VT03MouseData_t mouse;
	VT03KeysData_t keys;
	
	UART_HandleTypeDef * huart_;
	bool use_dma_;
	bool has_read_;
	uint8_t VT03_Buff_[VT03_BUFF_SIZE];
	
}VT03_t;
extern VT03_t VT03;

void VT03_Init(UART_HandleTypeDef * huart,bool use_dma);
void VT03_Request();
void VT03_Update(uint8_t * frame_start,uint16_t size);
void VT03_ToRemote(const VT03RemoteData_t * data);
extern void VT03_Task();

#endif
