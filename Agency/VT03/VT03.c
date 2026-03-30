#include "VT03.h"
#include "CRC.h"
#include "Referee.h"
#include "cmsis_os2.h"
#include "string.h"
#include "WatchDog.h"
#include "Variate.h"

VT03_t VT03;

Referee_t Referee = { .SOF = 0xA5,
											.HEAD_LEN = sizeof(FrameHeader_t),
											.CMD_ID_LEN = 2,
											.TAIL_LEN = 2,
											.DATA_START = 4
};

void VT03_Task(){
	VT03_Init(&huart6,true);
	VT03_Request();
	for(;;){
		osDelay(1);
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size){  
  if (huart == VT03.huart_) {
		VT03_Update(VT03.VT03_Buff_,Size);
    VT03_Request();
		Feed_Dog(&Referee_Dog);
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart){
  if (huart == VT03.huart_) {
    VT03_Request();
  }
}

void VT03_Init(UART_HandleTypeDef * huart,bool use_dma){
	VT03.huart_ = huart;
	VT03.use_dma_ = use_dma;
}
void VT03_Request(){
	if(VT03.use_dma_){
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(VT03.huart_, VT03.VT03_Buff_,VT03_BUFF_SIZE);
    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(VT03.huart_->hdmarx, DMA_IT_HT);		
	} else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(VT03.huart_,VT03.VT03_Buff_,VT03_BUFF_SIZE);		
	}	
}
void VT03_Update(uint8_t * frame_start, uint16_t size){
	VT03.has_read_ = true;
	
  if (frame_start[0] == 0xA9 && frame_start[1] == 0x53) {
    size_t frame_len = sizeof(VT03RemoteData_t);

    if (size < frame_len) return;
    if (!verify_CRC16_check_sum(frame_start, frame_len)) return;

    VT03_ToRemote((VT03RemoteData_t *)(frame_start));

    // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
    VT03_Update(frame_start + frame_len,size - frame_len);
    return;
  }

  if (size < Referee.HEAD_LEN) return;
  if (frame_start[0] != Referee.SOF) return;
  if (!verify_CRC8_check_sum(frame_start, Referee.HEAD_LEN)) return;

  size_t data_len = (frame_start[2] << 8) | frame_start[1];
  size_t frame_len = Referee.HEAD_LEN + Referee.CMD_ID_LEN + data_len + Referee.TAIL_LEN;

  if (size < frame_len) return;
  if (!verify_CRC16_check_sum(frame_start, frame_len)) return;

  uint16_t cmd_id = (frame_start[6] << 8) | frame_start[5];

  switch (cmd_id) {
//    // 0x0302 自定义控制器与机器人交互数据
//    case CUSTOM_ROBOT_DATA:
//      memcpy(
//        frame_start + Referee.DATA_START, frame_start + Referee.DATA_START + data_len,
//        (uint8_t *)(&VT03.custom));
//      break;

//    // 0x0309 自定义控制器与机器人交互数据
//    case ROBOT_CUSTOM_DATA :
//      memcpy(
//        frame_start + Referee.DATA_START, frame_start + Referee.DATA_START + data_len,
//        (uint8_t *)(&VT03.robot));
//      break;

    default:
      break;
  }
  // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
  VT03_Update(frame_start + frame_len, size - frame_len);	
}
void VT03_ToRemote(const VT03RemoteData_t * data){
	VT03.ch_rx = (data->ch_0 - 1024) / 660.0f;
  VT03.ch_ry = (data->ch_1 - 1024) / 660.0f;
  VT03.ch_lx = (data->ch_2 - 1024) / 660.0f;
  VT03.ch_ly = (data->ch_3 - 1024) / 660.0f;
  VT03.wheel = (data->wheel - 1024) / 660.0f;

  VT03.fn_l = data->fn_1;
  VT03.fn_r = data->fn_2;
  VT03.pause = data->pause;
  VT03.trigger = data->trigger;

  VT03.mouse.vx = data->mouse_x / 32768.0f;
  VT03.mouse.vy = data->mouse_y / 32768.0f;
  VT03.mouse.vz = data->mouse_z / 32768.0f;
  VT03.mouse.left = data->mouse_left;
  VT03.mouse.middle = data->mouse_middle;
  VT03.mouse.right = data->mouse_right;

  VT03.keys.W = (data->keys & 0x0001);
  VT03.keys.S = (data->keys & 0x0002);
  VT03.keys.A = (data->keys & 0x0004);
  VT03.keys.D = (data->keys & 0x0008);
  VT03.keys.Shift = (data->keys & 0x0010);
  VT03.keys.Ctrl = (data->keys & 0x0020);
  VT03.keys.Q = (data->keys & 0x0040);
  VT03.keys.E = (data->keys & 0x0080);
  VT03.keys.R = (data->keys & 0x0100);
  VT03.keys.F = (data->keys & 0x0200);
  VT03.keys.G = (data->keys & 0x0400);
  VT03.keys.Z = (data->keys & 0x0800);
  VT03.keys.X = (data->keys & 0x1000);
  VT03.keys.C = (data->keys & 0x2000);
  VT03.keys.V = (data->keys & 0x4000);
  VT03.keys.B = (data->keys & 0x8000);

  VT03.mode = (data->mode_sw == 0) ? (VT03.mode = C) : 
							(data->mode_sw == 1) ? (VT03.mode = N) : (VT03.mode = S);
}