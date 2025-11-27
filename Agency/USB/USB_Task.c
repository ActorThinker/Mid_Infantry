#include "usb_task.h"

static SendDataImu_s SEND_DATA_IMU = {.header.sof = 0x5A,
																			.header.len = (uint8_t)(sizeof(SendDataImu_s) - 6),
																			.header.id  = 0x01,
																			.eof = 0xA5};
ReceiveVisionData_t ReceiveVisionData = {.header.sof = 0x5A,
																				 .header.id  = 0X02,
																				 .eof = 0xA5,
																				 .data.dis = -1};
static void UsbInit(void);
static void UsbReceiveData(void);
static void UsbSendImuData(void);

void usb_task(void *pvParameters){
	UsbInit();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
			UsbSendImuData();
			UsbReceiveData();
			vTaskDelayUntil(&xLastWakeTime,1);
	}
}
static void UsbInit(void){
	memset(&SEND_DATA_IMU.data,0,sizeof(SEND_DATA_IMU.data));
	memset(&ReceiveVisionData.data,0,sizeof(ReceiveVisionData.data));
}

static void UsbReceiveData(void) {
	static uint8_t data_buffer[64] = {0};
  uint32_t actual_len = 0;
  USB_Receive(data_buffer, &actual_len);
	memcpy(&ReceiveVisionData, data_buffer, sizeof(ReceiveVisionData_t));
}

static void UsbSendImuData(void){
	SEND_DATA_IMU.time_stamp = HAL_GetTick();//获取当前时间戳

	SEND_DATA_IMU.data.pitch = 	IMU.Angle_Pitch *Pi/180.0f;//rad
	SEND_DATA_IMU.data.yaw 	 = 	IMU.Angle_Yaw 	*Pi/180.0f;
	SEND_DATA_IMU.data.roll  =	IMU.Angle_Roll 	*Pi/180.0f;
	SEND_DATA_IMU.data.pitch_vel = IMU.Gyro_Pitch;
	SEND_DATA_IMU.data.yaw_vel 	 = IMU.Gyro_Yaw;  
	SEND_DATA_IMU.data.roll_vel  = IMU.Gyro_Roll; 
	SEND_DATA_IMU.data.self_color = 1 ;//1蓝0红

  USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}
