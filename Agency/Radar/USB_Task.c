#include "usb_task.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

SendVision_t	 			SendVision = {.header = 0x5A,.footer = 0xA5};
ReceiveVisionData_t ReceiveVisionData = {.frame_header.sof = 0x5A, .frame_header.id = 0X02,.eof  = 0xA5,.data.dis = -1};

static SendDataImu_s	SEND_DATA_IMU;
static uint8_t data_buffer[64] = {0};

RobotCmdData_t ROBOT_CMD_DATA ;

typedef struct
{
  uint32_t Imu;
	uint32_t Vision;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

static void UsbReceiveData(void);
static void UsbInit(void);

static void UsbSendImuData(void);

void usb_task(void *pvParameters)
{
    UsbInit();
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	
    for(;;)
    {
        UsbSendImuData();
        UsbReceiveData();
        vTaskDelayUntil(&xLastWakeTime,1);
    }
}
static void UsbInit(void)
{
  memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
	memset(&ReceiveVisionData.data,0,sizeof(ReceiveVisionData.data));

  SEND_DATA_IMU.frame_header.sof = SEND_SOF;
  SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
  SEND_DATA_IMU.frame_header.id  = DATA_SEND_ID_Imu;
}

static void UsbReceiveData(void) {
  uint32_t actual_len = 0;
  USB_Receive(data_buffer, &actual_len);
	memcpy(&ReceiveVisionData, data_buffer, sizeof(ReceiveVisionData_t));
//	if(ReceiveVisionData.frame_header.sof == 0x5A && ReceiveVisionData.eof == 0xA5){
//		LAST_SEND_TIME.Vision = HAL_GetTick();
//		
//		if(GimbalCtrl == gAim){
//			tracking = 1;
//			Feed_Dog(&PC_Dog);
//		}
//		memset(&data_buffer[0],0,sizeof(data_buffer[0]));
//	}
//	
//	if(HAL_GetTick() - LAST_SEND_TIME.Vision > 500){
//		memset(&data_buffer,0,sizeof(data_buffer));
//		tracking = 0;
//	}
}

static void UsbSendImuData(void)
{
	SEND_DATA_IMU.time_stamp = HAL_GetTick();//获取当前时间戳

	SEND_DATA_IMU.data.pitch = 	IMU.Angle_Pitch *Pi/180.0f;
	SEND_DATA_IMU.data.yaw 	 = 	IMU.Angle_Yaw 	*Pi/180.0f;
	SEND_DATA_IMU.data.roll  =	IMU.Angle_Roll 	*Pi/180.0f;
	SEND_DATA_IMU.data.pitch_vel = IMU.Gyro_Pitch;
	SEND_DATA_IMU.data.yaw_vel 	 = IMU.Gyro_Yaw;  
	SEND_DATA_IMU.data.roll_vel  = IMU.Gyro_Roll; 
	SEND_DATA_IMU.data.self_color = 1 ;//1蓝0红

  SEND_DATA_IMU.eof = SEND_EOF;

  USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}
