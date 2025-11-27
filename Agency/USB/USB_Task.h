#ifndef USB_TASK_H
#define USB_TASK_H

#include "Variate.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#define SEND_SOF    ((uint8_t)0x5A)
#define SEND_EOF    ((uint8_t)0xA5)
#define RECEIVE_SOF ((uint8_t)0x5A)
#define RECEIVE_EOF ((uint8_t)0xA5)

#define SEND_IMU_DATA_ID ((uint8_t)0x01)
#define RECEIVE_VISION_DATA_TD ((uint8_t)0x02)
typedef struct{
	uint8_t sof;
	uint8_t len;
	uint8_t id;
} __PACKED FrameHeader_t;
/*-------------------- Send --------------------*/
typedef struct
{
  FrameHeader_t header;
  uint32_t time_stamp;
  struct{
		uint8_t self_color ; //0-blud  1-red
    float yaw;    // rad
		float pitch;  // rad
		float roll;   // rad

		float yaw_vel;    // rad/s
		float pitch_vel;  // rad/s
		float roll_vel;   // rad/s
    } __PACKED data;
    uint8_t eof;
} __PACKED SendDataImu_s;
/*-------------------- Receive --------------------*/
typedef struct{
  FrameHeader_t header;//3
  struct{
	  uint8_t FireFlag;
		float Ref_Yaw;//°
		float Ref_Pitch;
		float Ref_Vyaw;
		float Ref_Vpitch;
		float Ref_aYaw;
		float Ref_aPitch;
		float dis;
  } __PACKED data;
  uint8_t eof;//1
} __PACKED ReceiveVisionData_t;
extern ReceiveVisionData_t ReceiveVisionData;

extern void usb_task();

#endif /* USB_TASK_H */
