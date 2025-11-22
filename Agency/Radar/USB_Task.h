#ifndef USB_TASK_H
#define USB_TASK_H

#include "Variate.h"
#include "Function.h"

extern int tracking;
extern void usb_task();

#define SEND_SOF            ((uint8_t)0x5A)
#define SEND_EOF            ((uint8_t)0xA5)
#define RECEIVE_SOF 		((uint8_t)0x5A)
#define RECEIVE_EOF 		((uint8_t)0xA5)

#define DATA_SEND_ID_Imu           ((uint8_t)0x01)  // ms

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x03)
//#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)

#define ROBOT_VISION_DATA_RECEIVE_ID ((uint8_t)0x02)
typedef struct
{
    uint8_t sof;
    uint8_t len;
    uint8_t id;
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/
typedef struct
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;

    struct
    {
	uint8_t self_color ; //0-blud  1-red
        float yaw;    // rad
        float pitch;  // rad
        float roll;   // rad

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s
    } __packed__ data;
    uint8_t eof;
} __packed__ SendDataImu_s;
/*-------------------- Receive --------------------*/
typedef struct 
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
        
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector_last;
    } __packed__ data;
    uint16_t checksum;
} __packed__ RobotCmdData_t;

extern RobotCmdData_t ROBOT_CMD_DATA ;
extern RobotCmdData_t ROBOT_CMD_LASTDATA ;

typedef struct {
	uint8_t header;
	uint8_t detect_color : 1;
	bool reset_tracker : 1;
	uint8_t reserved :6;
	float roll;
	float pitch;
	float yaw;
	float aim_x;
	float aim_y;
	float aim_z;
    uint8_t footer;
}SendVision_t;
extern SendVision_t SendVision;

typedef struct{
  FrameHeader_t frame_header;//3
  struct
  {
	  uint8_t FireFlag;
        float Ref_Yaw;
        float Ref_Pitch;
        float Ref_Vyaw;
        float Ref_Vpitch;
        float Ref_aYaw;
        float Ref_aPitch;
		float dis;
  } __attribute__((packed)) data;

  uint8_t eof;//1
} __packed__ ReceiveVisionData_t;
extern ReceiveVisionData_t ReceiveVisionData;

typedef struct {
	 uint8_t header;
	 float linear_x;
	 float linear_y;
	 float linear_z;
	 float angular_x;
	 float angular_y;
	 float angular_z;
	 uint16_t checksum;
}ReceiveTwist_t;
extern ReceiveTwist_t ReceiveTwist;

#endif /* USB_TASK_H */
