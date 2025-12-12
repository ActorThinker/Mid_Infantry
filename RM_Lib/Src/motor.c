/**
 * @file    motor.c
 * @author  yao
 * @date    1-May-2020
 * @brief   电机驱动模块
 * @note    此模块依赖CAN底层驱动模块
 */

#include <motor.h>
#ifdef HAL_CAN_MODULE_ENABLED

/**
 * @brief 用转矩电流计算得到功率值
 * @param[in] speed 电机速度
 * @param[in] current 转矩电流
 * @param[in] pcof 参数
 * @return 电机功率
 */
static inline float GetChassisMotorPower(int speed, int current, struct PowerCOF_s *pcof) {
    return (pcof->ss * speed * speed +
            pcof->sc * speed * current +
            pcof->cc * current * current +
            pcof->constant);
}

void RM3508_Receive(RM3508_TypeDef *Dst, uint8_t *Data) {
	Dst->MchanicalAngle = (uint16_t)(Data[0] << 8 | Data[1]);
	Dst->Speed = (int16_t)(Data[2] << 8 | Data[3]);
	Dst->TorqueCurrent = (uint16_t)(Data[4] << 8 | Data[5]);
	Dst->temp = Data[6];
	
	int16_t diff = Dst->MchanicalAngle - Dst->LsatAngle;
	if(diff < -4096){
		Dst->r++;
	}
	if(diff > 4096){
		Dst->r--; 
	}
	Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
	Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
	Dst->Power = GetChassisMotorPower(Dst->Speed, Dst->TorqueCurrent, &Dst->PowerCOF);
	Dst->LsatAngle = Dst->MchanicalAngle;
}
void GM6020_Receive(GM6020_TypeDef *Dst, uint8_t *Data) {
	Dst->MchanicalAngle = (uint16_t)(Data[0] << 8 | Data[1]);
	Dst->Speed = (int16_t)(Data[2] << 8 | Data[3]);
	Dst->TorqueCurrent = (uint16_t)(Data[4] << 8 | Data[5]);
	Dst->temp = Data[6];
	int16_t diff = Dst->MchanicalAngle - Dst->LsatAngle;
	if(diff < -4096){
		Dst->r++;
	}
	if(diff > 4096){
		Dst->r--; 
	}
	Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
	Dst->Angle_DEG = (float)Dst->Angle * 0.0439453125f;
	Dst->LsatAngle = Dst->MchanicalAngle;
}
void DM4310_Receive(DM4310_TypeDef *Dst, uint8_t *Data) {
    Dst->MchanicalAngle = (uint16_t)(Data[0] << 8 | Data[1]);
    Dst->Speed = (int16_t)(Data[2] << 8 | Data[3]) / 1000;
    Dst->TorqueCurrent = (int16_t)(Data[4] << 8 | Data[5]);
    Dst->temp = Data[6];
    Dst->PCBtemp = Data[7];

    int16_t diff = Dst->MchanicalAngle - Dst->LsatAngle;

    if (diff > 4000)
        Dst->r--;
    if (diff < -4000)
        Dst->r++;

    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
//    Dst->Power = GetChassisMotorPower(Dst->Speed, Dst->TorqueCurrent, &Dst->PowerCOF);
    Dst->LsatAngle = Dst->MchanicalAngle;
}

void M2006_Receive(M2006_TypeDef *Dst, uint8_t *Data) {
    Dst->MchanicalAngle = (uint16_t)(Data[0] << 8 | Data[1]);
    Dst->Speed = (int16_t)(Data[2] << 8 | Data[3]);

    int16_t diff = Dst->MchanicalAngle - Dst->LsatAngle;

    if(diff != Dst->MchanicalAngle)
        Dst->flag = 1;
    if(Dst->flag == 1)
    {
        if (diff > 4000)
            Dst->r--;
        if (diff < -4000)
            Dst->r++;
    }

    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
    Dst->LsatAngle = Dst->MchanicalAngle;
}

HAL_StatusTypeDef MotorSend(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t *Data) {
    uint8_t temp[8];
    temp[0] = (uint8_t)(Data[0] >> 8);
    temp[1] = (uint8_t)(Data[0] & 0xff);
    temp[2] = (uint8_t)(Data[1] >> 8);
    temp[3] = (uint8_t)(Data[1] & 0xff);
    temp[4] = (uint8_t)(Data[2] >> 8);
    temp[5] = (uint8_t)(Data[2] & 0xff);
    temp[6] = (uint8_t)(Data[3] >> 8);
    temp[7] = (uint8_t)(Data[3] & 0xff);
    return CAN_Send_StdDataFrame(hcan, StdId, temp);
}

int16_t QuickCentering(uint16_t current, uint16_t target)
{
    int32_t diff = (int32_t)target - (int32_t)current;
    if (diff > 4096) {
        target -= 8192;
    } else if (diff < -4096) {
        target +=8192;
    }
    return target;
}

osStatus_t DM4310_Motor_Temp(DM4310_TypeDef *dst) 
{
	if (dst->temp > 80)
		return osError;
	else
		return osOK;
}
#endif
