#ifndef __SHOOT_H
#define __SHOOT_H

#include "Variate.h"
#include "USB_Task.h"

extern void Shoot_Stop();
extern void Shoot_Rc_Ctrl();    //!< @brief 发射机构遥控器模式
extern void Shoot_Key_Ctrl();   //!< @brief 发射机构键鼠模式
extern void Shoot_Drive();     //!< @brief 发射机构电机驱动
extern void Shoot_Close();
/* 检测发射机构 */
extern void Detect_Shoot();
/* 发布发射机构 */
extern void ShootPublish();
/* 更新状态量̬*/
extern void ShootData_Update();
/* 决定控制方式 */
extern void ShootCtrl_Decide();
/* 处理异常 */
extern void ShootHandleEception();
/* 设置目标量 */
extern void ShootRef_Set();
/* 枪口热量限制 */
extern void ShootHeat_Limit();
/* 计算控制量 */
extern void Shoot_Console();
/* 发送控制量 */
extern void Shoot_Send();
extern void Shoot_SendDown();
/* 自瞄 */
extern void Aim_Shoot();
#endif
