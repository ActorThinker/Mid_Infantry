# RM2026 CurrrentSmall

![AT Infantry](docs/actor_thinker.png)

基于 `STM32F407IGH6`、`STM32CubeMX`、`FreeRTOS` 和 `Keil MDK-ARM` 的 RoboMaster 云台主控工程。  
从当前代码来看，这个仓库更像是“云台侧主控板”工程，而不是整车双板总仓：

- 负责 BMI088 IMU 姿态解算与温控
- 负责云台 yaw / pitch 双轴控制与归中
- 负责发射机构控制
- 负责与下板通过 CAN 交换状态和控制量
- 负责与 MiniPC / 视觉侧通过 USB CDC 通信
- 负责遥控器、看门狗、蜂鸣器等基础逻辑

## 项目概述

工程入口位于 `MDK-ARM/C_Board_Standard_Robot.uvprojx`，底层初始化由 CubeMX 生成，业务逻辑主要分布在以下目录：

```text
CurrrentSmall
|-- Agency/         业务模块：云台、发射、底盘交互、USB、看门狗、串口工具
|-- Application/    应用层任务入口（当前与 Init_Ctrl 存在重复目录）
|-- Bsp/            板级支持：DWT、PWM、RC、蜂鸣器等
|-- Components/     算法、设备驱动、公共组件
|-- Init_Ctrl/      任务初始化与主控调度
|-- Inc/            CubeMX 生成头文件
|-- Src/            CubeMX 生成底层初始化与中断/USB/FreeRTOS 入口
|-- MDK-ARM/        Keil 工程与构建产物
`-- README.md
```

## 软件架构

### 启动流程

`Src/main.c` 在完成 HAL、时钟、CAN / SPI / TIM / USART / RNG 等初始化后：

1. 初始化 `DWT`
2. 阻塞等待 `BMI088Init()` 成功
3. 启动 FreeRTOS
4. 创建 `INSTask` 与 `Init_Task`

其中：

- `INSTask` 负责 IMU 解算
- `Init_Task` 负责 CAN、遥控器、看门狗初始化以及后续业务任务创建

### 任务划分

当前代码中的主要任务如下：

- `INSTask`  
  1ms 周期读取 BMI088，进行四元数 EKF 姿态解算，并执行 IMU 温控。
- `MainCtrl_Task`  
  15ms 周期执行总控逻辑，管理遥控在线状态、上下板 CAN 发送、看门狗轮询。
- `Gimbal_Task`  
  1ms 周期执行云台模式决策、参考值更新、姿态闭环和 CAN 电机发送。
- `Shoot_Task`  
  1ms 周期执行发射模式切换、卡弹检测、拨弹/摩擦轮控制。
- `usb_task`  
  1ms 周期发送 IMU 数据到上位机，并接收视觉目标数据。
- `Chassis_Task`  
  1ms 周期生成底盘控制量并通过 CAN 下发到底盘板。
- `VT03_Task` / `Music_Task`  
  分别用于视觉串口协议相关功能和蜂鸣器提示。

## 当前已实现功能

以下内容基于当前仓库源码静态整理，而不是根据历史说明推断。

### IMU 与姿态

- BMI088 传感器初始化与持续采样
- 基于四元数 EKF 的姿态解算
- 输出 `Yaw / Pitch / Roll / 连续 yaw`
- IMU 加热闭环控制

### 云台控制

- yaw / pitch 双轴闭环控制
- `Gyro` 模式与 `Aim` 模式切换
- 上电归中与前后半区判断
- 遥控器和键鼠两套输入路径
- 与视觉数据联动的自瞄参考值更新

### 发射机构

- 双摩擦轮速度控制
- 拨弹电机速度 / 位置混合控制
- 普通发射、连续发射、准备态、卡弹反转
- 基于裁判系统热量数据的射频限制逻辑

### 通信与状态管理

- USB CDC 向上位机发送 IMU、弹速、颜色等数据
- USB CDC 接收视觉侧目标数据
- CAN1 接收拨弹和摩擦轮反馈
- CAN2 接收云台电机反馈以及下板状态数据
- 看门狗维护遥控器、IMU、电机、下板、PC、裁判系统在线状态

## 关键数据流

### 视觉链路

`MiniPC -> USB CDC -> ReceiveVisionData -> Gimbal / Shoot`

- 云台在 `gAim` 模式下使用视觉提供的目标 yaw / pitch
- 发射逻辑根据 `FireFlag` 决定是否进入自动射击

### 上下板 CAN 链路

`MainCtrl_Task -> CAN2(0x120/0x130) -> 下板`

- `0x120`：发送 `Gimbal_action`
- `0x130`：发送 `Gimbal_data`

`下板 -> CAN2(0x101/0x102) -> 云台板`

- `0x101`：比赛状态、热量相关信息
- `0x102`：底盘速度、弹速等数据

### 发射链路

`Shoot_Task -> PID -> CAN1(0x200) -> M2006 / RM3508`

## 开发与构建

### 开发环境

- IDE: `Keil MDK-ARM`
- Project: `MDK-ARM/C_Board_Standard_Robot.uvprojx`
- MCU: `STM32F407IGH6`
- Middleware: `FreeRTOS`, `USB Device CDC`

### 建议阅读顺序

如果要快速理解工程，建议按这个顺序看：

1. `Src/main.c`
2. `Src/freertos.c`
3. `Init_Ctrl/Init_Task.c`
4. `Init_Ctrl/MainCtrl_Task.c`
5. `Init_Ctrl/ins_task.c`
6. `Agency/Gimbal/Gimbal.c`
7. `Agency/Shoot/Shoot.c`
8. `Agency/USB/USB_Task.c`
9. `Agency/WatchDog/Callback_Function.c`

## 代码审查结论

下面的问题按“影响范围 + 触发概率 + 修复收益”排序。  
这部分只基于当前代码静态审查，不代表所有问题都已在实机上复现。

### P1 高优先级

1. **USB 设备被重复初始化**

   - 位置：
     - `Src/freertos.c:131`
     - `Src/freertos.c:153`
   - 现象：`StartINSTask()` 和 `StartInit_Task()` 都调用了 `MX_USB_DEVICE_Init()`
   - 风险：USB CDC 栈初始化顺序不稳定，可能导致枚举异常、状态覆盖或偶发通信故障
   - 建议：
     - 统一 USB 初始化入口，只保留一次 `MX_USB_DEVICE_Init()`
     - 更稳妥的做法是放到系统统一初始化阶段，而不是放在两个任务入口里

2. **USB 接收链路没有真正消费“最新收到的数据”**

   - 位置：
     - `Agency/USB/USB_Task.c:36`
     - `Src/usbd_cdc_if.c:262`
   - 现象：
     - `usb_task` 里每 1ms 主动调用 `USB_Receive(data_buffer, &actual_len)`
     - `USB_Receive()` 实际只是再次调用 `CDC_Receive_FS()`，重新设置接收缓冲区
     - `CDC_Receive_FS()` 没有把 USB 中断收到的数据复制到任务层的 `data_buffer`
   - 风险：
     - `ReceiveVisionData` 很可能长期在读旧数据、未初始化数据，或者依赖偶然的内存内容
     - 自瞄和自动射击链路会变得不可预测
   - 建议：
     - 改为“USB 中断/回调写入缓冲区，任务只读缓冲区”的模型
     - 增加长度校验、帧头帧尾校验和数据有效标志
     - 使用双缓冲或消息队列，避免任务与 USB 回调直接竞争同一块内存

3. **CAN2 的 `0x102` 数据解析把两个字段都拷贝成了同一个 float**

   - 位置：`Agency/Gimbal/Gimbal.c:323`
   - 现象：
     - `Chassis_data_Rx.Chassis_Speed` 从 `CAN2_buff` 拷贝
     - `Chassis_data_Rx.bullet_speed` 也从 `CAN2_buff` 同一地址拷贝
   - 风险：
     - `bullet_speed` 实际上会和 `Chassis_Speed` 相同
     - 上位机发送的 IMU/弹速数据失真，后续弹道计算或日志分析会被误导
   - 建议：
     - 第二个字段应从 `CAN2_buff + 4` 读取
     - 同时补充对 `0x102` 帧格式的注释，避免上下板协议漂移

### P2 中优先级

4. **云台发送限幅写到了错误下标，实际输出没有被正确限幅**

   - 位置：
     - `Agency/Gimbal/Gimbal.c:243`
     - `Agency/Gimbal/Gimbal.c:279`
     - `Components/Variate.h:50`
   - 现象：
     - `GIMBAL_SUM = 2`
     - `limit(Can2Send_Gimbal[GIMBAL_SUM], ...)` 和 `limit(Can2Send[GIMBAL_SUM], ...)` 限的是数组第 3 个元素
     - 实际发送的是 `[PITCH]` 和 `[YAW]`
   - 风险：
     - 限幅逻辑表面存在，实际对真实输出通道不生效
     - PID 输出过大时可能直接打到电机
   - 建议：
     - 分别对 `Can2Send_Gimbal[PITCH]`、`Can2Send_Gimbal[YAW]` 单独限幅
     - 初始化归中阶段也同样处理

5. **USB 发送采用忙等重试，会直接侵占实时任务周期**

   - 位置：`Src/usbd_cdc_if.c:320`
   - 现象：`USB_Transmit()` 在 `USBD_BUSY` 时最多空转重试 `8192` 次
   - 风险：
     - `usb_task` 是 1ms 周期任务，忙等会造成任务抖动
     - 间接影响云台和发射实时性
   - 建议：
     - 改为非阻塞发送
     - 必要时增加发送队列或仅保留最新一帧
     - 发送失败时做丢帧统计，而不是死等

6. **热量控制中的 `shoot_time` 会无条件自减，存在无符号下溢风险**

   - 位置：`Agency/Shoot/Shoot.c:263`
   - 现象：`shoot_time` 是 `uint16_t`，不满足条件时直接 `shoot_time--`
   - 风险：
     - 刚上电或长时间不射击时会从 `0` 下溢到 `65535`
     - 影响射频估算公式，导致限热策略异常
   - 建议：
     - 自减前做下界保护
     - 直接改成“按时间戳计算窗口平均值”会更稳

### P3 低优先级

7. **总控任务每 15ms 反复 `suspend/resume` 其他任务，调度策略偏重**

   - 位置：`Init_Ctrl/MainCtrl_Task.c:10`
   - 风险：
     - 状态切换频繁时容易引入时序边界问题
     - 任务恢复后内部状态可能与外部状态不同步
   - 建议：
     - 改成状态机控制，让任务自行检查“允许运行”标志
     - 只在离线状态发生变化时再执行挂起/恢复

8. **公共状态大量裸露为全局变量，任务与中断共享边界不清晰**

   - 典型对象：
     - `DeviceState`
     - `ReceiveVisionData`
     - `Referee_data_Rx`
     - `Chassis_data_Rx`
     - `IMU`
   - 风险：
     - 当前能跑，但后续扩展时容易出现竞态、脏读和难以复现的状态问题
   - 建议：
     - 逐步引入“单写多读”的所有权约束
     - 对跨任务/中断共享的数据使用双缓冲、消息队列或时间戳快照

9. **仓库中混入大量 Keil 构建产物，影响代码阅读和版本管理**

   - 位置：`MDK-ARM/C_Board_Standard_Robot/`
   - 风险：
     - 提交噪声大
     - 审查时容易把产物误判成源码变化
   - 建议：
     - 将 `.o/.d/.axf/.map/.htm/.hex` 等产物加入忽略规则
     - 保留必要的工程文件，清理中间文件目录

## 建议的修复顺序

如果按投入产出比排序，建议优先处理：

1. USB 重复初始化
2. USB 接收链路重构
3. `0x102` CAN 数据解析错误
4. 云台输出限幅错误
5. 发射热量时间计数下溢
6. 调度与全局状态管理重构

## 后续可以补充的文档

当前 README 仍然缺少以下内容，如果后续继续完善会更适合交接和调参：

- 硬件连接图和接口定义
- CAN ID / USB 帧格式 / 遥控映射表
- 云台零点、限位和 PID 参数说明
- 发射机构减速比、拨盘角度和热量策略说明
- Keil 编译、下载、调试步骤

## 说明

- 本 README 基于当前仓库代码静态整理
- 本次仅更新文档，不修改任何源码逻辑
- 由于仓库当前存在未提交代码变更，以上结论以当前工作区状态为准
