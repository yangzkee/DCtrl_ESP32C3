# DF Link 通信协议

## 基础信息

- 导出视图：完整版
- 数据来源：飞书多维表真相源同步快照
- 更新时间：2026-05-28T05:03:55.060Z
- 协议章节数：19
- 协议指令数：94

本文档用于让人或 AI 助手快速理解 DFLink 串口通信协议。按第二章总表取出某条指令的 A、B、LEN 和数据字段格式后，即可按第一章的通用帧规则组包发送。

## 第一章：通用帧与编码说明

DFLink 使用固定帧结构承载上位机到设备的控制、设置、访问和回传数据。读一条指令时，先确认 A、B、LEN，再按数据字段格式填 payload。

| 字节位置 | 字段 | 含义 |
|---|---|---|
| [0] | 0xDF | 帧头 |
| [1] | target_id | 目标设备地址，例如 0x01 表示 1 号车 |
| [2] | source_id | 发送方地址，例如上位机常用 0x97 |
| [3] | A | 指令大类，例如控制、运动、设置、访问或回传 |
| [4] | B | 大类下的具体子命令 |
| [5] | LEN | payload 的字节长度；LEN=0 表示没有额外数据段 |
| [6..] | payload | 按总表“数据字段格式”逐项写入的数据段 |
| [6+LEN] | 0xFD | 帧尾 |
| [7+LEN] | sumL | 16 位累加校验低字节 |
| [8+LEN] | sumH | 16 位累加校验高字节 |

校验和从帧头 0xDF 累加到帧尾 0xFD，不包含 sumL/sumH 本身；发送时低字节在前。

| 类型 | 字节数 | 编码规则 |
|---|---:|---|
| U8 | 1 | 无符号单字节，直接写入 payload |
| S8 | 1 | 有符号单字节，按补码写入 payload |
| U16 | 2 | 无符号 16 位整数，小端序：低字节在前 |
| S16 | 2 | 有符号 16 位整数，小端序：低字节在前 |
| F16 | 2 | 定点数：业务值乘以 100 后按 int16 小端序写入；接收后除以 100 |
| F32 | 4 | 定点数：业务值乘以 10000 后按 int32 小端序写入；接收后除以 10000 |

组帧示例：如果某条指令 A=0x01、B=0x28、LEN=1、payload 为 U8 开关值 1，则核心字段按 `DF target_id source_id 01 28 01 01 FD sumL sumH` 组织。实际发送时替换 target_id/source_id，并计算最后两个校验字节。

## 第二章：协议指令总表

| 名称 | 说明 | A | B | LEN | 数据字段格式 | 可见性 |
|---|---|---|---|---|---|---|
| 蜂鸣器控制 | 0=关,1=开,11~13短叫1~3声,21~23鸣叫300/600/900ms - 常用值: `0` 关，`1` 开，`11~13` 短叫 1~3 声。 - 定长鸣叫: `21/22/23` 约 `300/600/900ms`。 常用值: 0 关， 1 开， 11~13 短叫 1~3 声。 定长鸣叫: 21/22/23 约 300/600/900ms 。 | 0x01 | 0x28 | 1 | [1] Date1: U8 / 业务值 | 公开 |
| IMU校准 | 校准前离地静止；旧表常见u8=1 - 操作前让轮子离地并保持静止。 - 旧表常写 `u8=1`，联调时按当前固件口径发送即可。 操作前让轮子离地并保持静止。 旧表常写 u8=1 ，联调时按当前固件口径发送即可。 | 0x01 | 0x3C | 0 | 无额外 payload；LEN=0。 | 公开 |
| 水平校准 | 触发水平校准 | 0x01 | 0x3D | 0 | 无额外 payload；LEN=0。 | 公开 |
| 磁力计校准 | 触发磁力计校准 | 0x01 | 0x3E | 0 | 无额外 payload；LEN=0。 | 公开 |
| 遥控校准 | 触发遥控校准 | 0x01 | 0x3F | 0 | 无额外 payload；LEN=0。 | 公开 |
| 电调校准 | 触发电调校准 | 0x01 | 0x40 | 0 | 无额外 payload；LEN=0。 | 公开 |
| 默认校准 | 恢复默认校准 | 0x01 | 0x41 | 0 | 无额外 payload；LEN=0。 | 公开 |
| 模式切换 | 工作模式切换 | 0x01 | 0x42 | 1 | [1] Mode: U8 / 业务枚举 | 公开 |
| 无头模式 | 0=关,1=开 | 0x01 | 0x43 | 1 | [1] Mode: U8 / 0或1 | 公开 |
| 轮速校准 | 轮速校准触发 | 0x01 | 0x44 | 0 | 无额外 payload；LEN=0。 | 公开 |
| 锁头模式 | 默认1；建议上电阶段设置 - 默认常开，通常建议在上电阶段配置。 - 是否掉电保存以当前产品实现为准。 默认常开，通常建议在上电阶段配置。 是否掉电保存以当前产品实现为准。 | 0x01 | 0x4F | 1 | [1] Enable: U8 / 0或1 | 公开 |
| 锁定状态 | 锁定状态 | 0x01 | 0x50 | 1 | [1] Enable: U8 / 0或1 | 公开 |
| 航点模式 | 航点模式 | 0x01 | 0x59 | 1 | [1] Mode: U8 / 业务枚举 | 公开 |
| 返回模式 | 0=经典返航,1=轨迹返航 - `0` 经典返航。 - `1` 轨迹返航。 0 经典返航。 1 轨迹返航。 | 0x01 | 0x5A | 1 | [1] Mode: U8 / 业务枚举 | 公开 |
| 降落状态 | 速度参数 - 该参数常被上位机当作降落速度。 该参数常被上位机当作降落速度。 | 0x01 | 0x5B | 2 | [1] SDate1: F16 | 公开 |
| 起步状态 | 速度参数 - 该参数常被上位机当作起步或起飞速度。 该参数常被上位机当作起步或起飞速度。 | 0x01 | 0x5C | 2 | [1] SDate1: F16 | 公开 |
| 刹车状态 | 刹车速度等级 - 该参数常被上位机当作刹车速度等级。 该参数常被上位机当作刹车速度等级。 | 0x01 | 0x5D | 2 | [1] SDate1: F16 | 公开 |
| 翻滚状态 | 滚动类参数 | 0x01 | 0x5E | 4 | [1] SDate1: F16；[2] SDate2: F16 | 公开 |
| 渐远 / 漂移 | 未生效 | 0x01 | 0x5F | 0 | 无额外 payload；LEN=0。 | 公开 |
| 冲天 | 未生效 | 0x01 | 0x60 | 0 | 无额外 payload；LEN=0。 | 公开 |
| Motion_Velocity | 三轴速度控制；Vx/Vy 为平动速度，当前底盘实测 Vz 为每条指令的 Z 轴角度增量。 - Payload 为 `Vx, Vy, Vz` 三个 F32。 - `Vx/Vy` 单位为 m/s，`Vz` 按 angle increment/cmd 使用。 Payload 为 Vx, Vy, Vz 三个 F32。 | 0x02 | 0x62 | 12 | [1] Vx: F32 / m/s；[2] Vy: F32 / m/s；[3] Vz: F32 / angle increment/cmd | 公开 |
| Motion_Rotate | 原地旋转；dyaw_rad 为相对偏航角，omega_max_rad_s 为最大角速度。 - Payload 为 `dyaw_rad, omega_max_rad_s` 两个 F32。 - yaw 按逆时针为正，即 CCW+。 Payload 为 dyaw_rad, omega_max_rad_s 两个 F32。 yaw 按逆时针为正，即 CCW+。 | 0x02 | 0x63 | 8 | [1] dyaw_rad: F32 / rad；[2] omega_max_rad_s: F32 / rad/s | 公开 |
| Motion_Linear | 保持航向不变平移；Profile 可选，LEN=12 时默认 1。 - Payload 为 `Px, Py, Speed [, Profile]`。 - `Profile=1` 表示梯形加减速；`0` 或其它值表示匀速。 Payload 为 Px, Py, Speed [, Profile] 。 Profile=1 表示梯形加减速； 0 或其它值表示匀速。 | 0x02 | 0x64 | 12或13 | [1] Px: F32 / m；[2] Py: F32 / m；[3] Speed: F32 / m/s；[4] Profile: U8 / 可选，LEN=12时默认1 | 公开 |
| Motion_LinearWithYaw | 带相对 yaw 的平移；Profile 可选，当前固件解析该字段但行为固定为匀速。 - Payload 为 `Px, Py, dyaw_rad, Speed [, Profile]`。 - `dyaw_rad` 单位为 rad；Profile 字段保留但当前固件固定匀速。 Payload 为 Px, Py, dyaw_rad, Speed [, Profile] 。 dyaw_rad 单位为 rad；Profile 字段保留但当前固件固定匀速。 | 0x02 | 0x65 | 16或17 | [1] Px: F32 / m；[2] Py: F32 / m；[3] dyaw_rad: F32 / rad；[4] Speed: F32 / m/s；[5] Profile: U8 / 可选，当前固件固定匀速 | 公开 |
| Motion_Arc | 圆弧运动；R_m 为半径，dyaw_rad 为转角，Speed 为线速度。 - Payload 为 `R_m, dyaw_rad, Speed [, Profile]`。 - Speed 可为负数，负数表示倒车；Profile 字段保留但当前固件固定匀速。 Payload 为 R_m, dyaw_rad, Speed [, Profile] 。 Speed 可为负数，负数表示倒车；Profile 字段保留但当前固件固定匀速。 | 0x02 | 0x66 | 12或13 | [1] R_m: F32 / m；[2] dyaw_rad: F32 / rad；[3] Speed: F32 / m/s；[4] Profile: U8 / 可选，当前固件固定匀速 | 公开 |
| 速度控制 | X/Y平动速度，Z常作Yaw增量 - `X/Y` 常作平动速度。 - `Z` 常作 `Yaw` 增量或旋转控制量。 X/Y 常作平动速度。 Z 常作 Yaw 增量或旋转控制量。 | 0x02 | 0x67 | 6 | [1] Vx: F16 / 速度单位；[2] Vy: F16 / 速度单位；[3] Vz: F16 / 角度增量或速度类量 | 公开 |
| DriveUnitSpeed | 8 路直接速度控制 | 0x02 | 0x68 | 8 | [1] Unit1..Unit4: U8 * 4 / 0~255；[2] Unit5..Unit8: U8 * 4 / 0~255 | 公开 |
| DriveUnitPWM | 8 路 PWM 控制 | 0x02 | 0x69 | 16 | [1] Unit1 + Unit2: F16 + F16；[2] Unit3 + Unit4: F16 + F16；[3] Unit5 + Unit6: F16 + F16；[4] Unit7 + Unit8: F16 + F16 | 公开 |
| DJUnit | 舵机类控制 | 0x02 | 0x6A | 5 | [1] Channel: U8；[2] Angle: F32 / deg或业务量 | 公开 |
| FOCUnit | FOC 通道控制 | 0x02 | 0x6B | 5 | [1] Channel: U8；[2] Angle: F32 / deg或业务量 | 公开 |
| CircularVMove | 命令项；勿与 A=0x6C 回传混淆 - 这是运动命令，不是实时回传。 - 不要和 `A=0x6C` 的实时数据回传混淆。 这是运动命令，不是实时回传。 不要和 A=0x6C 的实时数据回传混淆。 | 0x02 | 0x6C | 10 | [1] ReferX + ReferY: F16 + F16；[2] RotaSpeed + VX: F16 + F16；[3] VY: F16 | 公开 |
| Battery | 电池相关 | 0x04 | 0x64 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x73: 历史上存在编码冲突 | 公开 |
| ImuRaw | 原始 IMU | 0x04 | 0x65 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x65: 按协议定义填写 | 公开 |
| IMUFit | 滤波 IMU | 0x04 | 0x66 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x66: 按协议定义填写 | 公开 |
| MagRaw | 原始磁力计 | 0x04 | 0x67 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x67: 按协议定义填写 | 公开 |
| MagFit | 滤波磁力计 | 0x04 | 0x68 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x68: 按协议定义填写 | 公开 |
| BarRaw | 原始高度 | 0x04 | 0x69 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x69: 按协议定义填写 | 公开 |
| EuraAngle | 欧拉角 | 0x04 | 0x6A | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x6A: 按协议定义填写 | 公开 |
| ObrHeight | 观测高度 | 0x04 | 0x6B | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x6B: 按协议定义填写 | 公开 |
| VisHeight | 视觉高度 | 0x04 | 0x6C | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x6C: 按协议定义填写 | 公开 |
| Altitude | 融合高度 | 0x04 | 0x6D | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x6D: 按协议定义填写 | 公开 |
| B_Acc | 车体系加速度 | 0x04 | 0x6E | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x6E: 按协议定义填写 | 公开 |
| N_Acc | 世界系加速度 | 0x04 | 0x6F | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x6F: 按协议定义填写 | 公开 |
| B_Vel | 车体系速度 | 0x04 | 0x70 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x70: 按协议定义填写 | 公开 |
| N_Vel | 世界系速度 | 0x04 | 0x71 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x71: 按协议定义填写 | 公开 |
| B_Pos | 车体系位置 | 0x04 | 0x72 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x72: 按协议定义填写 | 公开 |
| N_Pos | 世界系位置 | 0x04 | 0x73 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x73: 按协议定义填写 | 公开 |
| Time/Odom | 时间戳 IMU 综合包 | 0x04 | 0x80 | 2 | [1] VisitType: U8 / 1或2；[2] FreqCode: U8 / 频率码；[3] A=0x6C, B=0x80: 按协议定义填写 | 公开 |
| ID | 固定访问类型 | 0x0B | 0x01 | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| Shake_Hand | 历史：识别代码1为标准版本。100为专业版本，（在25.10.9更新），新版本规范如下：10为迷你Mini版本，50为标准Std版本，100为Pro版本， | 0x0B | 0x02 | 4 | [1] 识别代码: U8；[2] 握手代码1: U8；[3] 握手代码2: U8；[4] 握手代码3: U8 | 加密 |
| Version | 固定访问类型 | 0x0B | 0x03 | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| Prog_State | 固定访问类型 | 0x0B | 0x06 | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| RobotClass | 固定访问类型 | 0x0B | 0x0C | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| RobotAttri | 固定访问类型 | 0x0B | 0x0F | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| RobotUse | 固定访问类型 | 0x0B | 0x10 | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| UpdateTim | 固定访问类型 | 0x0B | 0x13 | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| Parm_in1 | 固定访问类型 | 0x0B | 0x3C | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| Parm_in2 | 固定访问类型 | 0x0B | 0x3D | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| Parm_in3 | 固定访问类型 | 0x0B | 0x3E | 2 | [1] 访问类型: 0x02；[2] 访问频率: U8 | 加密 |
| RobotParam | 访问小车的电机，编码器，遥控器类型等参数 | 0x0B | 0x3F | 2 | [1] 访问类型: 0x02；[2] 访问频率: 0x00 | 加密 |
| ActivateState | 访问小车的激活状态 | 0x0B | 0x40 | 2 | [1] 访问类型: 0x02；[2] 访问频率: 0x00 | 加密 |
| 系统时间同步 | 系统时间同步 | 0x07 | 0x88 | 8 | [1] Int: U32；[2] Dec: U32 | 公开 |
| 波特率设置 | 波特率设置 | 0x07 | 0x14 | 2 | [1] BaudRate: U16 | 公开 |
| 设备ID设置 | 建议取值 1~6 | 0x07 | 0x4E | 1 | [1] RobotID: U8 / 1~6 | 公开 |
| 电机参数设置 | StepperGearRatio 为可选第4字节 | 0x07 | 0x4F | 3或4 | [1] MotorType + MotorRotas: U8 + U8；[2] EncoderRotas + StepperGearRatio: U8 + U8(可选) | 公开 |
| 轮型与轮径 | 轮型与轮径 | 0x07 | 0x50 | 3 | [1] WheelType: U8；[2] WheelDim: U16 / mm | 公开 |
| 蜂鸣器与OLED开关 | 0=关, 1=开；写入 Flash 持久化 - `BeepSwitch / OledSwitch` 都是 `0/1` 开关。 - 写入后持久化到 Flash。 BeepSwitch / OledSwitch 都是 0/1 开关。 写入后持久化到 Flash。 | 0x07 | 0x59 | 2 | [1] BeepSwitch: U8 / 0或1；[2] OledSwitch: U8 / 0或1 | 公开 |
| 遥控输入模式 | 1=SBUS(100k) 2=SBUS(38.4k/PS2) 3=CRSF/ELRS(420k)；写入 Flash 持久化，立即生效，切换成功短叫一声 - `1=SBUS100k`，`2=SBUS38.4k/PS2`，`3=CRSF/ELRS`。 - 写入 Flash 后会立即重配遥控串口。 1=SBUS100k ， 2=SBUS38.4k/PS2 ， 3=CRSF/ELRS 。 写入 Flash 后会立即重配遥控串口。 | 0x07 | 0x5B | 1 | [1] RCType: U8 / 1~3 | 公开 |
| Yaw漂移修改 | Yaw 漂移 / 补偿相关 | 0x09 | 0x6F | 6 | [1] DriftEstimates: F32；[2] FixEnable: U8 / 0或1；[3] CalculaEnable: U8 / 0或1 | 公开 |
| 单参数细调 | 单参数细调 | 0x09 | 0x6D | 16 | [1] Par1: F32；[2] Par2: F32；[3] Par3: F32；[4] Par4: F32 | 公开 |
| PTZ_Control | 本车对外部云台下发控制帧 | 0x0D | 0x50 | 12 | [1] 目标地址: 0x68；[2] A: 0x0D；[3] B: 80；[4] LEN长度: 12；[5] Yaw: F32；[6] Pitch: F32；[7] Roll: F32 | 公开 |
| 芯片 ID 回传 | 按本行 A/B/LEN 与数据字段格式组帧发送。 | 0x8C | 0x01 | 13 | [1] robot ID: U8；[2] ID1: U32；[3] ID2: U32；[4] ID3: U32 | 加密 |
| 激活回传 | 按本行 A/B/LEN 与数据字段格式组帧发送。 | 0x8C | 0x02 | 2 | [1] 状态进度: U8；[2] 提示信息: U8 | 加密 |
| 版本回传 | Eg：软件版本V3.21 激活状态 1 已经激活 2激活版本（MINI STD Pro） | 0x8C | 0x03 | 6 | [1] 软件版本: F16；[2] 激活状态: F16；[3] 激活版本: F16 | 加密 |
| 程序就绪提示 | ready + notice。 | 0x8C | 0x06 | 2 | [1] ready: U8；[2] notice: U8 | 加密 |
| 程序就绪提示 | 按本行 A/B/LEN 与数据字段格式组帧发送。 | 0x8C | 0x06 | 2 | [1] 状态进度: U8；[2] 提示信息: U8 | 加密 |
| 机器人类别 | DCar 系列为 0x01。 | 0x8C | 0x0C | 1 | [1] class: U8 | 加密 |
| 机器人类别 | 1 DCar系列 10:机械臂系列 | 0x8C | 0x0C | 1 | [1] 识别码: U8；[2] 激活信息: U8 | 加密 |
| 机器人属性 | 4 个属性字节。 | 0x8C | 0x0F | 4 | [1] attribute1..attribute4: U8 * 4 | 加密 |
| 机器人属性 | 按本行 A/B/LEN 与数据字段格式组帧发送。 | 0x8C | 0x0F | 4 | [1] 属性1: U8；[2] 属性2: U8；[3] 属性3: U8；[4] 属性4: U8 | 加密 |
| 使用情况 | 开关次数与使用时长。 | 0x8C | 0x10 | 8 | [1] switch_times: u32 LE；[2] usetime: u32 LE | 加密 |
| 使用情况 | 设计缺陷，无法启用 | 0x8C | 0x10 | 8 | [1] 开关次数: u32；[2] 使用时长: u32 | 加密 |
| 升级/优化次数 | 升级/优化次数。 | 0x8C | 0x13 | 1 | [1] updatetim: U8 | 加密 |
| 升级/优化次数 | 设计缺陷，无法启用 | 0x8C | 0x13 | 1 | [1] 升级次数: U8 | 加密 |
| 内参 1 | IMU1/IMU2/MAG，各 3 个 F16。 | 0x8C | 0x3C | 18 | [1] IMU1: F16 * 3；[2] IMU2: F16 * 3；[3] MAG: F16 * 3 | 加密 |
| 内参 1 | 保留,暂不启用 | 0x8C | 0x3C | 18 | [1] IMU1: F16*3；[2] IMU2: F16*3；[3] Mag: F16*3 | 加密 |
| 内参 2 | Control1/Control2/Control3，各 3 个 F16。 | 0x8C | 0x3D | 18 | [1] Control1: F16 * 3；[2] Control2: F16 * 3；[3] Control3: F16 * 3 | 加密 |
| 内参 2 | 按本行 A/B/LEN 与数据字段格式组帧发送。 | 0x8C | 0x3D | 18 | [1] Control: F16*3；[2] Control2: F16*3；[3] Control3: F16*3 | 加密 |
| 内参 3 | Reser1/Reser2/Reser3，各 3 个 F16。 | 0x8C | 0x3E | 18 | [1] Reser1: F16 * 3；[2] Reser2: F16 * 3；[3] Reser3: F16 * 3 | 加密 |
| 内参 3 | 重大升级，IMU参数2，只有Pro版本才需要显示 | 0x8C | 0x3E | 18 | [1] Resv: F16*3；[2] Resv: F16*3；[3] Resv: F16*3 | 加密 |
| 小车参数 | 轮子、电机、遥控、蜂鸣器和 OLED 开关参数。 | 0x8C | 0x3F | 13 | [1] WheelType: U8；[2] WheelDim: U16；[3] Encoder: U16；[4] ReduceRatio: U16；[5] WheelRot: S8；[6] MotorRot: S8；[7] MotorDetailType: U8；[8] RCType: U8；[9] BeepSwitch: U8；[10] OledSwitch: U8 | 加密 |
| 小车参数 | 小车型号，全向轮，普通轮子，都在这里面！！电机的型号为具体的型号而不是原来这样的直流和无刷 | 0x8C | 0x3F | 13 | [1] 轮子（车型）类型: U8；[2] 轮子直径: U16；[3] 编码器线数: U16；[4] 电机减速比: U16；[5] 轮子极性（编码）: S8；[6] 电机极性: S8；[7] 电机类型: U8；[8] 遥控类型: U8；[9] 蜂鸣器开关: U8；[10] OLED开关: U8 | 加密 |
| 校准/微调状态 | IMUParm2 与 IMUTemp 在原始代码中使用 *10000 口径。 | 0x8C | 0x40 | 12 | [1] ifIMUcali: U8；[2] ifFinetune: U8；[3] IMUParm1: F16；[4] IMUParm2: s16 / raw code uses *10000；[5] IMUTemp: s16 / raw code uses *10000；[6] ControlFinetune: F16；[7] ControlFinetune2: F16 | 加密 |
| 校准/微调状态 | 重大升级，IMU参数2，只有Pro版本才需要显示，Temp是IMU校准时候的温度 | 0x8C | 0x40 | 12 | [1] 是否校准: U8；[2] 是否微调: U8；[3] IMU参数1: F16；[4] IMU参数2: F16；[5] IMUTemp: F16；[6] 控制微调1: F16；[7] 控制微调2: F16 | 加密 |
