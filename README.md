# DCtrl ESP32-C3 R5 黑线 / 白线循迹固件

当前发布 R5，内部应用版本为 `1.1.0-dev460800-r5-black/white`。两份镜像直接使用本次研发产物，保持与实际烧录文件一致。

| 固件 | 适用线路 |
| --- | --- |
| [R5 黑线版下载](https://differ-tech.pages.dev/dctrl-flasher/DCtrl_V1.1.0-dev460800-r5_black-line.bin) | 浅色背景上的黑线 |
| [R5 白线版下载](https://differ-tech.pages.dev/dctrl-flasher/DCtrl_V1.1.0-dev460800-r5_white-line.bin) | 深色背景上的白线 |

[网页烧录入口](https://differ-tech.pages.dev/portal/view/dctrl-flasher)支持选择颜色。均为ESP32-C3 / 4MB完整合并镜像，从 **0x0** 烧录，包含bootloader、分区表和应用；更新会清除旧NVS参数，PID和档位恢复默认。正式文件和SHA-256位于 `release-packages/`，产物不纳入Git。

## 接线

| 用途 | ESP TX | ESP RX | 波特率 |
| --- | --- | --- | --- |
| 底盘 UART1 | GPIO4 | GPIO5 | 460800 8N1 |
| 循迹传感器 UART0 | GPIO2 | GPIO3 | 115200 8N1 |

ESP TX接外设RX，ESP RX接外设TX，所有设备共地，UART为3.3V逻辑。USB Serial/JTAG用于日志与烧录。传感器x1在车辆左侧、x8在右侧。底盘对应串口必须设为460800。

## 当前运动策略

- 巡线实际线速度：一档0.02、二档0.05、三档0.10 m/s；转弯上限16/20/20°/s。
- 各命中光电管按位置 `w=-7,-5,-3,-1,1,3,5,7` 贡献 `sign(w)*(abs(w)/7)^2`，直接求和，限幅[-2,2]，不按管数求平均；同侧累加，左右抵消。无目标线仍进入独立摆扫找线。
- 现有小程序无需更新：遥控入口将旧平移值0.25/1/3连续径向映射至0.02/0.05/0.10 m/s，两个分量同比缩放，Wz及零速不变；巡线/手动/诊断不重复缩放。旧帧无档位编号，映射按速度大小计算。
- 最终底盘方向映射相对DHelper为 `(-Vx,+Vy,+Wz)`，仅执行一次。底盘协议为 `02/62/LEN12`、S32LE×10000，单位m/s、m/s、rad/s；无旧底盘经验速度倍率。
- 上电待机；BLE广播 `DCtrl`，小程序自检准备后启动巡线，显式停止/断连时停车。Wi-Fi后备通过BLE `W1`按需开启。

## 构建与验证记录

`python3 tools/build_line_firmwares.py` 为统一构建入口，产物在 `release-packages/`。本地研发可加 `--development` 输出到 `output/firmware-dev/`；开发构建仅编译打包可再加 `--build-only`，该模式会明确记录回归测试跳过。构建使用ESP-IDF v5.5.4与项目 `.idf-tools/`，累计300秒上限。

本次R5两版已编译并完成镜像完整性校验；黑线版已在新ESP32-C3烧录并确认启动版本。白线版未烧录，R5遵循用户要求未重跑回归或整车运动测试，不宣称两版均已完成实车验证。PID保留当前设定，首次使用从一档开始。

使用说明见 `docs/system-guide.html`；开发说明见 `docs/developer-guide.md`；策略和模块边界分别见 `docs/strategy-register.html` 与 `references/module-contracts.md`。
