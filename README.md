# DCtrl ESP32-C3 黑线 / 白线循迹固件

本项目读取八路红外循迹传感器，并通过 DF-Link 持续速度指令控制小车底盘。V1.1.0 提供两个固定颜色版本：

| 固件 | 适用线路 |
| --- | --- |
| [黑线版 V1.1.0 下载](https://differ-tech.pages.dev/dctrl-flasher/DCtrl_V1.1.0_black-line.bin) | 浅色背景上的黑线 |
| [白线版 V1.1.0 下载](https://differ-tech.pages.dev/dctrl-flasher/DCtrl_V1.1.0_white-line.bin) | 深色背景上的白线 |

两份均为 ESP32-C3 / 4MB 完整合并固件，**烧录地址为 `0x0`**，包含 bootloader、分区表与应用。烧录会清除旧参数；目标颜色不会被旧 NVS 设置覆盖。

[网页烧录入口](https://differ-tech.pages.dev/portal/view/dctrl-flasher)支持选择黑线或白线版本。本地构建产物位于 `release-packages/`，不纳入 Git。

## 构建与验证

```sh
python3 tools/build_line_firmwares.py
```

单入口在累计 300 秒内完成两版离线回归、ESP-IDF 编译、合并、完整性及版本校验，然后清理本项目旧固件和历史发布包。发布记录与 SHA-256 位于 `release-packages/manifest.json`。编译使用 ESP-IDF v5.5.4，`scripts/idf.sh` 优先使用项目 `.idf-tools/` 工具链。

黑色传感器值为 `0`，白色为 `1`；由 `DCTRL_LINE_COLOR=black|white` 选择目标颜色。两版共享纠偏、丢线找回和停车逻辑。底盘使用 `02/62/LEN12`，三个数值为 m/s、m/s、rad/s，S32 小端 ×10000，正转向为左转；角速度不按发送周期缩放。

## 接线与使用

| 用途 | ESP TX | ESP RX | 波特率 |
| --- | --- | --- | --- |
| 底盘 UART1 | GPIO4 | GPIO5 | 115200 |
| 循迹传感器 UART0 | GPIO2 | GPIO3 | 115200 |

ESP TX 接外设 RX，ESP RX 接外设 TX，所有设备共地，UART 使用 3.3V 逻辑。USB Serial/JTAG 用于控制台和烧录。传感器通道 x1 约定在车辆左侧、x8 在右侧。

上电待机；DHelper 小程序先自检准备，再启动循迹。BLE 默认广播 `DCtrl`，调参命令及启动/停止协议沿用原接口。Wi-Fi 诊断后备通过 BLE `W1` 开启。

速度档位和 PID 数值保留原设置，尚未在新版底盘实车标定。首次试车从一档开始，验证左右纠偏、直线、弯道、丢线找回和停车。编译或离线测试通过不代表实车验收通过。

使用说明：`docs/system-guide.html`。开发与协议说明：`docs/developer-guide.md`。当前策略：`docs/strategy-register.html`。
