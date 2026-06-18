# ESP32-C3 循线小车固件

本项目通过串口指令控制小车底盘，并通过 VGTI/UART 读取八路红外循线传感器。

## 需要配置什么

### 硬件

- 使用 ESP32-C3 开发板。
- USB-C 串口用于烧录、日志和恢复。
- 一路 UART：ESP32-C3 → 小车底盘指令口。
- 一路 UART：ESP32-C3 → 八路红外循线传感器 VGTI/UART 口。
- ESP32、底盘控制器、传感器模块必须共地。
- UART 逻辑电平为 3.3V；外设若使用 5V 逻辑需加电平转换。
- 电源做好隔离，避免电机电源噪声复位 ESP32 开发板。
- Wi-Fi SoftAP 调试通道用于无线参数调试。
- 预留 BLE 调试传输边界，供后续近距离调参使用。

### ESP32-C3 接线

| 用途 | DCar 端口 | ESP32-C3 外设 | ESP TX | ESP RX | 波特率 |
| --- | --- | --- | --- | --- | --- |
| 底盘指令串口 | 3 号串口（UART3） | UART1 | GPIO4 | GPIO5 | 115200 |
| 八路循线传感器 | — | UART0 | GPIO2 | GPIO3 | 115200 |

控制台走 USB Serial/JTAG，因此 ESP 的 `UART0` 保留给循线传感器。

> 注意命名：底盘指令线一端插在 **DCar 底盘的 3 号串口**（底盘侧称 **UART3**，波特率 115200；底盘其他串口为 460800），另一端由 **ESP32-C3 的 UART1 外设**（GPIO4/5）处理。ESP32-C3 整颗芯片只有 UART0/UART1，没有 UART3，故板级配置中的 `UART_NUM_1` 指的是 ESP 端外设，不要误改为 UART3。

### 板级配置

所有与芯片相关的 UART 和 GPIO 选择都集中在 `components/board/board_profile.c`。

控制、底盘、传感器模块必须继续使用 `board_profile_t`，不要硬编码引脚。如果 ESP32-C3 板无法干净地引出两路独立 UART，只需修改板级配置（board profile），或新增一个外部 UART 桥接配置。

## 当前协议

- 底盘输出使用 DFLink `Motion_Velocity`：帧头 `0xDF`、目标 `0x01`、源 `0x97`、A=`0x02`、B=`0x62`、LEN=`12`，载荷 `Vx,Vy,Vz` 为定点 F32。
- 循线传感器输入使用 USART 数字模式。固件发送 `$0,0,1#`，并解析 `$D,x1:0,...,x8:0#`。

协议参考文件保存在 `references/protocols/`。

八路循线传感器优先采用 USART 数字模式：只需两个引脚、协议完整已知、且适配 ESP32-C3 紧张的引脚资源。IO 直接读取需要八个 GPIO；I2C 在确认其地址/寄存器协议后可作为未来选项。

## 无线调试脚手架

固件以 `DCtrl` 名称进行 BLE 广播。可选的 Wi-Fi 后备仍会启动名为 `DCar-Liner-XXXXXX`、密码 `DCar-Liner123` 的 SoftAP。

电脑连接该热点后：

- 打开 `http://192.168.4.1/` 查看精简调试首页。
- 从 `http://192.168.4.1/api/schema` 读取参数定义。
- 从 `http://192.168.4.1/api/params` 读取当前参数。
- 从 `http://192.168.4.1/api/telemetry` 读取当前遥测。
- 使用 WebSocket `ws://192.168.4.1/ws` 进行实时调参。

WebSocket 消息示例：

```json
{"type":"get_schema"}
```

```json
{"type":"set_param","id":"pid.kp","value":7500}
```

```json
{"type":"save_params"}
```

实时更新会立即改变 RAM；只有在 `save_params` 之后才会写入 Flash 持久化。
