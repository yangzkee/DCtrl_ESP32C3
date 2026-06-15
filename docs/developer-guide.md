# 开发者指南

## 当前固件形态

本项目是一套面向 ESP32-S3 循线小车的 ESP-IDF 固件骨架，其设计使得相同的模块边界日后可以迁移到 ESP32-C3。

固件包含以下核心模块：

- `board`：目标平台配置、UART 端口、引脚、波特率和缓冲区大小。
- `chassis`：向小车底盘输出串口运动指令。
- `line_sensor`：八通道红外循线传感器的串口输入解析器。
- `vehicle_state`：主运动状态、无线调参会话状态、故障原因以及手动测试超时。
- `control`：运动控制流水线，分为契约、输入归一化、循线解释、运动意图和执行输出。
- `param_store`：可在线调节的参数注册表以及 NVS 持久化。
- `debug_protocol`：用于回退 Wi-Fi/BLE 诊断的 JSON 请求/响应协议。
- `debug_server`：按需启动的 Wi-Fi SoftAP、HTTP API、WebSocket 端点和 BLE GATT 调试通道。
- `telemetry`：最新的循线传感器、循线 UART 诊断、运动指令、PID 输出以及参数版本快照。

## 功能模式归属

底盘 UART 是共享的物理资源。当前活动模式有 `SAFE_IDLE`、`REMOTE_BRIDGE`、`PARAM_TUNING`、`MANUAL_TEST`、`AUTO_ARMED`、`AUTO_RUNNING` 和 `FAULT`。只有当前活动模式才能向底盘写入运动字节。`REMOTE_BRIDGE` 被刻意设计得很薄：它把 DCtrl BLE 写入转发给 `chassis_uart_write_raw()`，不解析速度、不缓存目标、不转换偏航单位、不运行本地喂帧循环，也不在高速率路径上回送成功 ACK。远程桥接成功时保持静默，错误仍以 `E:*` 形式通知。桥接只更新用于现场诊断的观测遥测计数器，这些计数器绝不能影响运动行为。远程遥测会话的范围限定为一次连续的写入运行：BLE 断开会结束一个会话，而下一次写入开始前若出现超过 3000 ms 的间隔则启动一个新会话。

共享的远程运动坐标基准为：`X > 0` 前进、`Y > 0` 向左平移、`Z/Yaw > 0` 向左/逆时针旋转。DHelper 在编码 DFLink 之前会把摇杆、按键和运动控制输入映射到该坐标系；固件桥接代码不得再添加一层符号处理。

`line_trace_controller` 不得从 `app_main` 启动。它只在 `manual_motion`、`arm_auto` 或 `start_auto` 进入循线控制模式时启动，并在状态返回调参、空闲、远程桥接或故障时退出。启动路径必须在 `xTaskCreate()` 之前预留一个 `STARTING` 状态，以防并发的 BLE/Wi-Fi 请求创建出重复的控制器任务。已禁用和空闲的循线状态不得周期性发送 `chassis_uart_stop()`；只有显式停止、断开连接、手动测试超时或故障处理才可以发送一次性的零速帧。

未来功能在改动底盘 UART 归属之前必须遵循 `docs/vehicle-feature-mode-maintenance.md`。新功能在接入共享运动输出之前，必须先定义其目标、输入、输出、独立测试、失败规则和集成边界。

## 当前循线地图范围

当前项目地图刻意保持简单：地面上一个由黑色胶带贴成的不规则圆圈。其不规则性来自手工贴胶带的自然误差，而非刻意设计的迷宫。车辆只需保持在该回路上并持续循线即可。

当前运动代码的范围之外的内容：

- 迷宫求解。
- 分支或路口分类。
- 路线记忆。
- 在交叉口处选择左转/右转。
- 丢线后的搜索行为。

这些功能日后应作为新的策略状态或独立的路线规划器加入，不应混入当前的圆形循线策略。

## 运动控制模块契约

`components/control/line_trace_controller.c` 负责硬件集成：

- 从 `param_store` 读取在线参数。
- 通过 `line_sensor_uart` 读取循线传感器。
- 把 `vehicle_state` 映射为循线运行请求。
- 把最终指令发送给 `chassis_uart`。
- 发布遥测。

运动控制代码现已拆分为五层。第一次重构刻意保持行为不变；当前的策略改动仍应保持稳定的循线手感，除非本文档明确指出所改变的行为。

- `Layer 0: motion_contracts`：运动常量和规则的唯一真实来源，包括循线有效位质量、偏移曲线、速度档位映射、定时恢复扫描行为、速度保持比例以及通用钳位辅助函数。
- `Layer 1: motion_inputs`：把车辆/系统状态和原始传感器采样转换为归一化的策略输入。它执行诸如运行模式映射和可选传感器位反转等机械操作，但不判断线形含义。
- `Layer 2: line_interpreter`：把归一化的循线采样解释为几何事实，包括有效传感器数量、循线质量、丢线标志、曲线偏移误差以及简单的当前帧模式状态。
- `Layer 3: motion_policy`：把运行模式加上已解释的几何信息转化为运动意图。现有的阶段转换、PID 计算、自适应降速和丢线恢复状态机都在这里。
- `Layer 4: motion_executor`：把运动意图转化为现有的 `line_trace_policy_output_t` 契约。其输出携带一个共享的 `control_plan_t`；控制器仍是唯一实际执行车辆状态动作或向 `chassis_uart` 发送指令的层。

`control_plan_t` 是策略与执行之间共享的控制指令契约。它将底盘动作（`NONE`、`SEND_MOTION`、`STOP`）与车辆状态动作（`NONE`、`STOP_TO_IDLE`、`ENTER_FAULT`）分离开来。策略代码不得直接调用 `vehicle_state_stop()` 或 `chassis_uart_send_motion()`，只能填写该 plan。控制器是解释该 plan 的执行边界。

`components/control/line_trace_policy.c` 仍是围绕该流水线的稳定公共封装。它不访问 UART 或 NVS。其输入是运行请求、传感器结果、手动指令以及 PID/速度配置。其输出是循线阶段、共享控制 plan 和控制器遥测。

当前循线阶段：

- `STOPPED`：开机、空闲、调参、故障，或任何不允许运动的状态。
- `MANUAL_TEST`：调参会话中的短时手动指令。
- `ACQUIRE_LINE`：auto 已 armed，车辆保持停止，直到操作员显式启动。
- `TRACK_LINE`：带 PID 转向的正常圆形循线跟踪。
- `LINE_LOST`：运行中无有效循线位；策略持续发送 `linear_mm_s=0` 的 DFLink `Motion_Velocity`，并进行逐步扩大的定时左右搜索。固定序列为：左 `0.8 s`、右 `1.6 s`、左 `3.2 s`、右 `6.4 s`、左 `12.8 s`、右 `25.6 s`，最后右 `30.0 s`。若在最后一段之后仍未找回线，plan 请求 `STOP_TO_IDLE`。
- `SENSOR_FAULT`：传感器超时或解析/读取错误；运行期间，策略持续发送安全的零线速搜索指令，而不是立即进入故障。

丢线恢复刻意拆分为两路输出。运动输出是通过 `control_plan_t` 发出的安全恢复指令：`SEND_MOTION`，其中 `linear_mm_s=0`，`angular_mdeg_s` 设为当前档位的正常转向限值。档位 1/2/3 分别使用 `8000/10000/10000 mdeg/cmd`。诊断输出记录线回来时推断出的首个几何关系：`UNDERSTEER` 表示成功的恢复方向与最后一次有效跟踪转向一致，而 `OSCILLATION_SKEW` 表示线从相反方向回来。遥测会暴露定时扫描的 `segment_index`、`elapsed_ms` 和 `target_ms`。该关系目前仅作遥测用途，尚未改变 PID 增益、速度或未来的转向决策。

要修改循线行为，请按意图选择对应的层：

- 在 `motion_contracts` 中改动常量、速度档位规则、黑/白语义、循线质量阈值或协议级运动限值。
- 在 `motion_inputs` 中改动系统状态、传感器采样、未来 ODOM 或未来车辆位姿的缓存与归一化方式。
- 在 `line_interpreter` 中改动循线几何、当前帧模式分类或未来的 N 帧趋势分类。
- 在 `motion_policy` 中改动循线阶段、PID 行为、自适应速度策略、丢线搜索意图或车辆状态动作请求。
- 在 `motion_executor` 中改动与策略无关的最终输出整形，例如未来的斜坡或变化率限制。

请把 UART、BLE、NVS 和板级引脚细节排除在 `line_interpreter`、`motion_policy` 和 `motion_executor` 之外。

运动流水线模块定义：

- `motion_contracts`
  - `Goal`：把规范的运动规则集中到一处，使未来改动不会散落在控制器代码各处。
  - `Inputs`：档位号、循线位、偏移、有效传感器数量、通用数值。
  - `Outputs`：速度曲线、循线质量、曲线误差、自适应速度以及钳位后的值。
  - `Independent Test`：C3/S3 构建，以及对档位映射、循线质量和偏移曲线的确定性检查。
  - `Failure Rules`：无硬件访问、无持久状态、无副作用。
  - `Next Integration Boundary`：未来的协议和物理单位常量应首先进入这里。
- `motion_inputs`
  - `Goal`：把一切可能影响底盘运动的因素归一化为稳定的输入快照。
  - `Inputs`：`vehicle_state_snapshot_t`、原始 `line_sensor_sample_t`、传感器极性设置、未来的 ODOM/位姿/策略目标字段。
  - `Outputs`：`line_trace_policy_input_t`。
  - `Independent Test`：固定车辆状态必须映射到预期的运行模式；反转的传感器位必须重新计算偏移。
  - `Failure Rules`：不做线形含义判断，也不做 PID 运算。
  - `Next Integration Boundary`：未来的 ODOM 和全局位姿应作为归一化的缓存字段加入这里。
- `line_interpreter`
  - `Goal`：从归一化输入估计车辆与线之间的几何关系。
  - `Inputs`：归一化的运行/传感器输入。
  - `Outputs`：有效传感器数量、质量、丢线标志、曲线误差以及当前帧线模式状态。
  - `Independent Test`：固定的 `bits/offset` 采样必须产生稳定的质量、丢线以及左/右/中/丢的模式输出。
  - `Failure Rules`：不生成底盘指令。
  - `Next Integration Boundary`：未来的 N 帧模式识别属于这里，而不属于 PID 或 UART 驱动。
- `motion_policy`
  - `Goal`：决定车辆打算执行什么运动。
  - `Inputs`：运行模式、已解释的循线几何、PID 参数、速度曲线、运行时记忆。
  - `Outputs`：`motion_intent_t`，含阶段、`control_plan_t`、遥测字段以及可选的车辆状态动作。
  - `Independent Test`：固定的输入序列必须为 stop、armed、track、line-lost、sensor-fault 和 manual-test 各情形保持稳定的发布输出。
  - `Failure Rules`：不进行硬件发送，也不进行参数存储。
  - `Next Integration Boundary`：未来的路线策略可作为更高层的目标喂入本层。
- `motion_executor`
  - `Goal`：把运动意图转换为控制器输出契约。
  - `Inputs`：`motion_intent_t`。
  - `Outputs`：包含 `control_plan_t` 的 `line_trace_policy_output_t`。
  - `Independent Test`：逐字段拷贝应保留控制 plan 和遥测值。
  - `Failure Rules`：不直接进行 UART 发送；`line_trace_controller` 仍是唯一的硬件发送归属者。
  - `Next Integration Boundary`：未来与策略无关的平滑处理可加在这里。

循线质量主要受以下因素影响：

- 循线传感器黑色输出是否如预期为 `0`；`sensor.invert_bits` 仅用于输出相反的传感器模块。
- 传感器高度、对齐情况，以及黑色胶带对 8 通道阵列是否足够宽。
- 循环周期和传感器超时。
- 速度档位，它同时映射前进速度和转向限值。
- PID 增益和积分限值。
- 来自 `max_turn_mdeg_s` 的转向饱和。
- 丢线策略；当前行为是零线速的安全 Z 搜索，而 BLE
  断开仍是立即停止的条件。

## 所需外设

- 用于烧录、日志和恢复的 USB-C 串口连接。
- 底盘串口链路：ESP TX/RX 接到小车底盘的串口指令接口。
- 八通道红外循线传感器串口链路：ESP RX/TX 接到 VGTI/UART 接口。
- ESP32 板、底盘控制器和传感器模块之间的共地。
- 3.3 V 逻辑电平兼容，若任一外设使用 5 V UART 逻辑则需电平转换。
- 为 ESP32-S3、传感器和底盘控制器提供稳定的供电预算。电机电源不得倒灌进 ESP 模块。
- BLE 低功耗调试通道。这是默认的现场调参路径，并与 ESP32-C3 保持兼容。
- 按需启动的 Wi-Fi SoftAP 调试通道。它仍是回退诊断路径，默认不供电。

## 默认 ESP32-S3 配置

- 底盘 UART：`UART1`，TX `GPIO17`，RX `GPIO18`，`115200 8N1`。
- 循线传感器 UART：`UART2`，TX `GPIO15`，RX `GPIO16`，`115200 8N1`。

在确认确切的 ESP32-S3 开发板引脚分布和接线之前，这些引脚均为占位值。避免使用 strapping 引脚以及已被 USB、flash、PSRAM 或启动模式占用的引脚。

## ESP32-C3 迁移规则

业务代码不得依赖原始 GPIO 编号或 UART 端口号。所有与目标平台相关的选择都保留在 `components/board/board_profile.c` 中。

C3 配置当前预留：

- 底盘 UART：`UART1`，TX `GPIO4`，RX `GPIO5`。
- 循线传感器 UART：`UART0`，TX `GPIO2`，RX `GPIO3`。
- 控制台：主输出在 USB Serial/JTAG 上，因此 `UART0` 预留给循线传感器驱动。

UART 接线需交叉：ESP TX 接到外设 RX，ESP RX 接到外设 TX。

- 当前 C3 配置的底盘接线：`GPIO4` -> 底盘 RX，`GPIO5` <- 底盘 TX，外加共地 GND。
- 当前 C3 配置的循线传感器接线：`GPIO2` -> 循线传感器 RX，`GPIO3` <- 循线传感器 TX，外加共地 GND。

在 `/dev/cu.usbmodem11201` 上首次进行 C3 板冒烟测试，确认固件以 `esp32c3-portable-dual-uart-profile` 启动。C3 循线传感器接线后来从 `GPIO20/GPIO21` 移走，因为现场遥测显示循线 UART 读回的是 `$0,0,1#` 而非模块的 `$D,...#` 帧。未来的 C3 引脚改动请保持在 `components/board/board_profile.c` 内。

## ESP-IDF 环境

ESP-IDF 本地安装在 `/Users/schwartz/esp/esp-idf`，工具链和 Python 环境位于 `/Users/schwartz/.espressif` 下。

已安装并验证的版本：

```sh
ESP-IDF v5.5.4
```

该安装包含 `esp32s3` 和 `esp32c3` 两套目标工具链。下面的项目辅助脚本会加载 `export.sh`、进入项目根目录，并把参数转发给 `idf.py`：

```sh
./scripts/idf.sh --version
```

## 构建命令

```sh
./scripts/idf.sh set-target esp32s3
./scripts/idf.sh build
./scripts/idf.sh -p /dev/cu.usbserial-1120 flash monitor
```

当前 ESP32-S3 构建验证通过，并生成 `build/DCar-Liner.bin`。

固件对 S3 和 C3 构建都使用自定义的 4 MB 单 app 分区表 `partitions_single_4mb.csv`。只有一个 `factory` app 槽（`0x3B0000`）；空中升级（OTA）已被移除，因此固件只能通过 USB 或 web-serial 烧录更新。

更改分区表后，先通过 USB 烧录一次，让 bootloader 看到新布局：

```sh
./scripts/idf.sh -B build-esp32c3 -DSDKCONFIG=sdkconfig.esp32c3 -p /dev/cu.usbmodem11201 flash
```

## 无线调试 API

默认无线模式：

- BLE 默认启动，作为低功耗调参/调试入口。
- DCtrl 远程桥接作为透明的 BLE UART 服务启动。小程序发送 21 字节的 DFLink 帧；固件原样把 BLE 写入字节转发给底盘 UART。对于远程旋转，小程序负责业务级的 `Vz deg/cmd`，并在 BLE 写入前将其编码为 DFLink 线级的 `rad/cmd`。
- Wi-Fi SoftAP 默认关闭，以降低功耗。
- 紧凑 BLE 命令 `W1` 启动回退的 Wi-Fi SoftAP 和 HTTP/WebSocket 服务器。

`W1` 之后的回退 SoftAP：

- SSID：`DCar-Liner-XXXXXX`，后缀来自 ESP32 的 Wi-Fi MAC。
- 密码：`DCar-Liner123`。
- URL：`http://192.168.4.1/`。
- WebSocket：`ws://192.168.4.1/ws`。

HTTP 端点：

- `GET /api/health`：用于检查 HTTP 可达性、空闲堆和 HTTP 任务栈余量的小型健康检查。
- `GET /api/schema`：参数定义、范围、默认值、当前值。
- `GET /api/scheme`：`/api/schema` 的兼容别名。
- `GET /api/params`：当前参数值。
- `GET /api/telemetry`：当前循线传感器状态、循线 UART 诊断、运动指令、控制器状态以及参数版本。

`http://192.168.4.1/` 处的根调试页面只是一个回退诊断页面，提供指向各 API 端点的静态链接。正常的调参 UI 是通过紧凑 BLE 帧的微信小程序；Wi-Fi 仍是维护和回退诊断通道。

HTTP 调试服务器遵循 ESP-IDF 示例和成熟 ESP32 Wi-Fi 配置项目的稳定性模式：

- HTTP 服务器栈显式配置为 `8192` 字节。
- 大型 JSON 响应使用堆缓冲区，而非 HTTP handler 栈。
- 正常的 HTTP API 响应添加 no-cache、CORS 和 `Connection: close` 头。
- 服务器保持启用 LRU socket 清理，以便释放陈旧的浏览器连接。
- 应首先测试 `/api/health`；如果它反复正常但 `/api/schema` 失败，请检查响应大小或堆余量。

BLE 调试传输：

- 协议栈：NimBLE BLE GATT，而非经典蓝牙 SPP。这使固件与 ESP32-C3 兼容，后者支持 BLE 但不支持经典蓝牙。
- 设备名前缀：`DCtrl`。
- 默认运行时设备名：`DCtrl`。BLE 改名命令已禁用；`N=*` 仅清除旧的已存名称并确认固定名称。
- 服务 UUID：`7b3a0001-8d4d-4b9a-b5c7-0f7c4c415243`。
- RX 特征 UUID：`7b3a0002-8d4d-4b9a-b5c7-0f7c4c415243`，在此写入紧凑帧或 JSON 请求。
- TX 特征 UUID：`7b3a0003-8d4d-4b9a-b5c7-0f7c4c415243`，在此读取紧凑帧或 JSON 响应。
- TX 通知为可选项。启用时，固件在处理完请求后会发送一个小的 `ble_response_ready` 通知；客户端随后应读取 TX。
- 短 TX 读取直接返回 JSON 响应。
- 长 TX 读取返回一个或多个分块信封。每个信封包含 `type:"ble_chunk"`、`seq`、`offset`、`total`、`done` 和 `data`。客户端必须不断追加 `data` 直到 `done:true`，然后解析拼装好的 JSON。
- RX 流缓冲区的范围限定为单个 BLE 连接。固件在连接和断开时清除任何部分写入的紧凑/JSON 请求，使未完成的写入不会污染下一个请求或下一个客户端会话。

供 DHelper 小程序 `line-tuning` 功能使用的紧凑 BLE 帧：

- `G\n`：读取四个面向手机的参数。
- 响应：`P<kp>,<ki>,<kd>,<gear>\n`，例如 `P9000,0,0,2`。
- `S<kp>,<ki>,<kd>,<gear>\n`：写入并保存四个参数。
- `N\n`：读取固定的 BLE 广播名称。
- 响应：`NDCtrl\n`。
- `N=<name>\n`：已禁用的兼容命令；返回 `E:FIXED\n`。
- `N=*\n`：清除任何旧的自定义名称 NVS 键，并恢复/确认固定的 `DCtrl` 名称。
- 成功响应：`OK\n`。
- 错误响应：`E:PARSE\n`、`E:RANGE\n`、`E:FIXED\n`、`E:FAULT\n`、`E:BUSY\n` 或 `E:SAVE\n`。
- `W1\n`：启动回退的 Wi-Fi 调试服务器。

不要重新引入可编辑的 BLE 名称。小程序必须通过固定的 `DCtrl` 名称或 DCtrl 服务 UUID 扫描/连接。

DHelper 小程序的循线控制在同一组 RX/TX
特征上使用 BLE JSON 请求：

- `一键自检`：发送 `get_telemetry`，仅当遥测
  报告 `FAULT` 时才发送 `clear_fault`，然后发送 `arm_auto`。除非最终观测到的状态为
  `AUTO_ARMED`，否则启动按钮保持禁用。
- `开始循线`：发送 `start_auto` 并期待 `AUTO_RUNNING`。
- `停止`：发送 `stop`。

面向手机的参数刻意做了限制：

- `pid.kp`：`7000-11000`
- `pid.ki`：`0-300`
- `pid.kd`：`0-3000`
- `speed.gear`：`1-3`

`speed.gear` 同时映射协议线速度和每条指令的 Z 角度增量限值：

- 档位 1：最大协议线速度 `250 mm/s`（`Vx=0.25`），Z 限值 `8000 mdeg/cmd`
- 档位 2：最大协议线速度 `600 mm/s`（`Vx=0.6`），Z 限值 `10000 mdeg/cmd`
- 档位 3：最大协议线速度 `1000 mm/s`（`Vx=1.0`），Z 限值 `10000 mdeg/cmd`

自动循线期间，这些是最大线速度。当有效循线
位置偏离中央对 `x4/x5` 时，策略以先慢后快的二次曲线降低线
速度。在最外侧偏移处，它保持
大约所选档位速度的 45%。PID 转向输入采用相同的
先慢后快思路：原始偏移在进入 PID 计算前会被归一化并平方，
因此小偏移产生平缓的转向增量，而大
偏移则更激进地加大。

JSON BLE 请求流程示例：

1. 扫描 BLE 设备并连接到所选设备。
2. 向 RX 写入 `{"type":"get_telemetry"}`。
3. 读取 TX。若响应为 `ble_chunk`，持续读取并追加 `data` 直到 `done:true`。
4. 解析最终的 JSON 响应。

计算机端 BLE 验证：

```sh
swift tools/ble_debug_tester.swift --timeout 25
```

该测试工具直接使用 macOS CoreBluetooth。默认情况下它扫描以 `DCtrl` 开头的名称、连接、写入所选命令、跟随分块的 TX 响应、接受 JSON 或紧凑 BLE 响应，并把日志写入 `logs/ble-debug-tests/<timestamp>/`。仅在需要锁定某个确切 BLE 名称时才使用 `--name`。

紧凑帧示例：

```sh
swift tools/ble_debug_tester.swift --timeout 25 --command G --append-newline --expect-prefix P
swift tools/ble_debug_tester.swift --timeout 25 --command S9000,0,0,2 --append-newline --expect-prefix OK
```

WebSocket 消息：

```json
{"type":"get_schema"}
{"type":"get_params"}
{"type":"get_telemetry"}
{"type":"enter_tuning"}
{"type":"set_param","id":"pid.kp","value":7500}
{"type":"save_params"}
{"type":"reset_defaults"}
{"type":"exit_tuning"}
{"type":"arm_auto"}
{"type":"start_auto"}
{"type":"stop"}
{"type":"clear_fault"}
{"type":"manual_motion","motion":"forward","duration_ms":500}
```

车辆空闲或运行时允许只读调试消息。修改参数的消息采用“停车再调”策略：`set_param`、`save_params` 和 `reset_defaults` 仅在 `enter_tuning` 已把车辆切换到 `PARAM_TUNING` 之后才被接受。`save_params` 把当前值写入 NVS。不要在每次拖动滑块时都保存。

BLE 紧凑命令 `S<kp>,<ki>,<kd>,<gear>` 同样会短暂进入调参、校验四个面向手机的值、保存到 NVS、退出调参，并返回 `OK` 或错误。它不会重写固件。

车辆运动状态：

- `BOOT_INIT`：启动和外设初始化。
- `SAFE_IDLE`：默认安全状态；控制器发送停止指令。
- `PARAM_TUNING`：无线调参会话进行中；底盘保持停止。
- `MANUAL_TEST`：来自调参会话的短时低速手动底盘测试。
- `AUTO_ARMED`：自动循线已准备好但仍处于停止。
- `AUTO_RUNNING`：允许闭环循线发送运动指令。
- `FAULT`：底盘发送失败或安全状态非法；用 `clear_fault` 显式清除。正常丢线由搜索阶段处理，而不是进入 `FAULT`。

`manual_motion` 接受 `motion` 为 `forward`、`backward`、`left`、`right` 或 `stop`，或者接受数值型的 `linear_mm_s` 和 `angular_mdeg_s`。手动指令限制为 `1500 mm/s`、`10000 mdeg/cmd` 和 `50-3000 ms`；线速值作为 DFLink 协议速度值发送，而 Z 值是每条指令的角度增量。协议 `Vx=0.5` 是实测下舒适的手动测试速度，而自动循线目前上限为 `Vx=1.0`。

## 计算机端状态机测试工具

项目包含一个离线安全的 PC 测试运行器 `tools/state_machine_tester.py`。它只使用 Python 标准库，因此在计算机从正常 Wi-Fi 切换到 ESP32 SoftAP 并失去互联网访问之后仍可运行。

安全默认运行：

```sh
python3 tools/state_machine_tester.py
```

默认目标：

- 主机：`192.168.4.1`。
- HTTP：`/api/health`、`/api/telemetry`、`/api/params`、`/api/schema`。
- WebSocket：`/ws`。
- 日志根目录：`logs/state-machine-tests/`。

安全默认流程先发送一次预检 `stop`，然后验证 HTTP 可达性、WebSocket 可达性、停车再调写入门控、`enter_tuning`、一次低速 `manual_motion`、从 `MANUAL_TEST` 自动返回 `PARAM_TUNING`、`exit_tuning`、`arm_auto` 和 `stop`。它刻意跳过 `start_auto`，因此不应启动闭环行驶。

要包含 `AUTO_RUNNING` 转换，仅在车辆被抬起、被约束或物理上安全时运行：

```sh
python3 tools/state_machine_tester.py --include-auto
```

每次运行会创建：

- `logs/state-machine-tests/<timestamp>/events.jsonl`：每一次 HTTP/WebSocket 请求、响应、检查、错误和安全停止。
- `logs/state-machine-tests/<timestamp>/summary.md`：用于快速查看的通过/失败摘要。

`GET /api/telemetry` 包含：

- `vehicle.motion_state`
- `vehicle.debug_session`
- `vehicle.fault`
- `line.active_count`
- `line.quality`
- `motion.linear_mm_s`
- `motion.angular_mdeg_s`
- `remote_bridge.success`
- `remote_bridge.errors`
- `remote_bridge.session_id`
- `remote_bridge.session_success`
- `remote_bridge.last_gap_ms`
- `remote_bridge.max_gap_ms`
- `remote_bridge.last_error_code`
- `controller.line_phase`
- `controller.lost_line`
- `controller.pid_output_mdeg_s`

当前可调参数：

- `pid.kp`、`pid.ki`、`pid.kd`、`pid.integral_limit`
- `speed.gear`
- `control.base_speed_mm_s`
- `control.max_turn_mdeg_s`
- `control.loop_period_ms`
- `sensor.timeout_ms`
- `sensor.invert_bits`

进行迁移冒烟测试时，使用独立的 C3 构建目录和 SDK 配置，使当前的 ESP32-S3 配置保持完好：

```sh
./scripts/idf.sh -B build-esp32c3 -DSDKCONFIG=sdkconfig.esp32c3 set-target esp32c3 build
```

当前 ESP32-C3 冒烟构建验证通过，并生成 `build-esp32c3/DCar-Liner.bin`。

## 保留的协议参考

- `references/protocols/dflink-chassis-protocol-20260528.md`：DFLink 底盘 UART 的拷贝源文件。
- `references/protocols/line-sensor-8ch-protocol-from-screenshot.md`：从用户提供的截图中提取的循线传感器协议。
- `references/protocols/interface-selection.md`：当前接口决策。
- `references/wifi-debug-research-20260603.md`：30 个项目的 Wi-Fi 调试/配置调研以及采纳的稳定性模式。

## 底盘协议

底盘驱动发送 DFLink `Motion_Velocity` 作为连续的循线速度指令：

- 帧：`0xDF target source A B LEN payload 0xFD sumL sumH`。
- 目标 id：`0x01`。
- 源 id：`0x97`。
- A=`0x02`，B=`0x62`，LEN=`12`。
- 载荷：`Vx`、`Vy`、`Vz`，为 DFLink F32 定点值。
- `linear_mm_s` 是以 mm/s 表示的 DFLink 协议速度指令，而非
  保证的真实世界速度。`500 mm/s` 在 DFLink 载荷中发送 `Vx=0.5`。
- 当前现场标定估计为 `protocol speed : real speed = 10:1`。
  该比例用于日后解读实测底盘速度，而非在
  把值发送给底盘之前对其进行缩放。
- `Vy` 固定为 `0`。
- `angular_mdeg_s` 现被当作以毫度表示的每条指令 Z 角度增量，并映射为 DFLink `Vz` 的每条指令弧度值。底盘必须持续接收指令才能保持运动。

对于 DCtrl 遥控，固件不调用该转换路径。小程序是 DFLink 远程帧的真实来源，包括 `Vz deg/cmd` 到线级 `rad/cmd` 的转换，而 ESP32-C3 远程服务只转发原始字节。这保留了旧的蓝牙串口行为，并防止手机端和固件端的运动策略相互冲突。

官方固件也包含探针式底盘诊断，因此
临时的独立探针固件不再属于维护路径。
在 ESP32-C3 上，当前官方底盘配置为 `UART1 TX GPIO4 / RX GPIO5`，
波特率 `115200`，对应用户所选的 115200 底盘端口。BLE 紧凑
诊断命令为：

- `C?`：报告官方固件的底盘 UART 引脚和波特率。
- `Cready`：发送 DFLink ready/access 查询并解析任何响应帧。
- `Cversion`：发送 DFLink 版本查询并解析任何响应帧。
- `Cparams`：发送 DFLink 参数查询并解析任何响应帧。
- `Crepeat`：依次发送 ready、version 和 params 查询。
- `Codom`：以 10 Hz 请求里程计并短暂监听响应帧。
- `Cxyz`：发送一帧低速 `Motion_Velocity` 测试帧，`Vx=0.05 m/s`、
  `Vy=0`、`Vz=0.10 rad/cmd`；这主要验证 XYZ 速度指令路径，
  可能不产生响应。
- `Cxyzrx`：发送同样的低速 XYZ 帧，然后在官方 RX 引脚上
  短暂监听底盘返回字节。
- `Cxyz500`：发送 `linear_mm_s=500`、
  `angular_mdeg_s=0` 的协议线速度指令；这会发出 DFLink `Vx=0.5`、`Vy=0`、`Vz=0`。
- `Cxyz500rx`：发送同样的 `Vx=0.5` 线速度指令，然后短暂监听
  底盘返回字节。
- `Cxyz0`：通过同一 XYZ 路径发送一帧零 `Motion_Velocity`。
- `Cstop0`：发送官方的零速度停止帧。

每条紧凑响应以 `C:` 开头，报告 `txpin`、`rxpin`、`baud`、
发送/接收字节数、解析帧数、checksum-ok 计数、
checksum-bad 计数、最后的 `A/B/LEN` 以及 `err`。相同诊断也
可通过 JSON 调试协议使用：

```json
{"type":"chassis_diag","command":"repeat"}
```

支持的 JSON 命令值与紧凑后缀对应：`status`、`ready`、
`version`、`params`、`repeat`、`odom`、`xyz`、`xyzrx`、`xyz500`、`xyz500rx`、
`xyz0` 和 `stop0`。

## 循线传感器协议

循线传感器驱动使用 USART 数字模式：

```text
$0,0,1#
```

数字响应为：

```text
$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
```

当前硬件把黑线报告为 `0`、白色背景报告为 `1`，因此每个为零的通道值都成为一个有效循线位。若未来某个传感器模块报告相反的极性，通过无线调试 API 调整 `sensor.invert_bits`。

`GET /api/telemetry` 还为调试引导包含一个 `line_uart` 诊断对象：

- `ready`：UART 驱动是否已初始化。
- `requests`：已发送多少条数字模式请求命令。
- `frames`：已接收多少条以 `#` 结尾的完整帧。
- `digital_frames`：已接收帧中有多少条符合 `$D,x1:...#` 数字协议。
- `invalid_frames`：有多少条完整帧未通过数字解析器。
- `timeouts`：有多少次读取尝试在没有完整帧的情况下超时。
- `last_error`：最新的 UART 写/读/解析器错误码。
- `last_frame`：最新的原始帧，为诊断而做了截断。

如果 `requests` 增加但 `frames` 保持为 `0`，先检查传感器 TX/RX 方向、共地、供电和波特率。如果 `frames` 增加但 `digital_frames` 保持为 `0`，检查 `last_frame` 并更新解析器或请求命令以匹配模块的实际输出。

## 协议待办事项

- 确认循线传感器的波特率。
- 确认在物理模块上数字值 `1` 表示黑线还是白色背景。
- 物理测试后调整基础速度和转向增益。
