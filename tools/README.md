# 工具

## ESP32 状态机测试器

`state_machine_tester.py` 是电脑端、离线可用的 ESP32 循线车调试 API 测试运行器。它只依赖 Python 标准库，因此电脑断网并接入 ESP32 SoftAP 后依然可用。

固件收到 BLE 紧凑命令 `W1` 后的默认 Wi-Fi 后备目标：

- Wi-Fi SSID：`DCar-Liner-F47785`，或固件当前显示的 `DCar-Liner-XXXXXX`。
- Wi-Fi 密码：`DCar-Liner123`。
- ESP32 地址：`http://192.168.4.1`。
- WebSocket：`ws://192.168.4.1/ws`。

运行安全默认测试：

```sh
python3 tools/state_machine_tester.py
```

安全默认测试会先发送一次预检 `stop`，然后验证 HTTP 可达性、WebSocket 命令、调参锁、手动测试超时、`AUTO_ARMED` 和 `stop`。它不会发送 `start_auto`。

仅在小车被架起、固定或确保安全时，才运行可选的自动运行测试：

```sh
python3 tools/state_machine_tester.py --include-auto
```

每次运行都会写入：

- `logs/state-machine-tests/<timestamp>/events.jsonl`
- `logs/state-machine-tests/<timestamp>/summary.md`

恢复正常网络后，查看打印出的日志目录，或把目录路径分享出来以供分析。

## BLE 调试测试器

`ble_debug_tester.swift` 是 Mac 端基于 CoreBluetooth 的 BLE 调试传输测试运行器。无需手机或网络连接。

默认目标：

- BLE 设备名前缀：`DCtrl`。
- 运行时 BLE 名称：`DCtrl`。
- 服务 UUID：`7b3a0001-8d4d-4b9a-b5c7-0f7c4c415243`。
- RX 特征：`7b3a0002-8d4d-4b9a-b5c7-0f7c4c415243`。
- TX 特征：`7b3a0003-8d4d-4b9a-b5c7-0f7c4c415243`。

运行：

```sh
swift tools/ble_debug_tester.swift --timeout 25
```

测试器会扫描、连接、写入 `{"type":"get_telemetry"}`，读取直接或分包返回的 TX 响应，校验字节数和 JSON 合法性，然后写入：

- `logs/ble-debug-tests/<timestamp>/events.jsonl`
- `logs/ble-debug-tests/<timestamp>/summary.md`

微信路径使用的紧凑 PID/档位帧：

```sh
swift tools/ble_debug_tester.swift --timeout 25 --command G --append-newline --expect-prefix P
swift tools/ble_debug_tester.swift --timeout 25 --command S9000,0,0,2 --append-newline --expect-prefix OK
```

紧凑命令 `G` 的响应为 `P<kp>,<ki>,<kd>,<gear>`。紧凑命令 `S` 会写入并保存这四个值；超出范围返回 `E:RANGE`。

## BLE 遥控测试器

`ble_remote_tester.swift` 针对独立的 DCtrl 遥控服务，通过透传桥发送原始 21 字节 DFLink `Motion_Velocity` 帧。它是 Mac 端烟雾测试器，验证与微信摇杆相同的线缆约定：`Vx/Vy` 编码为 m/s，`Vz` 在业务层 deg/cmd 转换后编码为 rad/cmd。

```sh
swift tools/ble_remote_tester.swift --timeout 25 --motion zero
swift tools/ble_remote_tester.swift --timeout 25 --motion forward
swift tools/ble_remote_tester.swift --timeout 25 --motion strafe
swift tools/ble_remote_tester.swift --timeout 25 --motion yaw
```

对于非零运动，测试器每 `50 ms` 重发一次当前帧，持续 `500 ms`，然后发送三帧零速。遥控桥的成功路径刻意保持静默；当所有 CoreBluetooth 写入完成且没有 `E:*` 遥控错误时测试通过。出现 `E:BUSY`、`E:UART`、`E:LEN`、`E:COPY`、断连或超时则判为失败。
