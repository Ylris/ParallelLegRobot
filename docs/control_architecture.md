# 主控控制结构说明

本文档说明当前 `ParallelLegRobot` 主控程序如何参考本地 `foc-wheel-legged-robot`，以及哪些能力已经实现、哪些能力要等轮电机和整车条件满足后再做。

## 参考仓库的可用思路

参考文件：

- `foc-wheel-legged-robot/esp32-controller/software/src/main.cpp`
- `foc-wheel-legged-robot/esp32-controller/software/src/leg_pos.c`
- `foc-wheel-legged-robot/esp32-controller/software/src/leg_spd.c`
- `foc-wheel-legged-robot/esp32-controller/software/src/leg_conv.c`
- `foc-wheel-legged-robot/esp32-controller/software/src/lqr_k.c`

参考仓库的主程序是完整轮腿平衡车结构，大致分为：

- 电机层：保存角度、速度、零点、方向、目标电压。
- CAN 层：周期发送 0x100/0x200 电压命令，接收 0x101/0x102/... 反馈。
- 腿部状态层：把关节角度换算为腿长、腿角、腿速。
- 控制层：根据 IMU、轮速、腿长等状态计算关节和轮电机输出。
- 安全层：`robotArmed` 默认关闭，上电不允许直接出力。

当前项目没有轮电机，四个腿电机架空，所以只采用“电机层 + CAN 层 + 腿部几何 + 安全输出”的部分。不移植完整 LQR 平衡控制，因为没有轮电机时无法验证站立和平衡闭环。

## 当前默认主程序

默认构建目标是 `platformio.ini` 里的 `[env:esp32-c3-devkitm-1]`。

主程序文件：

- `src/main.cpp`

编译时关键参数：

- `YYT_CAN_RX_PIN=7`
- `YYT_CAN_TX_PIN=6`
- `YYT_SAFE_MV_LIMIT=250`
- `YYT_HOLD_MV_LIMIT=180`
- `ARDUINO_USB_CDC_ON_BOOT=1`

CAN 线序：

- 自制主控板 CAN+ 接 YYT CANH。
- 自制主控板 CAN- 接 YYT CANL。
- ESP32C3 逻辑侧 CAN_RX 是 GPIO7，CAN_TX 是 GPIO6。

## 电机 ID 和零点

当前四个腿电机：

- ID1：`left_front_upper`
- ID2：`left_rear_lower`
- ID5：`right_front_upper`
- ID6：`right_rear_lower`

零点来自：

- `config/leg_calibration.json`

固件中对应常量在 `src/main.cpp` 的 `kMotors`：

- ID1 zero = `4.860`
- ID2 zero = `5.553`
- ID5 zero = `0.161`
- ID6 zero = `1.991`

反馈角度处理逻辑：

```text
raw angle mrad -> angle_rad -> normalize(angle_rad - zero) * sign -> q
```

其中 `q` 是主控使用的归一化关节角。运行时可以用 `sign <id> <1|-1>` 或 `invert <id>` 临时调整方向；重启后恢复固件默认方向。

## CAN 协议

YYT 驱动板反馈：

- `0x101` -> ID1
- `0x102` -> ID2
- `0x105` -> ID5
- `0x106` -> ID6

反馈数据：

- bytes 0..3：`int32 angle_mrad`
- bytes 4..5：`int16 speed_rpm_x10`

主控发送：

- `0x100` 控制 ID1..ID4
- `0x200` 控制 ID5..ID8

每个电机一个 `int16 mV` 槽位，小端序。

## 安全状态机

上电默认：

- `armed = false`
- `height_hold_enabled = false`
- 所有命令为 `0 mV`

必须输入：

```text
arm
```

才允许输出非零电压。

单关节点动：

```text
test <id> <mv> <ms>
```

限制：

- 只允许 ID1/2/5/6。
- 持续时间限制为 20 到 300 ms。
- 电压受 `YYT_SAFE_MV_LIMIT` 限制。
- 点动结束自动清零输出。
- 点动后打印 `q_before`、`q_after`、`dq`。

方向确认：

- 每个 ID 必须至少成功执行过一次 `test`。
- `confirm_dirs` 只有在 ID1/2/5/6 都是 `tested=yes` 后才会通过。
- 使用 `sign` 或 `invert` 后，该 ID 的 `tested` 会回到 `no`。

高度保持：

```text
height 100
```

限制：

- 必须先 `arm`。
- 必须先 `confirm_dirs`。
- 四个腿电机必须在线。
- 高度范围暂时限制为 80 到 120 mm。
- 输出受 `YYT_HOLD_MV_LIMIT` 限制。
- 目标角度按斜坡慢慢接近，避免突然动作。

自动停止条件：

- `disarm` 或 `stop`
- 任一腿电机离线
- 关节超过软限位
- CAN 发送失败
- CAN 总线错误
- TWAI 状态不是 `running`

## 当前已经实现的目标

当前软件已经实现：

- 四个腿电机 CAN 反馈读取。
- 零点归一和方向修正。
- 默认上电不动作。
- 串口 `arm/disarm/stop`。
- 单关节点动和 `dq` 输出。
- 四关节 tested 门槛。
- 指定高度保持框架。
- CAN 状态查询。
- 点动日志、高度保持日志、日志分析脚本。

当前已经通过只读串口验证：

- ID1/2/5/6 能在线。
- 默认 `armed=no`。
- 默认 `hold=off`。
- 默认命令 `0 mV`。

## 当前还必须现场验证的目标

必须由现场测试证明：

- 四个关节点动时电流不异常。
- 四个关节 ID 和机械方向正确。
- `height 100` 时两条腿缓慢移动到目标高度。
- 高度保持期间不撞限位、不明显抖动。
- 高度保持日志中没有 CAN 掉线、发送失败、电机离线。

推荐现场顺序：

```sh
python3 tools/leg_test_sequence.py --port /dev/cu.usbmodem1301
python3 tools/height_hold_monitor.py --port /dev/cu.usbmodem1301 --height 100 --duration 8
python3 tools/analyze_bringup_logs.py --leg-log logs/leg_test_时间.txt --height-log logs/height_hold_时间.txt
```

只有日志分析通过，并且现场观察确认电流、限位、抖动都正常，当前 goal 才能算完成。

## 后续扩展路径

没有轮电机时，不做真正站立平衡。

后续加回轮电机后，再按这个顺序扩展：

- 加轮电机 CAN ID、零点和方向。
- 接入 MPU6050 姿态。
- 引入腿长、腿角、腿速状态。
- 引入轮速和车体速度估计。
- 再参考 `lqr_k.c` 和原主控结构做 LQR 平衡。
