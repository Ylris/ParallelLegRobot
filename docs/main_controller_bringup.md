# ESP32C3 主控安全联调流程

本文档对应默认固件 `src/main.cpp`，目标是在轮电机闭环尚未接入、四个腿电机架空时，先完成腿部 CAN 控制链路。

## 当前硬件连接

- 自制主控板 CAN+ 接 YYT 驱动 CANH。
- 自制主控板 CAN- 接 YYT 驱动 CANL。
- 自制主控板 ESP32C3 逻辑侧 CAN_RX 为 GPIO7，CAN_TX 为 GPIO6。
- YYT 电机 ID 按侧视图/图面坐标记录：
  - 图面左上腿关节：ID1
  - 图面左下腿关节：ID2
  - 图面右上腿关节：ID5
  - 图面右下腿关节：ID6
  - 左腿轮电机：ID3
  - 右腿轮电机：ID4

## 固件安全策略

- 当前构建上电 `disarmed`，默认命令为 0 mV，不会主动点动或自动保持姿态。
- 需要通过 `arm` 显式允许腿电机输出电压。
- `test` 只允许 ID1/2/5/6，持续时间限制为 20 到 500 ms。当前非零点动电压会限制到 `+/-6000..12000 mV`。
- `height` 高度保持必须先完成 ID1/2/5/6 四个关节的 `test` 点动，再输入 `confirm_dirs`。
- 如果某个关节方向反了，可以先用 `invert <id>` 临时翻转，或者用 `sign <id> <1|-1>` 指定方向；修改方向后需要重新点动确认。
- 任一腿电机离线、关节超过软限位、CAN 发送失败、CAN 总线错误、或者 `disarm/stop`，高度保持都会停止并清零输出。

## 编译和烧录

```sh
cd /Users/ylris/轮腿/ParallelLegRobot
pio run -e esp32-c3-devkitm-1
pio run -e esp32-c3-devkitm-1 -t upload --upload-port /dev/cu.usbmodem1301
```

如果端口不是 `/dev/cu.usbmodem1301`，先查：

```sh
ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null
```

## 串口查看

```sh
pio device monitor -p /dev/cu.usbmodem1301 -b 115200
```

正常启动后会周期性看到：

```text
online: ID1=yes ID2=yes ID5=yes ID6=yes armed=no hold=off
```

输入：

```text
can
status
```

`can` 正常应看到 `state=running`，`tx_fail=0`。`status` 正常时能看到四个电机的 `raw` 原始角度和 `q` 零点归一角度。零点姿态附近，`q` 应接近 0 rad。

输入：

```text
dirs
```

可以看到方向检查提示和四个关节当前 `q` 值。这个命令只读数，不会让电机动作。
其中 `tested=yes/no` 表示这个关节是否已经在当前上电周期执行过 `test`。只有四个关节都是 `tested=yes`，`confirm_dirs` 才会通过。

## 单关节点动

先确保：

- 四个腿电机架空。
- 手能马上关闭实验电源。
- 限流先保守。
- 串口状态四个 ID 都在线。

推荐先用脚本辅助，它会在每个电机点动前暂停，按回车才继续，并在最后汇总四个 `dq`：

```sh
cd /Users/ylris/轮腿/ParallelLegRobot
python3 tools/leg_test_sequence.py --port /dev/cu.usbmodem1301
```

脚本会自动把完整输出保存到 `logs/leg_test_时间.txt`。如果想指定日志文件：

```sh
python3 tools/leg_test_sequence.py --port /dev/cu.usbmodem1301 --log logs/my_leg_test.txt
```

`logs/*.txt` 默认被 `.gitignore` 忽略，作为本地现场证据保存。需要分析时，把日志路径或内容发出来即可。

如果要全自动不暂停，可以加 `--yes`，但首次不建议。

也可以手动在串口输入：

串口输入：

```text
arm
test 1 6000 100
stop
```

观察 ID1 是否轻微动一下，电流是否异常。`test` 结束后会打印 `q_before`、`q_after`、`dq`，把 `dq` 的正负记下来。然后依次测：

```text
test 2 6000 100
test 5 6000 100
test 6 6000 100
```

如果方向反了，先不要输入 `confirm_dirs`。例如 ID2 方向反了，输入：

```text
invert 2
test 2 6000 100
```

也可以直接指定方向：

```text
sign 2 -1
test 2 6000 100
```

注意：这个方向修改是运行时临时修改，重启后会恢复固件默认值。等四个方向都确认稳定后，再把最终 sign 固化进代码。
每次 `invert` 或 `sign` 后，该 ID 的 `tested` 会回到 `no`，必须重新 `test`。

## 指定高度保持

只有四个关节点动方向都确认后，输入：

```text
confirm_dirs
height 100
```

`height 100` 表示以零点姿态附近的 100 mm 腿高为目标，程序会慢慢斜坡靠近，输出限制为很小的电压。安全范围暂时限制为 80 到 120 mm。

推荐使用监控脚本，它会先检查四个关节都是 `tested=yes`，再提示你确认方向，然后执行 `confirm_dirs` 和 `height 100`，并周期打印 CAN 和关节状态。结束或 Ctrl-C 后会自动执行 `holdoff`、`stop`、`disarm`：

```sh
cd /Users/ylris/轮腿/ParallelLegRobot
python3 tools/height_hold_monitor.py --port /dev/cu.usbmodem1301 --height 100 --duration 8
```

脚本会自动把完整输出保存到 `logs/height_hold_时间.txt`。这份日志就是判断“没有 CAN 掉线、电流异常、撞限位或明显抖动”的主要软件证据。

如果脚本提示不是四个 `tested=yes`，先回到单关节点动步骤。

两份日志都有后，可以用分析脚本做软件证据检查：

```sh
python3 tools/analyze_bringup_logs.py \
  --leg-log logs/leg_test_时间.txt \
  --height-log logs/height_hold_时间.txt
```

脚本通过只说明软件日志里没有看到 CAN 掉线、发送失败、电机离线等问题；电流是否异常、是否撞限位、是否明显抖动，仍然以现场观察为准。

停止：

```text
holdoff
stop
disarm
```

## 当前阶段验收标准

- 上电能稳定读到 ID1/2/5/6。
- 默认命令为 0 mV，不主动运动。
- `arm` 后可以安全点动单个关节。
- 确认方向后，两条腿能在架空状态下缓慢移动到指定高度并保持。
- 不出现 CAN 掉线、电流异常、撞限位或明显抖动。

完成以上内容，说明腿部控制链路已经基本打通；真正站立和平衡需要等轮电机、IMU、轮速和整车姿态闭环加入后再做。
