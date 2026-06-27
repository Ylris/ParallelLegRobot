# Current Handoff

Last updated: 2026-06-28 CST (Asia/Shanghai)

> [!IMPORTANT]
> 当前阶段先把 YYT 腿电机调成可靠执行器。先不要调轮电机、新 IMU、平衡、站立高度闭环。

## 当前目标

把 ID1/ID2/ID5/ID6 四块 YYT MiniOdrive 都统一成：

- 上电做 sweep-align 电角度对齐
- 通过 CAN 稳定 online 并回传角度
- 收到 ESP32 的 mV 电压命令后平滑出力
- 不使用 YYT 本地位置保持作为通用默认行为
- 默认 0 mV，不主动动作；ESP32 输入 `arm` 后才允许点动

ESP32 仍然是上位机，负责之后的腿高、姿态、整车控制；YYT 先只做底层 FOC/电压执行器。

## 当前现场状态

- ST-Link 当前按用户描述接在 ID2。
- 已重新编译并烧录 ID2 的 sweep-align CAN 电压执行器固件。
- 烧录命令已执行成功，OpenOCD 输出 `Verified OK`。
- 随后固件增加了 sweep-align 后的 32 次圆周平均采样，用于降低电角度零点读数抖动；此平均采样版已编译通过，但尚未烧录到板子。
- ESP32 主控程序已编译通过。
- macOS 当前没有识别到 ESP32 USB 串口；扫描只看到 `/dev/cu.Bluetooth-Incoming-Port` 和 `/dev/cu.debug-console`。
- 因为没有 ESP32 串口，暂时还不能发 `status`、`arm`、`test` 来验证 CAN 和 ID2 正负点动。
- 最新一次 OpenOCD 探测失败为 `Error: open failed`，随后 USB 设备树也没有看到 `STM32 STLink`，需要重新插好 ST-Link USB 后再烧录。

## 立即下一步

1. 插好 ESP32 主控板 Type-C USB 线。
2. 让 macOS 出现类似 `/dev/cu.usbmodem...` 或 `/dev/cu.usbserial...` 的端口。
3. 给 YYT 板上 12V 动力电源，限流保守，电机保持自由转动或架空。
4. 等待 3 秒左右，让 ID2 完成 sweep-align。
5. 查询状态：

```sh
cd /Users/ylris/轮腿/ParallelLegRobot
python3 -B tools/read_status_safe.py --command status
python3 -B tools/read_status_safe.py --command can
```

6. 如果 ID2 online，再做正负点动：

```sh
python3 -B tools/test_pulse_safe.py --id 2 --mv 6000 --ms 100 --no-reset
python3 -B tools/test_pulse_safe.py --id 2 --mv -6000 --ms 100 --no-reset
```

正常现象：

- `status` 里 ID2 是 `online`
- `can` 里状态是 `running`
- 正负点动都有角度变化，`dq` 一正一负，幅度接近
- 电机只轻微转动，不尖叫、不抖、不乱转
- `tx_fail` 不持续增加

异常处理：

- 没有 USB 串口：换 Type-C 线/方向/接口，必要时重启 Mac。
- ID2 offline：检查 CANH/CANL/GND、YYT 主电源、ID2 固件是否烧在正确板子上。
- 点动 `dq≈0`：断电重新上电，让 sweep-align 重跑，再测一次。
- 抖动、尖叫、过流、乱转：立刻断动力电源，保留串口输出再分析。

## Drive 固件目标

推荐使用 `DriveFirmware` 的新目标：

```sh
cd /Users/ylris/轮腿/ParallelLegRobot
make -C DriveFirmware DRIVE_ID=2 sweep-align-id
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "adapter speed 100; init; reset halt; program DriveFirmware/build/turing_CBU6.bin 0x08000000 verify reset; shutdown"
```

同一流程按物理 ST-Link 连接依次替换 `DRIVE_ID=1/2/5/6`。不要在 ST-Link 还接着 ID2 时烧 ID1/5/6。

当前 sweep-align 默认参数：

- `MOTOR_ALIGN_ONLY=1`
- `MOTOR_POLE_PAIRS=14.0f`
- `MOTOR_ALIGN_VOLTAGE=12.0f`
- `MOTOR_ALIGN_AVG_SAMPLES=32`
- `MOTOR_ALIGN_AVG_DELAY_MS=2U`
- `FOC_MODULATION_LIMIT=1.0f`
- `CAN_MAX_COMMAND_VOLTAGE=12.0f`
- `DRIVE_AUTO_ZERO_HOLD=0`
- `YYT_DISABLE_OUTPUT=0`

## ESP32 主控状态

- 默认环境：`esp32-c3-devkitm-1`
- 当前编译通过。
- 已改为上电 `disarmed`。
- 已关闭上电自动 fixed-pose hold。
- 已关闭 `YYT_ALLOW_UNTESTED_DIRS` 绕过，必须点动 ID1/2/5/6 后才允许 `confirm_dirs`。
- 串口命令以 `status`、`can`、`dirs`、`arm`、`test`、`stop`、`disarm` 为主。
- 当前阶段不要使用 `height`、`stand`、`balance on`。

## 四块腿电机通过标准

- ID1/ID2/ID5/ID6 都能稳定 online。
- 每个 ID 的 `test <id> 6000 100` 和 `test <id> -6000 100` 都能产生可预测方向。
- 点动后角度反馈变化合理，正负 `dq` 幅度接近。
- 无明显抖动、尖叫、过流、乱跳。
- CAN 不掉线，`tx_fail` 不持续增加。

四块都通过以后，再装回机械腿，重新做机械零点、方向确认、腿高保持。
