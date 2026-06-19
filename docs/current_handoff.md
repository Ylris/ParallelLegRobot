# Current Handoff

Last updated: 2026-06-19 08:21 CST (Asia/Shanghai)

> [!IMPORTANT]
> 读这个文件先，然后读 `docs/yyt_motor_workflow.md` 和 `docs/yyt_firmware_backups.md`。

## 当前状态：需要重启 Mac 恢复 USB

用户正在重启 Mac 来解决 USB 串口识别问题。ESP32 和 USB 线硬件正常（别的电脑能识别到），是 macOS USB 子系统卡住了。

### 重启后操作步骤
1. 插上 ESP32 USB 线
2. 确认 `ls /dev/cu.usbmodem*` 能看到设备
3. 给电机供电（12V）
4. 等待 ~3 秒让电机完成 sweep-align 对齐
5. 运行高度闭环测试：
```
arm
confirm_dirs
height 83
status
# 观察几秒
holdoff
disarm
```

### 关键参数
- **USB 串口**: `/dev/cu.usbmodem11301`，115200，DTR=True，RTS=False
- **目标高度**: **83mm**（用户确认的站立高度，不要用 100mm）
- **ST-Link**: 物理连接在 **ID2** 板子上

## Drive 固件状态

四个电机全部刷了 **sweep-align 12V** 固件（最新版）：

| Drive | DRIVE_ID | 对齐方式 | 对齐电压 | 固件版本 |
|-------|----------|---------|---------|---------|
| ID1 | 1 | sweep-align | 12V | 最新 |
| ID2 | 2 | sweep-align | 12V | 最新 |
| ID5 | 5 | sweep-align | 12V | 最新 |
| ID6 | 6 | sweep-align | 12V | 最新 |

### Drive 固件编译命令
```sh
make -C DriveFirmware clean && make -C DriveFirmware \
  DRIVE_ID=<N> MOTOR_ALIGN_ONLY=1 MOTOR_POLE_PAIRS=14.0f \
  MOTOR_ALIGN_VOLTAGE=12.0f FOC_MODULATION_LIMIT=1.0f \
  CAN_MAX_COMMAND_VOLTAGE=12.0f DRIVE_AUTO_ZERO_HOLD=0 \
  YYT_DISABLE_OUTPUT=0 all
```

### Sweep-align 原理
在 `DriveFirmware/FOC/Motor.c` 中 `MOTOR_ALIGN_ONLY` 模式：
- 12V 电压缓慢扫过一个电周期（25.7° 机械角，200步×3ms）
- 正向扫一圈再反向扫回来
- 最后在目标角度保持 500ms，读编码器计算电零点
- 比简单施加固定电压更可靠，但仍有 ~30% 概率对齐失败（dq≈0）
- 对齐失败的解决方法：断电重新上电

## ESP32 主控固件状态
- PlatformIO 项目，环境 `esp32-c3-devkitm-1`
- 上传命令: `pio run -e esp32-c3-devkitm-1 -t upload`
- 固件功能正常，只改了 `src/main.cpp` 中的：
  - PD 增益: Kp=3000, Kd=150
  - drive_sign 映射
  - IK 高度→关节角度计算
  - 斜坡率: 0.08 rad/s

## 高度闭环测试结果（成功的那次）

在全部重新上电后，四电机全部对齐成功时的数据：

### 方向脉冲测试（全部对称）
| Drive | +6000 mV → dq | -6000 mV → dq |
|-------|--------------|---------------|
| ID1 | +0.547 | -0.556 |
| ID2 | -0.540 | +0.547 |
| ID5 | -0.626 | +0.619 |
| ID6 | -0.567 | +0.573 |

### 83mm 高度闭环（稳定运行 13 秒）
| 电机 | q (rad) | target (rad) | cmd (mV) |
|------|---------|-------------|----------|
| ID1 | -0.962 | -0.881 | -243 |
| ID2 | +1.454 | +1.111 | +1024 |
| ID5 | +1.320 | +1.185 | +387 |
| ID6 | +0.691 | +0.616 | -221 |

## drive_sign 和方向
- 用户要求: **ID1/5 向前，ID2/6 向后**
- 当前 sign: ID1=+1, ID2=-1, ID5=-1, ID6=+1（在 main.cpp 中）
- 方向尚未完全验证（用户还没确认物理方向是否正确）

## 已知问题

### 1. 对齐不稳定（~70% 成功率）
上电后 sweep-align 有时标定到 d 轴（dq≈0），需断电重试。
未来改进：对齐后自动检测，如果 dq≈0 则偏移 PI/2 重试。

### 2. ID1 CAN 间歇掉线
移动 ST-Link 线缆时 ID1 可能掉线。全部重新上电后通常恢复。

### 3. macOS USB 识别问题
本次会话后期 macOS 无法识别 ESP32 USB（别的电脑可以），
需要重启 Mac 解决。

## 修改过的文件
- `src/main.cpp` — PD 增益、drive_sign、IK 修复
- `DriveFirmware/FOC/Motor.c` — MOTOR_ALIGN_ONLY sweep 模式
- `DriveFirmware/Makefile` — MOTOR_ALIGN_ONLY 编译变量
- `docs/current_handoff.md` — 本文件

## 下一步计划
1. 重启 Mac，恢复 USB 连接
2. 重新上电，跑 `height 83` 闭环测试
3. 让用户确认物理方向（ID1/5 是否向前）
4. 如果方向不对，调整 drive_sign
5. 改进 align 可靠性（加自检+自动重试）
6. 长时间稳定性测试
