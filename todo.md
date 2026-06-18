# 轮腿机器人系统逐步调试指南 (System Debugging & Commissioning Checklist)

为了确保软硬件安全，避免因转向错误、大电压飞车、传感器异常导致机械结构损坏或驱动器烧毁，调试过程必须遵循**“先逻辑后动力、自底向上、软硬件解耦”**的原则。

---

## 阶段一：总线与通信调试 (Low-Level Communications Check)
*此阶段不加高压动力电（仅通逻辑电，如 5V/USB 供电），使机器人悬空离地。*

### [ ] Step 1.1: I2C 总线物理连接与电平诊断
- **调试对象**：ESP32-C3 I2C 物理引脚（`GPIO3=SCL`, `GPIO4=SDA`）与外部从设备之间的物理电气连接。
- **涉及代码/文件**：
  - [LegConfig.h](file:///home/waya/Projects/Embedded/ParallelLegRobot/include/LegConfig.h) (I2C 引脚定义 `I2C_SDA_PIN`, `I2C_SCL_PIN`)
  - [i2c_line_diagnostic.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/i2c_line_diagnostic.cpp)
- **排查要点**：
  - [ ] 用万用表量取 `SDA/SCL` 对地阻抗，确认没有发生物理短路。
  - [ ] 烧录 `i2c-line-diagnostic` 固件，开启串口监视器，验证空闲状态下 `SDA=1`, `SCL=1`。若为 `0` 说明总线被拉低或短路。
  - [ ] 确认总线上存在物理上拉电阻（4.7kΩ 或 10kΩ 连至 3.3V）。

### [ ] Step 1.2: I2C 从设备在线扫描 (I2C Scanner)
- **调试对象**：MPU6050 IMU, 双侧磁编码器 (AS5600), STM32F103 协处理器。
- **涉及代码/文件**：
  - [main.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/main.cpp) (函数 `scanI2cBus()`)
- **排查要点**：
  - [ ] 确保机器人整机上电（使外部传感器和 STM32F103 获得工作电压）。
  - [ ] 串口终端输入 `i2cscan` 指令。
  - [ ] 验证输出结果中正确检测到以下四个地址：
    - `0x12`：STM32F103 轮协处理器。
    - `0x36`：左轮磁编码器。
    - `0x38`：右轮磁编码器。
    - `0x68`：MPU6050 IMU 陀螺仪。
  - **安全提醒**：如果某个地址缺失，切勿继续后期的动力测试。

### [ ] Step 1.3: CAN 总线与关节驱动器通信
- **调试对象**：ESP32-C3 TWAI 控制器、TJA1050T 收发器、4 个腿关节电机驱动板（STM32G431）。
- **涉及代码/文件**：
  - [main.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/main.cpp) (函数 `setupCan()`, `printCanStatus()`)
- **排查要点**：
  - [ ] 检查 CANH/CANL 差分线上是否有 120Ω 终端电阻（总线两端各一个）。
  - [ ] 串口输入 `can` 指令，确认 CAN 控制器状态为 `RUNNING`。
  - [ ] 串口输入 `status` 指令，验证 4 个关节电机状态均为 `yes` (Online)：
    - `ID1` (左上), `ID2` (左下), `ID5` (右上), `ID6` (右下)。

---

## 阶段二：传感器与状态反馈校验 (Sensor & Feedback Verification)
*确保控制算法获取 of 输入信号是物理正确的。*

### [ ] Step 2.1: 机身 IMU 姿态解算校验
- **调试对象**：MPU6050 陀螺仪与加速度计。
- **涉及代码/文件**：
  - [ImuManager.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/ImuManager.cpp)
  - [main.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/main.cpp) (指令 `imu`, `imustream on/off`)
- **排查要点**：
  - [ ] 机器人保持水平静止，输入 `imu`，检查初始 Pitch 角是否接近 `0.0`。
  - [ ] 输入 `imustream on`。手动让机器人向前倾斜，观察输出 Pitch 角变化是否为正，向后倾斜时是否为负。
  - [ ] 观察在剧烈摇晃后重新静止，Pitch 角能否迅速收敛至正确数值，有无积分漂移。

### [ ] Step 2.2: 车轮磁编码器旋向与范围校验
- **调试对象**：左轮 (`0x36`) 与右轮 (`0x38`) 的 AS5600 磁编码器。
- **涉及代码/文件**：
  - [main.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/main.cpp) (函数 `readAS5600Encoder()`, 指令 `wheel`, `wheelstream on`)
- **排查要点**：
  - [ ] 输入 `wheelstream on`。
  - [ ] 手动旋转左轮和右轮整整一圈，检查 `raw_l` 和 `raw_r` 能否从 `0 ~ 4095` 满量程变化，且无断点跳变。
  - [ ] 验证旋向：
    - 车轮向前滚时，读数应呈递增变化；向后滚时呈递减变化。
    - 若数值方向相反，需在代码中取反，或调整物理极性。

---

## 阶段三：轮部 SimpleFOC 闭环对准与开环运行 (SimpleFOC Alignment & Drive)
*调校 STM32F103 的三相无刷驱动逻辑与 FOC 零点。车轮悬空！*

### [ ] Step 3.1: FOC 零电角度对齐 (FOC Motor Alignment)
- **调试对象**：STM32F103 协处理器、2 块 SimpleFOC 无刷驱动板、无刷轮电机。
- **涉及代码/文件**：
  - [f103_wheel_pwm.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/f103_wheel_pwm.cpp) (函数 `setup()`, `motor_left.linkSensor(&sensor_left)`, `motor_left.init()`, `motor_left.initFOC()`)
- **排查要点**：
  - [ ] 车轮必须完全悬空，无外力干扰。
  - [ ] 主控 ESP32-C3 运行并保持发送 I2C 角度数据（`wheel_coprocessor_online` 显示 `online`）。
  - [ ] 给 STM32F103 上电，观察电机是否轻轻扭动（正反转）。
  - [ ] 检查 F103 上的 `PC13` 状态指示灯：对相校准通过后，该 LED 将从闪烁/常暗变为**常亮/变暗**。
  - **安全提醒**：如果对相失败，电机会剧烈震动、发热且无法转动。必须检查 F103 中配置的 `MOTOR_POLE_PAIRS`（电机极对数，当前为 11）是否与物理电机一致。

### [ ] Step 3.2: 轮子开环电压指令测试 (Open-Loop Voltage Drive)
- **调试对象**：3-PWM 无刷驱动器输出、轮子转向。
- **涉及代码/文件**：
  - [main.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/main.cpp) (指令 `wheelarm`, `wheelpwm <left_pwm> <right_pwm>`, `wheeldisarm`)
- **排查要点**：
  - [ ] 输入 `wheelarm` 使能轮子控制器输出。
  - [ ] 输入微小电压：`wheelpwm 100 100`（占空比约 10%）。
  - [ ] 检查车轮是否平稳低速向前旋转。
  - [ ] 输入反向电压：`wheelpwm -100 -100`，确认车轮向后旋转。
  - [ ] 输入 `wheeldisarm` 确认车轮立刻停转且完全失去阻尼（变为自由状态）。

---

## 阶段四：腿部关节电机单腿闭环与高度保持 (Leg Zero Hold & Height Control)
*调校并联五杆腿部的运动学与零位保持。*

### [ ] Step 4.1: 腿电机单关节零点与极性测试
- **调试对象**：4 个 leg 关节电机。
- **涉及代码/文件**：
  - [main.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/main.cpp) (指令 `dirs`, `confirm_dirs`, `zero <id>`)
- **排查要点**：
  - [ ] 用手托住机械腿，输入 `dirs`，手动微动各关节，配合串口输出验证其极性符号是否物理正确。
  - [ ] 极性验证无误后，输入 `confirm_dirs`。
  - [ ] 输入 `arm`，然后输入 `zero 1`，锁死 1 号电机。用手转动 1 号关节主动臂，感受是否有极强的阻力（PD 刚度）。
  - [ ] 依次测试 `zero 2`, `zero 5`, `zero 6`。
  - **安全提醒**：一旦发现电机啸叫、剧烈震动或向单方向飞车，必须立刻输入 `stop` 或 `disarm` 切断输出！

### [ ] Step 4.2: 双腿协同高度保持 (Zero Hold / Height Control)
- **调试对象**：并联五杆腿部高度 PD 控制器、足端反力。
- **涉及代码/文件**：
  - [LegKinematics.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/LegKinematics.cpp)
  - [main.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/main.cpp) (指令 `zero`, `holdoff`)
- **排查要点**：
  - [ ] 机器人双腿着地，双手扶住机身以防倾倒，输入 `zero`。
  - [ ] 观察机器人是否能够自主站立（高度支撑在默认 100mm 附近）。
  - [ ] 手动轻轻向下按压机身，检查机械腿是否表现出类似于物理弹簧的弹性支撑和阻尼感。

---

## 阶段五：车体动静态自平衡联调 (Self-Balancing Coordination)
*将 Pitch 反馈引入车轮控制环路，实现自平衡站立。*

### [ ] Step 5.1: 直立环（角度 PD 控制）调试
- **调试对象**：自平衡直立环控制算法、双轮差速响应。
- **涉及代码/文件**：
  - [main.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/main.cpp) (直立环 PID 计算代码段)
- **排查要点**：
  - [ ] 先将速度环参数置零，仅保留较小的直立环参数（`Kp_pitch`, `Kd_pitch`）。
  - [ ] 双轮着地，人手扶住机身让其小幅度前后倾斜。
  - [ ] 校验响应：
    - 车身向前倾斜时，双轮必须立刻向前加速运动；车身后仰时，双轮必须立刻向后退。
    - 若轮子运动方向相反（导致加速摔倒），需要对直立环控制输出或轮子 PWM 符号进行取反。
  - [ ] 逐步加大 `Kp_pitch` 直至手松开后车身能产生短暂 of 直立恢复力，且不产生剧烈的高频震荡。

### [ ] Step 5.2: 速度环（位置/速度 PI 控制）融入
- **调试对象**：速度反馈低通滤波器、平衡收敛点。
- **涉及代码/文件**：
  - [main.cpp](file:///home/waya/Projects/Embedded/ParallelLegRobot/src/main.cpp) (速度环 PID 计算代码段)
- **排查要点**：
  - [ ] 逐步引入速度环 `Kp_speed` 和 `Ki_speed`，用以消除车体在一个方向上的持续漂移。
  - [ ] 测试用手轻推车身，车身倾斜避让后是否能够迅速回到原定站立点。
  - [ ] 观察在粗糙或地毯等不同地面上，机器人的自平衡静态稳定性。

---

## 阶段六：高级轮腿融合控制 (Advanced Leg-Wheel Integration)
*实现动态高度变化、差速转向与弹跳。*

### [ ] Step 6.1: 变高度站立与差速转向
- **调试对象**：腿部几何解算（IK/FK）动态更新，轮部差速偏置。
- **排查要点**：
  - [ ] 在自平衡站立状态下，发送高度修改指令（例如从 100mm 降到 80mm），观察双腿是否能平稳下蹲而不影响自平衡。
  - [ ] 差速发送转向指令，测试机器人能否进行原地自转（Spin）与前行弯道转向（Yaw 轴偏航控制）。

### [ ] Step 6.2: 动态推地与跳跃
- **调试对象**：关节电机前馈力矩（Feedforward Torque）与虚拟阻抗力矩控制。
- **排查要点**：
  - [ ] 调优高度控制器，输出瞬间向下的爆发力以离地。
  - [ ] 离地瞬时快速收回机械腿（改变工作高度目标值），避免触地硬撞。
  - [ ] 落地时，切换为大阻尼、小刚度的 PD 虚拟阻抗参数，完成软着陆吸能缓冲。
