# ESP32-C3 轮腿 CAN 主控 + IMU 姿态反馈

这是一个基于 ESP32-C3 与 PlatformIO 搭建的 **轮腿机器人主控板** 项目。主控通过 CAN 总线连接 YYT MiniOdrive 电机驱动器，并接入 MPU6050 IMU，用于轮腿/并联腿机构的运动控制与姿态反馈。

并联腿（如经典的五杆闭链机构，Five-bar linkage）由于其电机（舵机）都可以安装在靠近身体的根部，具有极低的腿部惯量，非常适合制作高性能的双足或四足跑跳机器人。

此外，本项目集成了 **MPU6050 IMU** 传感器，通过互补滤波算法计算出机身姿态（Pitch），实现了闭环姿态反馈调节（自平衡模式）。

---

## 1. 机构几何模型与原理

该并联腿使用五杆闭链机构，其坐标系与结构定义如下：

```
       [ 机器人身体 (y = 0) ]
     A1 (-d, 0)      A2 (d, 0)   <-- 左右两个主动舵机轴心
       \            /
   L1   \          /   L1        <-- 主动臂 (Link 1)
         \        /
         B1      B2              <-- 肘部转轴 (无源关节)
         /        \
   L2   /          \   L2        <-- 从动臂 (Link 2)
       /            \
       \            /
        \          /
         \        /
          \      /
           \    /
            \  /
             \/
             P (x, y)            <-- 足端 (Foot End-effector)
```

- **原点 (0, 0)**: 左右舵机旋转轴心连线的中点。
- **$A_1, A_2$**: 左右舵机轴心位置，坐标分别为 $(-d, 0)$ 和 $(d, 0)$。
- **$L_1$**: 主动臂长度 (从舵机臂到肘部)。
- **$L_2$**: 从动臂长度 (从肘部到足端)。
- **$B_1, B_2$**: 左右肘部关节坐标。
- **$P(x, y)$**: 足端坐标。

### 逆运动学 (Inverse Kinematics, IK)
通过给定的足端目标位置 $P(x, y)$，反算出左右舵机的目标控制角度 $\theta_1$ 和 $\theta_2$。
- 利用余弦定理和极坐标变换求解三角形 $A_1B_1P$ 和 $A_2B_2P$。
- 设定肘部向外弯曲（肘朝外，Outer-bending），以获得最大的工作空间并避免杆件相撞。

### 顺运动学 (Forward Kinematics, FK)
通过给定的左右舵机角度 $\theta_1$ 和 $\theta_2$，计算出当前足端所处的坐标 $P(x, y)$。
- 求解两圆相交问题：圆心分别为 $B_1$ 和 $B_2$，半径均为 $L_2$ 的两个圆的交点中，取 $Y$ 值较小的交点（向下延伸的点）。

---

## 2. 硬件方案与引脚分配 (ESP32-C3 + CAN)

当前原理图使用 **ESP32-C3** 作为主控，通过片内 **TWAI/CAN 控制器** 输出 `CAN_TX/CAN_RX`，再由外置 **TJA1050T CAN 收发器** 转换为 CANH/CANL 差分总线。MPU6050 通过 I2C 接入，USB Type-C 用于 5V 供电、烧录和串口调试。

### 2.1 主控引脚分配

| 功能 | ESP32-C3 引脚 | 原理图网络名 | 逻辑电平 | 连接对象 | 备注 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| I2C SCL | `GPIO4` | `I2C_SCL` | 3.3V | MPU6050 `SCL` | 原理图中通过 `R3=1k` 上拉到 3V3 |
| I2C SDA | `GPIO5` | `I2C_SDA` | 3.3V | MPU6050 `SDA` | 原理图中通过 `R4=1k` 上拉到 3V3 |
| CAN TX | `GPIO6` | `CAN_TX` | 3.3V | TJA1050T `TXD` | ESP32-C3 TWAI 发送脚 |
| CAN RX | `GPIO7` | `CAN_RX` | 3.3V 输入 | TJA1050T `RXD` | 需要注意 TJA1050T 的 5V 输出电平 |
| BOOT | `GPIO9` | `BOOT` | 3.3V | 下载按键到 GND | 进入下载模式使用 |
| LED | `GPIO10` | `LED` | 3.3V | `R5 + LED1` | 状态指示灯 |
| ADC | `GPIO0` | `ADC` | 3.3V | 电压采样分压 | `R7/R8` 分压后进入 ADC |
| USB D+ / D- | `GPIO19/GPIO18` | `USB+` / `USB-` | USB | Type-C 座 | 烧录、调试串口 |
| EN | `EN` | `EN` | 3.3V | 复位按键 | `R1=10k` 上拉到 3V3 |

> **[!WARNING]**
> * ESP32-C3 的 GPIO 是 3.3V 逻辑，不建议直接承受 5V 输入。TJA1050T 使用 5V 供电，`RXD` 高电平可能接近 5V，因此 `TJA1050T RXD -> ESP32-C3 CAN_RX(GPIO7)` 之间建议加电平转换，或改用带 3.3V 逻辑接口的 CAN 收发器。
> * 更推荐的 3.3V 方案是 `SN65HVD230/VP230`、`TCAN332/TCAN337`，或带 `VIO=3.3V` 的 `MCP2562`。如果继续使用 TJA1050T，至少要确认 CAN_RX 输入不会超过 ESP32-C3 允许范围。
> * ESP32-C3 原生 USB 通常使用 `GPIO19=USB_D+`、`GPIO18=USB_D-`。画板前请再核对 `USB+`、`USB-` 是否分别接到正确引脚。


---

## 2.2 CAN 收发器与总线接线

TJA1050T 在本方案中只负责物理层收发，CAN 协议解析由 ESP32-C3 的 TWAI 控制器和 YYT MiniOdrive 上的 STM32 FDCAN 外设完成，不需要 MCP2515。

| TJA1050T 引脚 | 原理图连接 | 说明 |
| :--- | :--- | :--- |
| `TXD` | `CAN_TX` / ESP32-C3 `GPIO6` | 主控发送到收发器 |
| `RXD` | `CAN_RX` / ESP32-C3 `GPIO7` | 收发器输出到主控，建议做 5V 到 3.3V 电平转换 |
| `VCC` | `5V0` | TJA1050T 供电为 5V |
| `GND` | `GND` | 与 ESP32-C3、电机驱动器共地 |
| `S` | `GND` | 高速正常模式 |
| `CANH` | CAN 连接器 `CANH` | CAN 总线高线 |
| `CANL` | CAN 连接器 `CANL` | CAN 总线低线 |
| `VREF` | 悬空 | 当前方案未使用 |

总线两端各放一个 `120R` 终端电阻即可。原理图中的 `R6=120R` 已经跨接在 CANH/CANL 上，如果这块板位于总线中间，应取消或改成可选跳帽；如果这块板位于总线端点，可以保留。

### 2.3 电源与外设

* **Type-C 供电**：USB Type-C 的 `VBUS` 接入 `5V0`，用于给 5V 轨和后级 3.3V LDO 供电。
* **3.3V 供电**：`SPX3819-3.3` 从 `5V0` 生成 `3V3`，供 ESP32-C3 和 MPU6050 使用。
* **5V 供电**：TJA1050T 接 `5V0`。电机动力电源不要从 ESP32-C3 板上取，应由独立电池或电源给 YYT 驱动器供电。
* **MPU6050**：`VDD` 接 `3V3`，`GND/EP` 接地，I2C 上拉到 3V3。
* **复位与下载**：`EN` 通过 10k 上拉到 3V3，复位按键拉低；`BOOT(GPIO9)` 下载按键拉低。

### 2.4 下位机配置

YYT MiniOdrive 使用 STM32G431，工程目录 `DriveFirmware/` 中已经包含 FDCAN 相关代码：

* `DriveFirmware/Core/Src/fdcan.c`：FDCAN1 初始化，Classic CAN，当前位时序约 1 Mbps。
* `DriveFirmware/Core/Src/can_bridge.c`：CAN 命令解析、电机电压控制和状态回传。

每块电机驱动板需要设置唯一的 `DRIVE_ID`。当前协议按 ID 分组：

| 驱动器 ID | 所属分组 | 接收命令 ID |
| :--- | :--- | :--- |
| `1` - `4` | 左侧腿组 | `0x100` |
| `5` - `8` | 右侧腿组 | `0x200` |

电机组装好后，需要根据 `闭环参数调试.docx` 校准 AS5600 或 MT6816 磁编码器，并确认零点参数已经保存到驱动板。

### 2.5 当前软件适配状态

`DriveFirmware/` 下位机固件已经是 CAN 接收方案；根目录 `src/main.cpp` 当前是现场 bring-up 用的安全 CAN 主控，默认使用 `CAN_TX=GPIO6`、`CAN_RX=GPIO7`，启动后默认 `disarmed` 且输出 0 mV。

截至 2026-06-18 的现场状态：

* 四个腿电机 ID 为 `1/2/5/6`，均能通过 CAN 在线反馈。
* 机械零点已写入 `config/leg_calibration.json`，主控中也固化了对应零点。
* YYT 预编译 6V 固件在 `DriveFirmware/firmware_ids/`。
* 主控支持串口命令 `status`、`can`、`imu`、`imustream on/off`、`arm`、`test`、`confirm_dirs`、`stand`、`height`、`holdoff`、`stop`、`disarm`。
* 最后一次测试中 CAN 链路稳定，主控能发出 `+/-6000 mV`，但架空腿没有移动到目标；下一步优先排查电池电量、主电源限流/压降或机构卡滞。

---


## 3. 项目结构说明

本项目的目录结构符合标准的 PlatformIO 规范：

```
WheelLegCANController/
├── include/
│   ├── LegConfig.h         # 物理参数、I2C引脚、零偏校准及工作空间限幅
│   ├── LegKinematics.h     # 顺/逆运动学求解器类定义
│   └── ImuManager.h        # IMU读取和互补滤波类定义
├── src/
│   ├── LegKinematics.cpp   # 运动学数学计算实现
│   ├── ImuManager.cpp      # I2C总线扫描、MPU6050初始化及姿态滤波算法
│   └── main.cpp            # ESP32-C3 初始化、指令解析与轨迹控制主程序
├── platformio.ini          # PlatformIO 项目编译配置文件
└── README.md               # 本说明文件
```

---

## 4. IMU 姿态解算与自平衡控制

1. **I2C 设备扫描**: 
   启动时，主机会自动扫描 I2C 总线并打印所有在线设备。这方便了用户排查接线故障，确保 IMU（如 MPU6050 默认地址 `0x68`）正确挂载。
2. **互补滤波姿态估计 (Complementary Filter)**:
   - 融合陀螺仪的积分角度与加速度计得到的重力夹角。
   - 表达式：$Angle = 0.96 \times (Angle + GyroRate \times dt) + 0.04 \times AccelAngle$。
   - 具有极高的稳定性和响应速度，有效滤除高频电机振动干扰。
3. **姿态平衡环控制 (MODE_BALANCE)**:
   - 闭环控制算法：机身向前倾斜（$Pitch > 0$）时，控制脚掌向前偏移以维持中心平衡；
   - 控制率：$X_{\text{compensate}} = Pitch \times K_p$ ($K_p = -1.2 \text{mm/deg}$)。
   - 可在开发板倾斜时实时驱动舵机运动以使足端朝反方向平移支撑。

---

## 5. 串口指令控制与交互

烧录程序后，通过串口助手（如 PlatformIO Serial Monitor，波特率 `115200`）连接机器人，可发送以下指令进行交互：

- `standby`: 机器人腿部将收回并静态悬停在中位 `(0, -100)` mm 处（开机默认模式）。
- `balance`: 开启 IMU 自平衡反馈（当倾斜机身时，机器人会根据 MPU6050 采集到的 Pitch 角度反向移动以维持平衡）。
- `X Y`: 手动控制足端移动到指定坐标（例如输入 `10 -90`，回车后足端会平滑移动到该位置。若输入的值超出了可达工作空间，终端会输出警告且保持安全限幅）。

---

## 6. 编译与烧录

使用 PlatformIO 命令行在项目根目录下进行编译与烧录：

```bash
# 1. 编译项目
pio run

# 2. 烧录到 ESP32-C3 核心板
pio run --target upload
```

---

## 7. 系统通信架构与协议规范

系统主要由三部分组成：**ESP32-C3（主控）**、**YYT MiniOdrive（电机驱动器）** 和 **PioPulse（PC 端 TUI 可视化软件）**。当前硬件方案以 CAN 总线连接主控和电机驱动器，USB 串口用于 PC 调试与姿态可视化。

### ① ESP32-C3 $\leftrightarrow$ YYT MiniOdrive 电机驱动器

* **物理连接**：ESP32-C3 `GPIO6/GPIO7` 作为 TWAI `TX/RX`，经过 TJA1050T 转换为 CANH/CANL，再连接到所有 YYT MiniOdrive 驱动板。
* **总线类型**：Classic CAN，非 CAN FD。
* **建议波特率**：`1 Mbps`，需要与 `DriveFirmware/Core/Src/fdcan.c` 的 FDCAN 位时序保持一致。
* **命令帧**：主控周期性发送标准帧 `0x100` 和 `0x200`，每帧 8 字节，每两个字节对应一个电机的命令值。

命令帧 payload 使用小端序 `int16_t`，单位为 mV：

| 字节 | 含义 |
| :--- | :--- |
| `0..1` | 分组内第 1 个电机电压命令 |
| `2..3` | 分组内第 2 个电机电压命令 |
| `4..5` | 分组内第 3 个电机电压命令 |
| `6..7` | 分组内第 4 个电机电压命令 |

下位机 `can_bridge.c` 当前将命令限幅到 `+/-3.0V`，并在收到有效命令后进入 `mode=7`。超过 `100 ms` 没有收到新命令时，下位机会清零命令并退出该模式。

驱动器反馈帧使用标准 ID `0x100 + DRIVE_ID`，8 字节小端序：

| 字节 | 类型 | 含义 |
| :--- | :--- | :--- |
| `0..3` | `int32_t` | 电机角度，单位 mrad |
| `4..5` | `int16_t` | 电机速度，单位 `rpm x10` |
| `6..7` | `int16_t` | 预留 |

### ② ESP32-C3 $\leftrightarrow$ PioPulse 终端可视化
* **物理连接**：ESP32-C3 的 Native USB/调试串口（`GPIO 18/19`）通过 USB 线连接至 PC 电脑。
* **串口配置**：波特率 **`115200`**，流控无。
* **TUI 帧传输协议 (FireWater)**：
  * **数据格式**：以 `\n`（换行符）结尾的逗号分割 ASCII 字符串。
  * **输出示例**：`imu:pitch_val,roll_val,yaw_val,0.0,0.0,0.0\n` （前三位为 MPU6050 经过互补滤波后的 Pitch, Roll, Yaw 姿态度数；后三位为平移占位符，PioPulse 内置的 3D 立方体控件将会直接读取这 6 个通道渲染出实时 3D 姿态）。
