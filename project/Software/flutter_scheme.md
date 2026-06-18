# VaporOS // Flutter 跨平台轮腿控制器技术方案 (Flutter Technical Spec)

本方案旨在利用 **Flutter** 框架构建一个能够在 Linux (Fedora/GNOME)、Windows、macOS 以及 Android 平板/手机上原生运行的 **Vaporwave 蒸汽波 + ASCII Art** 风格开发者控制台 App，并实现与主控板的 USB 串口通信及低延迟音效合成。

---

## 1. 技术栈选型 (Technology Stack)

*   **UI 渲染引擎**：Flutter SDK (Dart 3.x)
*   **物理串口通信**：
    *   桌面端 (Linux, Windows, macOS): [flutter_libserialport](https://pub.dev/packages/flutter_libserialport) (基于 `libserialport` C 库，对 Fedora 完美支持)
    *   移动端 (Android): [usb_serial](https://pub.dev/packages/usb_serial) (基于 Android USB Host API，免 Root 支持 Android USB OTG 串口通信)
*   **字体库**：[google_fonts](https://pub.dev/packages/google_fonts) (支持 `JetBrains Mono` 和 `IBM Plex Mono`)
*   **低延迟音效引擎**：[soundpool](https://pub.dev/packages/soundpool) (用于机械键盘咔哒声、蜂鸣声等毫秒级响应音效)
*   **状态管理**：`Provider` 或 `flutter_bloc` (处理串口数据流、遥测数据动态刷新)

---

## 2. 依赖配置 (`pubspec.yaml`)

```yaml
name: vapor_coprocessor_controller
description: "Vaporwave CRT Serial Controller for Parallel Leg-Wheel Robot"
publish_to: 'none'
version: 1.0.0+1

environment:
  sdk: '>=3.0.0 <4.0.0'

dependencies:
  flutter:
    sdk: flutter
  
  # 跨平台串口驱动
  flutter_libserialport: ^0.4.0
  usb_serial: ^3.1.0 # (可选：仅限 Android 端使用)
  
  # 状态与工具
  provider: ^6.1.1
  google_fonts: ^6.1.0
  
  # 音效
  soundpool: ^2.3.0
  
  # 动画与矢量处理
  vector_math: ^2.1.4

flutter:
  uses-material-design: true
  assets:
    - assets/sounds/click.wav
    - assets/sounds/boot.wav
    - assets/sounds/hum.wav
```

---

## 3. 核心串口服务实现 (`lib/services/serial_service.dart`)

我们通过统一的抽象接口，使得 UI 层无需关心底层是 Desktop 串口还是 Android OTG 串口。

```dart
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter/foundation.dart';
import 'package:flutter_libserialport/flutter_libserialport.dart';

class SerialService extends ChangeNotifier {
  SerialPort? _port;
  SerialPortReader? _reader;
  bool _isConnected = false;
  final List<String> _logs = [];
  
  // 遥测数据流控制
  final StreamController<String> _lineStreamController = StreamController<String>.broadcast();
  Stream<String> get lineStream => _lineStreamController.stream;

  bool get isConnected => _isConnected;
  List<String> get logs => _logs;

  List<String> getAvailablePorts() {
    return SerialPort.availablePorts;
  }

  Future<bool> connect(String portName, {int baudRate = 115200}) async {
    try {
      _port = SerialPort(portName);
      if (!_port!.openReadWrite()) {
        _log("SYSTEM ERROR: Failed to open port $portName");
        return false;
      }
      
      // 配置串口波特率与帧格式
      SerialPortConfig config = _port!.config;
      config.baudRate = baudRate;
      config.bits = 8;
      config.stopBits = 1;
      config.parity = SerialPortParity.none;
      _port!.config = config;

      _isConnected = true;
      _log("CONNECTED TO SYSTEM ON $portName @ $baudRate");

      // 开启异步监听循环
      _reader = SerialPortReader(_port!);
      _reader!.stream.listen(_handleIncomingRawData, onError: (err) {
        _log("RX ERROR: $err");
        disconnect();
      });

      notifyListeners();
      return true;
    } catch (e) {
      _log("CONNECTION EXCEPTION: $e");
      return false;
    }
  }

  void _handleIncomingRawData(Uint8List data) {
    String chunk = utf8.decode(data, allowMalformed: true);
    // 行缓冲拼接逻辑
    _buffer += chunk;
    int lineEnd;
    while ((lineEnd = _buffer.indexOf('\n')) >= 0) {
      String line = _buffer.substring(0, lineEnd).trim();
      _buffer = _buffer.substring(lineEnd + 1);
      if (line.isNotEmpty) {
        _lineStreamController.add(line);
        _log(line);
      }
    }
  }

  String _buffer = "";

  Future<void> sendCommand(String cmd) async {
    if (!_isConnected || _port == null) return;
    try {
      final bytes = utf8.encode("$cmd\n");
      _port!.write(Uint8List.fromList(bytes));
      _log("> $cmd");
    } catch (e) {
      _log("TX ERROR: $e");
    }
  }

  void disconnect() {
    if (!_isConnected) return;
    _reader?.close();
    _port?.close();
    _isConnected = false;
    _log("DISCONNECTED FROM SYSTEM PORT");
    notifyListeners();
  }

  void _log(String msg) {
    _logs.add(msg);
    if (_logs.length > 500) _logs.removeAt(0); // 限制缓冲区大小
    notifyListeners();
  }
}
```

---

## 4. 蒸汽波 UI 设计规范与实现

### 4.1 颜色系统 (`lib/constants/colors.dart`)

```dart
import 'package:flutter/material.dart';

class VaporColors {
  static const Color bgDark = Color(0xFF0A0712);      // 深邃太空黑
  static const Color neonPink = Color(0xFFFF4FD8);    // 极光粉
  static const Color neonCyan = Color(0xFF2BE4FF);    // 霓虹青
  static const Color accentPurple = Color(0xFF8A4FFF); // 迷幻紫
  static const Color sunsetOrange = Color(0xFFFF9E57); // 晚霞橙
  static const Color gridLine = Color(0xFF2D2142);     // 网格暗紫
  static const Color textColor = Color(0xFFE8E8F0);    // 亮荧光白
}
```

### 4.2 3D 透视滚屏网格的 Flutter 实现 (`lib/widgets/neon_grid.dart`)

利用 `Transform` 矩阵组件进行 X 轴偏转，并在 `AnimationController` 中刷新贴图偏移，创造复古 3D 赛博空间感：

```dart
import 'package:flutter/material.dart';
import '../constants/colors.dart';

class NeonGridWidget extends StatefulWidget {
  const NeonGridWidget({Key? key}) : super(key: key);

  @override
  State<NeonGridWidget> createState() => _NeonGridWidgetState();
}

class _NeonGridWidgetState extends State<NeonGridWidget> with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 4),
    )..repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _controller,
      builder: (context, child) {
        return Transform(
          transform: Matrix4.identity()
            ..setEntry(3, 2, 0.003) // 透视矩阵投影
            ..rotateX(1.15),       // 绕X轴翻转，形成地平线视角
          alignment: Alignment.bottomCenter,
          child: Container(
            width: double.infinity,
            height: 350,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topCenter,
                end: Alignment.bottomCenter,
                colors: [
                  VaporColors.bgDark,
                  VaporColors.accentPurple.withOpacity(0.15),
                ],
              ),
            ),
            child: CustomPaint(
              painter: _GridLinePainter(_controller.value),
            ),
          ),
        );
      },
    );
  }
}

class _GridLinePainter extends CustomPainter {
  final double progress;
  _GridLinePainter(this.progress);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = VaporColors.accentPurple.withOpacity(0.2)
      ..strokeWidth = 1.0;

    // 绘制纵向射线网格
    const int linesCount = 20;
    for (int i = 0; i <= linesCount; i++) {
      double x = (size.width / linesCount) * i;
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }

    // 绘制横向滚屏横线
    double step = 40.0;
    double offset = progress * step;
    for (double y = offset; y < size.height; y += step) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }
  }

  @override
  bool shouldRepaint(covariant _GridLinePainter oldDelegate) => true;
}
```

### 4.3 CRT 扫描线与闪烁滤镜效果 (`lib/widgets/crt_screen.dart`)

为整个应用包裹一个 CRT 噪点扫描滤镜：

```dart
import 'dart:math';
import 'package:flutter/material.dart';

class CrtFilterWidget extends StatelessWidget {
  final Widget child;
  const CrtFilterWidget({Key? key, required this.child}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        child,
        // CRT 扫描线叠加层
        IgnorePointer(
          child: Container(
            decoration: const BoxDecoration(
              image: DecorationImage(
                image: AssetImage('assets/images/scanlines.png'), // 或者使用 CustomPaint 绘制
                repeat: ImageRepeat.repeat,
                opacity: 0.1,
              ),
            ),
          ),
        ),
        // 动态 CRT 横向扫描亮线动画
        const IgnorePointer(
          child: _CrtScanLineAnimation(),
        ),
      ],
    );
  }
}

class _CrtScanLineAnimation extends StatefulWidget {
  const _CrtScanLineAnimation({Key? key}) : super(key: key);

  @override
  State<_CrtScanLineAnimation> createState() => _CrtScanLineAnimationState();
}

class _CrtScanLineAnimationState extends State<_CrtScanLineAnimation> with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 8),
    )..repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _controller,
      builder: (context, child) {
        return Positioned(
          top: _controller.value * MediaQuery.of(context).size.height,
          left: 0,
          right: 0,
          height: 6,
          child: Container(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topCenter,
                end: Alignment.bottomCenter,
                colors: [
                  Colors.transparent,
                  Colors.white.withOpacity(0.06),
                  Colors.transparent,
                ],
              ),
            ),
          ),
        );
      },
    );
  }
}
```

---

## 5. UI 架构与控制交互

### 5.1 ASCII 风格仪表盘组件示例

```dart
class TelemetryBar extends StatelessWidget {
  final String label;
  final double value;
  final double max;

  const TelemetryBar({required this.label, required this.value, required this.max});

  @override
  Widget build(BuildContext context) {
    final int barsCount = 20;
    double progress = (value / max).clamp(0.0, 1.0);
    int filledCount = (progress * barsCount).round();
    
    String bar = '[' + '█' * filledCount + '░' * (barsCount - filledCount) + ']';

    return Text(
      "$label: $bar ${(progress * 100).toStringAsFixed(0)}%",
      style: GoogleFonts.ibmPlexMono(
        color: VaporColors.neonCyan,
        fontSize: 13,
        fontWeight: FontWeight.bold,
        shadows: [
          const Shadow(
            color: VaporColors.neonCyan,
            blurRadius: 8,
          ),
        ],
      ),
    );
  }
}
```

### 5.2 发送限制滑块 (Debounced PWM Slider)

为了防止快速滑动 Slider 时产生海量 I2C 报文导致系统卡死，需要在 Dart 中对串口发送函数进行防抖/节流限制：

```dart
import 'dart:async';
import 'package:flutter/material.dart';

class PwmControlSlider extends StatefulWidget {
  final String title;
  final Function(int) onPwmChanged;
  
  const PwmControlSlider({required this.title, required this.onPwmChanged});

  @override
  State<PwmControlSlider> createState() => _PwmControlSliderState();
}

class _PwmControlSliderState extends State<PwmControlSlider> {
  double _currentValue = 0;
  Timer? _throttleTimer;

  void _onSliderValueUpdated(double val) {
    setState(() {
      _currentValue = val;
    });

    // 节流处理：限制发送频率为每 60 毫秒一次 (约 16Hz)
    if (_throttleTimer == null || !_throttleTimer!.isActive) {
      _throttleTimer = Timer(const Duration(milliseconds: 60), () {
        widget.onPwmChanged(_currentValue.toInt());
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          justifyAxisAlignment: MainAxisAlignment.between,
          children: [
            Text(widget.title, style: TextStyle(color: VaporColors.sunsetOrange, fontSize: 11)),
            Text("${_currentValue.toInt()}", style: TextStyle(color: VaporColors.neonPink)),
          ],
        ),
        SliderTheme(
          data: SliderTheme.of(context).copyWith(
            thumbColor: VaporColors.neonPink,
            activeTrackColor: VaporColors.neonCyan,
            inactiveTrackColor: VaporColors.gridLine,
          ),
          child: Slider(
            value: _currentValue,
            min: -1000,
            max: 1000,
            onChanged: _onSliderValueUpdated,
          ),
        ),
      ],
    );
  }
}
```

---

## 6. 音效包集成

借助 `soundpool` 库，在串口动作或键盘按下时播放复古音效：

```dart
import 'package:flutter/services.dart';
import 'package:soundpool/soundpool.dart';

class SoundEffectsManager {
  static Soundpool? _pool;
  static int? _clickSoundId;
  static int? _bootSoundId;

  static Future<void> init() async {
    _pool = Soundpool.fromOptions(options: const SoundpoolOptions(maxStreams: 4));
    
    // 加载资源中的短音频
    _clickSoundId = await rootBundle.load("assets/sounds/click.wav").then((ByteData soundData) {
      return _pool!.load(soundData);
    });
    _bootSoundId = await rootBundle.load("assets/sounds/boot.wav").then((ByteData soundData) {
      return _pool!.load(soundData);
    });
  }

  static void playClick() {
    if (_pool != null && _clickSoundId != null) {
      _pool!.play(_clickSoundId!);
    }
  }

  static void playBoot() {
    if (_pool != null && _bootSoundId != null) {
      _pool!.play(_bootSoundId!);
    }
  }
}
```

---

## 7. 桌面端 (Linux/Fedora) 构建步骤

要在 Linux 端正常开发和构建此 App，需要配置串口 C 库的本地编译环境：

1.  **安装 Linux 底层 C 依赖库**：
    在 Fedora 系统上，执行以下命令安装 `libserialport` 开发包：
    ```bash
    sudo dnf install libserialport-devel clang make pkg-config gtk3-devel
    ```
2.  **启用 Linux 桌面端开发**：
    ```bash
    flutter config --enable-linux-desktop
    ```
3.  **运行程序**：
    ```bash
    flutter run -d linux
    ```
4.  **打包 Release 可执行文件**：
    ```bash
    flutter build linux --release
    ```
    构建产物将输出在 `build/linux/x64/release/bundle/` 中，可直接解压运行。
