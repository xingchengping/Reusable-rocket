# 🚀 可回收火箭

垂直起降 (VTVL) 火箭模型项目 — 模拟 SpaceX「筷子」机械臂空中捕获回收。
低空 (≤50m) 垂直起飞、悬停、降落，由地面「筷子架」机械臂在着陆瞬间捕获。

## ✨ 核心特性

- **双端混合控制**: 箭载 ESP32-S3 本地双环 PID 主控 (50Hz) + PC 端 LLM 修正 (25% 权重融合)
- **地面 LLM 控制器**: Qwen2.5-0.5B 量化模型 (llama-cpp-python) 实时输出油门/TVC 修正
- **「筷子」全自主捕获**: 独立 ESP32-S3, 超声波 + 双对射管 2-of-3 投票, 5 状态 FSM
- **一站式地面站**: Web 面板 (遥测/视频/一键指令/急停/起飞前自检)
- **多级安全**: 急停 (遥控+PC) / 看门狗 / 连续故障 ABORT / 燃料保护 / 飞行超时终止
- **数据记录**: SD 卡飞行日志 (CSV) + 遥测回传

## 🏗️ 系统架构

```
┌─────────────────────────────┐      ┌──────────────────────────────┐
│  火箭端 ESP32-S3 (N16R8-CAM) │      │  PC 地面站 (web_control.py)   │
│                             │      │                              │
│  传感器(200Hz)              │      │  Flask Web 面板 (8080)        │
│  ├─ MPU6050 IMU             │      │  ├─ 遥测/视频/一键指令        │
│  ├─ BMP280 气压计           │      │  ├─ 起飞前自检 + 急停         │
│  └─ OV2640 下视摄像头       │      │  └─ LLM 控制器 (10Hz, 25%修正)│
│       ↓                    │ 数传  │                              │
│  融合 fusion (200Hz)        │◄────►│  筷子架 ESP32-S3 (独立)       │
│  ├─ 姿态: 互补滤波          │ 电台  │  ├─ 超声波+对射管 2-of-3 投票  │
│  └─ 高度: 2态卡尔曼         │ UART  │  └─ 5状态 FSM 全自主捕获       │
│       ↓                    │      │                              │
│  本地双环 PID (50Hz)        │      │                              │
│  ├─ 内环姿态→TVC(±12°)     │      │                              │
│  └─ 外环高度→油门(重力补偿) │      │                              │
└─────────────────────────────┘      └──────────────────────────────┘
```

## 📁 目录结构

```
可回收火箭/
├── rocket_flight/          # 箭载固件 (Arduino C++, ESP32-S3)
│   ├── rocket_flight.ino   # 主程序/控制循环/遥测/起飞前自检
│   ├── config.h            # 引脚分配/物理参数/安全阈值
│   ├── sensors.*           # IMU / BMP280(trim补偿) / 激光测距
│   ├── fusion.*            # 姿态互补滤波 + 2态高度卡尔曼
│   ├── pid_local.*         # 内外双环 PID (推导见 docs/PID_DESIGN.md)
│   ├── actuators.*         # 舵机/泵/离散执行器 (非阻塞点火)
│   ├── link_comms.*        # 数传协议 (66B 遥测 + CRC8)
│   ├── safety.*            # 安全检测/看门狗/急停
│   ├── state_machine.*     # 飞行状态机
│   ├── camera.*            # 下视标记检测 + JPEG 回传
│   ├── sd_logger.*         # SD 飞行日志 (CSV)
│   └── chopsticks/         # 筷子架独立控制器
├── ai_controller/          # PC 地面站 (Python)
│   ├── web_control.py      # 一站式地面站 (推荐)
│   ├── llm_service.py      # LLM 着陆控制器
│   ├── pid_controller.py   # 地面 PID 备援 (仿真/参考)
│   ├── ground_station.py   # ⚠️ 已弃用 (由 web_control.py 取代)
│   ├── qwen2.5-0.5b-...gguf # 本地 LLM 模型 (491MB)
│   └── templates/          # Web 面板
└── docs/                   # 架构/协议/PID 设计文档
```

## 🛠️ 硬件需求

| 部件 | 规格 |
|---|---|
| 飞控 | ESP32-S3-N16R8-CAM (8MB PSRAM + 摄像头) |
| IMU | MPU6050 (I²C) |
| 气压计 | BMP280 (I²C) |
| 摄像头 | OV2640 板载 (DVP) |
| 执行器 | 2× 数字舵机 (TVC) + 硝酸泵 PWM + 5× 离散通道 |
| 数传 | SiK / ELRS 数传电台 (UART 57600) |
| 筷子架 | 独立 ESP32-S3 + HC-SR04 + 2× 对射管 + 3× 舵机 |
| 地面站 | 任意 PC, Python 3.9+, 可选 NVIDIA GPU 加速 LLM |

> 引脚分配已重排避免冲突 (v3.1)，详见 `rocket_flight/config.h`。
> 激光测距因引脚不足默认禁用 (PIN_TOF_* = -1)。

## 🚀 快速开始

### 1. 烧录箭载固件

用 Arduino IDE 打开 `rocket_flight/rocket_flight.ino`：

1. 安装 Arduino-ESP32 核心 (**v3.x**, 需 ≥3.0 以支持新看门狗/LEDC API)
2. 安装依赖库: `ESP32Servo`, `esp32-camera` (Espressif)
3. 开发板选择 **ESP32S3 Dev Module**, 开启 PSRAM (OPI PSRAM)
4. 烧录前通过 USB 串口可验证: `FIRE` / `STATUS` / `ABORT`

### 2. 烧录筷子架固件

打开 `rocket_flight/chopsticks/chopsticks.ino`，选择同一开发板烧录。

### 3. 安装地面站依赖

```bash
cd ai_controller
pip install -r requirements.txt
```

### 4. 运行地面站

```bash
# 火箭数传 COM3, 筷子架 COM4, 带 LLM 控制:
python web_control.py --port COM3 --chop-port COM4 \
    --model-path ./qwen2.5-0.5b-instruct-q4_k_m.gguf

# 无 LLM 模式 (仅监控面板, 火箭本地 PID 自主飞行):
python web_control.py --port COM3

# 查看可用串口:
python web_control.py --list
```

浏览器打开 **http://localhost:8080** 进入控制面板。

## 🎛️ Web 控制面板

| 功能 | 说明 |
|---|---|
| 📷 箭载摄像头 | MJPEG 实时回传 (0.5fps) |
| 📊 遥测 | 高度/垂直速度/水平误差/燃料/油门/阶段/安全等级 |
| 🚦 起飞前检查 | **IMU/气压计/执行器/燃料/安全全绿才解锁点火按钮** |
| 🔥 一键打火 | 带确认弹窗, 非 IDLE 状态自动禁用 |
| 💨 泄压 / 关阀 / 🪂 开伞 | 离散指令 (开伞会强制 ABORT) |
| ⛔ 紧急急停 ABORT | 通过协议 flags b1 远程终止 |
| ✋ 筷子架开/关 | 操作员手动干预 |

## 🔒 安全注意事项

1. **双组元推进剂 (HNO₃ + 乙醇) 具有腐蚀性和可燃性**, 操作必须佩戴防护装备, 远离人群
2. 点火为不可逆操作 — 面板会强制检查「起飞前自检」全绿
3. 急停通道: 遥控 GPIO (低电平, 3 次确认) + PC ABORT 按钮
4. 首次飞行建议: 无燃料台架测试 → 低推力地面绑绳测试 → 低空 (10m) 飞行
5. 总飞行超 8s 自动 ABORT; 燃料 <5% 强制低推力缓降

## 📖 文档

| 文档 | 内容 |
|---|---|
| `docs/ARCHITECTURE.md` | 系统架构/控制策略/状态机/安全机制 |
| `docs/PROTOCOL.md` | 通信协议定义 (66B 遥测/15B 指令/3B 离散/JPEG 分块) |
| `docs/PID_DESIGN.md` | 内外双环 PID 完整推导 |

## 📝 版本记录

- **v3.1** (当前): 修复点火失效/高度卡尔曼/协议吞包/引脚冲突等严重问题; 新增起飞前自检、PC 急停、断线重连; 统一物理参数; 完善文档
- **v3.0**: 本地 PID + 远程 LLM 混合控制架构
