# 可回收火箭项目 — 架构文档

> 版本: v3.1 (2025, 全面修正优化后)
> 目标: 低空 (≤50m) 垂直起降 + 「筷子」机械臂空中捕获回收

## 1. 系统总览

```
┌─────────────────────────────┐      ┌──────────────────────────────┐
│  火箭端 ESP32-S3 (N16R8-CAM) │      │  PC 地面站 (web_control.py)   │
│                             │      │                              │
│  传感器(200Hz)              │      │  Flask Web 面板 (8080)        │
│  ├─ MPU6050 IMU             │      │  ├─ 遥测/视频/一键指令        │
│  ├─ BMP280 气压计           │      │  └─ 急停 ABORT (v3.1)         │
│  └─ OV2640 下视摄像头       │      │                              │
│       ↓                    │      │  LLM 控制器 (10Hz)            │
│  融合 fusion (200Hz)        │◄────►│  ├─ Qwen2.5-0.5B GGUF         │
│  ├─ 姿态: 互补滤波          │ 数传  │  └─ 25% 权重融合修正         │
│  └─ 高度: 2态卡尔曼(v3.1)   │ 电台  │                              │
│       ↓                    │ UART  │  筷子架 ESP32-S3 (独立)       │
│  本地双环 PID (50Hz)        │      │  ├─ 超声波+对射管 2-of-3 投票  │
│  ├─ 内环姿态→TVC(±12°)     │      │  └─ 5状态 FSM 全自主捕获       │
│  └─ 外环高度→油门(重力补偿) │      │                              │
│       ↓                    │      │                              │
│  执行器: 舵机/泵/阀/点火/伞  │      │                              │
│  遥测 20Hz + JPEG 0.5fps    │      │                              │
└─────────────────────────────┘      └──────────────────────────────┘
```

## 2. 控制架构 (关键设计)

**本地 PID 主控 + 地面 LLM 修正**: 火箭端 50Hz 双环 PID 是唯一权威控制源;
LLM 指令以 25% 权重叠加修正 (`rocket_flight.ino apply_control`), LLM 失败/超时
自动回退纯 PID, 保证地面推理错误不会导致失控。

- 内环姿态 PID: Kp=0.73, Ki=0.073, Kd=0.128 (由 Iyy=0.2, F=35N, 力臂0.5m 推导)
- 外环高度 PID: Kp=6.7, Ki=2.0, Kd=5.3 (含 m(t)·g 重力补偿)

## 3. 通信协议 (UART 57.6kbps, CRC8 Dallas)

| 包 | 方向 | 格式 |
|---|---|---|
| 遥测 66B | 火箭→PC | 0xCD + 14f + 4B + f + crc (含自检位) |
| 控制 15B | PC→火箭 | 0xAB + 3f + flags(B) + crc (b1=急停) |
| 离散 3B | PC→火箭 | 0xEF + cmd + crc (点火/阀门/泄压/开伞) |
| JPEG 分块 | 火箭→PC | 0xFC + 头7B + 220B块 + crc |

详细字段定义见 `docs/PROTOCOL.md`; 协议权威实现: `rocket_flight/link_comms.h`; PC 端解析: `web_control.py`。

## 4. 飞行状态机

```
IDLE ─点火命令(0xEF/串口FIRE)→ IGNITION ─(t>0.2s 且 alt>0.3m)→ ASCENT
ASCENT ─(alt>45m 或 t>2.5s)→ HOVER ─(alt>53m 或 t>3.3s)→ DESCENT
DESCENT ─(alt<0.2m 且 vz>-0.5)→ CAPTURE(筷子捕获, 立即收油)
任意阶段 ─(急停/安全故障/飞行超8s)→ ABORT
```

## 5. 安全机制

- 连续故障计数 ≥5 → ABORT (传感器超时/倾角>45°/高度>80m/水平漂移>30m/|vz|>20m/s)
- 遥控急停 GPIO (3 次确认防抖) + PC 端 ABORT 指令 (v3.1)
- 3s 任务看门狗 (Arduino-ESP32 v3.x API)
- 燃料 <5% 时强制低推力缓降

## 5.1 起飞前自检清单 (v3.1)

火箭端每个遥测周期上报 `sys_status` 位掩码, 地面站据此执行"全绿才允许点火":

| 项 | 位 | 说明 |
|---|---|---|
| IMU | 0x01 | 必需 |
| 气压计 | 0x02 | 必需 |
| 激光测距 | 0x04 | 可选 (未接不计入) |
| 执行器/舵机 | 0x08 | 必需 |
| 燃料 >20% | 0x10 | 必需 |
| READY | 0x80 | 必需项全过 + IDLE + 安全等级 OK |

Web 面板"起飞前检查"卡片逐项显示 ✓/✗, READY 位置位前点火按钮保持禁用。

## 6. 文件地图

```
rocket_flight/          箭载固件 (Arduino C++, ESP32-S3)
  rocket_flight.ino     主程序/控制循环/遥测/起飞前自检
  config.h              引脚/物理参数/安全阈值 (v3.1 重排引脚)
  sensors.*             IMU/BMP280(trim补偿)/TOF
  fusion.*              姿态互补滤波 + 2态高度卡尔曼 (v3.1 重写)
  pid_local.*           内外双环 PID (推导见 docs/PID_DESIGN.md)
  actuators.*           舵机/泵/离散执行器 (v3.1 非阻塞点火)
  link_comms.*          数传协议 (v3.1 修复吞包 + 66B 遥测)
  safety.*              安全检测/看门狗/急停
  state_machine.*       飞行状态机 (v3.1 修复点火)
  camera.*              标记检测 + JPEG 回传
  sd_logger.*           SD 飞行日志 (CSV)
  chopsticks/           筷子架独立控制器
ai_controller/          PC 地面站
  web_control.py        一站式地面站 (推荐, 含起飞前检查/急停)
  llm_service.py        LLM 着陆控制器 (llama-cpp-python)
  pid_controller.py     地面 PID 备援 (仿真/参考用)
  ground_station.py     ⚠️ 已弃用, 由 web_control.py 取代
  templates/            Web 面板
docs/                   架构/协议/PID 设计文档
```

## 7. 已知限制 / 待办

- 激光测距 (TOF) 默认禁用 (引脚不足, config.h PIN_TOF_* = -1)
- 熄火检测 (FAULT_FLAMEOUT) 待实现 (safety.cpp 有注释说明)
- 电池电压遥测为占位 (v_batt = 0)
- 水平位置为加速度开环积分 (无 GPS/视觉测距, 飞行时间短可接受)
- 加速度计 ±4g 量程已提高小加速度分辨率 (v3.1)
