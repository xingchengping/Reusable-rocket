# 通信协议定义 (v3.1)

> 数传链路 UART 57600 baud (SiK/ELRS 透明透传)
> 全部小端序 (struct.pack `<`), 每包末尾 CRC8 (Dallas/Maxim)
> 权威实现: `rocket_flight/link_comms.h` (固件) / `web_control.py` (PC)

## CRC8 (Dallas/Maxim)

多项式 0x8C (反向 0x31), 初值 0, 覆盖除 CRC 字节外的全部包内容。

```python
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        for _ in range(8):
            if (crc ^ b) & 1: crc = (crc >> 1) ^ 0x8C
            else:             crc >>= 1
            b >>= 1
    return crc & 0xFF
```

---

## 1. 遥测 (火箭 → PC, 66B)

`sync(1) + 14×float(56) + 4×uint8(4) + 1×float(4) + crc8(1) = 66B`

Python 格式: `"<B 14f 4B f B"` — sync 0xCD

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | sync | u8 | 0xCD |
| 1 | alt | f32 | 高度 (m) |
| 2 | vz | f32 | 垂直速度 (m/s, 上为正) |
| 3 | vx | f32 | 水平 X 速度 |
| 4 | vy | f32 | 水平 Y 速度 |
| 5 | horiz_err | f32 | 距筷子架水平距离 (m) |
| 6 | roll | f32 | 横滚 (rad) |
| 7 | pitch | f32 | 俯仰 (rad) |
| 8 | yaw | f32 | 偏航 (rad) |
| 9 | fuel | f32 | 燃料比例 0~1 |
| 10 | throttle | f32 | 当前油门 0~1 |
| 11 | cam_dx | f32 | 摄像头偏移 X (m) |
| 12 | cam_dy | f32 | 摄像头偏移 Y (m) |
| 13 | cam_conf | f32 | 摄像头置信度 0~1 |
| 14 | reserved | f32 | 预留 |
| 15 | phase | u8 | 飞行阶段 (见下) |
| 16 | safety | u8 | 安全等级 (见下) |
| 17 | cam_valid | u8 | 摄像头检测有效 0/1 |
| 18 | sys_status | u8 | 起飞前自检位掩码 (v3.1, 见下) |
| 19 | v_batt | f32 | 电池电压 (V, 暂为占位) |
| 20 | crc8 | u8 | 覆盖字节 0~19 |

### 飞行阶段 phase

`0=IDLE 1=IGNITION 2=ASCENT 3=HOVER 4=DESCENT 5=CAPTURE 6=ABORT`

### 安全等级 safety

`0=OK 1=WARNING 2=CRITICAL 3=ABORT`

### 自检位 sys_status (v3.1)

| 位 | 含义 | 必需 |
|---|---|---|
| 0x01 | IMU 正常 | ✅ |
| 0x02 | 气压计正常 | ✅ |
| 0x04 | 激光测距正常 | 可选 (未接常为 0) |
| 0x08 | 执行器/舵机初始化正常 | ✅ |
| 0x10 | 燃料 >20% | ✅ |
| 0x80 | **READY** = 必需项全过 + IDLE + safety==OK | 点火许可 |

---

## 2. 控制指令 (PC → 火箭, 15B)

`sync(1) + 3×float(12) + flags(1) + crc8(1) = 15B`

Python 格式: `"<B 3f B B"` — sync 0xAB

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | sync | u8 | 0xAB |
| 1 | throttle | f32 | 油门 0~1 (LLM 输出) |
| 2 | tvc_pitch | f32 | TVC 俯仰 -1~1 (×12° = 度) |
| 3 | tvc_yaw | f32 | TVC 偏航 -1~1 |
| 4 | flags | u8 | b0=reset, **b1=abort 急停** |
| 5 | crc8 | u8 | 覆盖字节 0~4 |

> flags b1 置位时火箭端立即 `state_machine_set_abort()` (地面急停入口)。

---

## 3. 离散指令 (PC → 火箭, 3B)

`sync(1) + cmd(1) + crc8(1) = 3B` — sync 0xEF

| cmd | 含义 |
|---|---|
| 0x01 | 远程点火 (DISC_IGNITE) |
| 0x02 | 打开燃烧室泄压阀 |
| 0x03 | 关闭乙醇阀门 |
| 0x04 | 关闭硝酸阀门 (同时停泵) |
| 0x05 | 释放降落伞 (强制 ABORT) |

---

## 4. JPEG 帧回传 (火箭 → PC, 分块)

`0xFC + 7B 头 + 数据块(≤220B) + crc8` — 每帧拆 N 块, 块大小固定 220B (末块除外)。

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | sync | u8 | 0xFC |
| 1~2 | seq | u16 | 帧序号 (递增) |
| 3 | chunk_idx | u8 | 块索引 (0-based) |
| 4 | total_chunks | u8 | 总块数 |
| 5~6 | jpeg_len | u16 | JPEG 总长度 (B) |
| 7.. | data | bytes | 块数据, 长度 = min(220, 剩余) |
| 末 | crc8 | u8 | 覆盖整包 |

PC 端按 `seq` 组装, 校验 `FF D8` 头 / `FF D9` 尾, 末块截断到 jpeg_len。

---

## 5. 筷子架通道 (PC → 筷子架 ESP32, 115200)

| 字节 | 含义 |
|---|---|
| 0xFF | 手动打开 (arm_open) |
| 0xFE | 手动关闭 (进入 CAPTURE) |

筷子架全自主捕获, 本通道仅供操作员干预。

---

## 6. 关键一致性

- 改协议必须**同步**修改: `link_comms.h` (结构) → `rocket_flight.ino` (发送) → `web_control.py` (`TELEM_FMT/TELEM_SIZE` + 解析)
- 遥测 66B @20Hz ≈ 1320 B/s, JPEG 0.5fps ≈ 2~4 KB/帧, 合计在 57.6kbps (≈7200 B/s) 预算内
