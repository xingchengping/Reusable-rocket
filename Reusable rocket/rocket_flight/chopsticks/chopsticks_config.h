// chopsticks_config.h — 筷子架硬件配置
#pragma once
#include <Arduino.h>

// ======================== 串口 ===============================================
#define CHOP_UART_NUM  1          // Serial1
#define CHOP_BAUD      115200     // 与 PC 通信波特率

// ======================== 伺服 ===============================================
#define PIN_SERVO_LEFT   13       // 左臂舵机
#define PIN_SERVO_RIGHT  14       // 右臂舵机
#define PIN_LOCK_SERVO   15       // 锁止舵机

// 舵机角度范围
#define CHOP_OPEN_ANGLE      90   // 全开 (度)
#define CHOP_HALF_ANGLE      35   // 预捕获半开 (度)
#define CHOP_CLOSED_ANGLE    10   // 全闭 (度)
#define CHOP_LOCKED_ANGLE    5    // 锁止位置

// ======================== 传感器 =============================================
#define PIN_US_TRIG       16       // 超声波 Trig
#define PIN_US_ECHO       17       // 超声波 Echo

#define PIN_BEAM_LOW      18       // 对射管低位 (1.2m)
#define PIN_BEAM_HIGH     21       // 对射管高位 (1.8m)
#define PIN_MANUAL_OPEN   19       // 手动打开按钮 (物理按钮 + PC远程GPIO)

// ======================== 捕获参数 ===========================================
// FSM 决策阈值 (超声波 + 对射管)
#define CHOP_DIST_TRACKING      8.0f    // 进入 TRACKING 的最大距离 (m)
#define CHOP_DIST_TRACKING_MIN  1.5f    // 进入 TRACKING 的最小距离 (m)
#define CHOP_DIST_PRE_CAPTURE   3.0f    // 进入 PRE_CAPTURE 的最大距离 (m)
#define CHOP_CAPTURE_WIN_MIN    0.5f    // 捕获窗口最小距离 (m) — 太近=错过
#define CHOP_CAPTURE_WIN_MAX    1.8f    // 捕获窗口最大距离 (m)
#define CHOP_DIST_TOO_CLOSE     0.3f    // 过近未触发 → 放弃 (m)
#define CHOP_DIST_IDLE          5.0f    // 回到 IDLE 的距离 (m)
#define CHOP_IDLE_TIMEOUT_MS    3000    // IDLE 超时 (ms)
#define CHOP_CAPTURE_SPEED_MAX  2.0f    // 捕获允许最大下降速度 (m/s)
#define CHOP_SPEED_ABORT        3.0f    // 触发 FAILSAFE 的下降速度 (m/s)
#define CHOP_DIST_ABORT         3.0f    // 触发 FAILSAFE 的距离内 (m)

// ======================== 协议 ===============================================
#define CHOP_SYNC_PC2CHOP  0xFD    // PC → 筷子架
#define CHOP_SYNC_CHOP2PC  0xFE    // 筷子架 → PC

// 指令
#define CHOP_CMD_OPEN       0x01
#define CHOP_CMD_CLOSE      0x02
#define CHOP_CMD_LOCK       0x03
#define CHOP_CMD_ESTOP      0x04
