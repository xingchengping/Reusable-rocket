// chopsticks.ino — 筷子架全自主捕获控制器
// ===================================================================
// 一切逻辑在 ESP32-S3 本地运行, 不依赖 PC。
// 唯一与地面通信: 收到任意串口字节 = 操作员手动打开筷子。
//
// 传感器:
//   - 超声波 HC-SR04: 测量火箭距离
//   - 对射管低位 (1.2m): 火箭进入夹持窗口
//   - 对射管高位 (1.8m): 火箭即将到达 (冗余)
//
// 执行器:
//   - 双舵机筷子臂 (left/right)
//   - 锁止舵机
//
// 控制逻辑:
//   - 5 状态 FSM (IDLE→TRACKING→PRE_CAPTURE→CAPTURE→LOCKED)
//   - 软阈值 (距离 ±5cm, 速度 ±0.3m/s)
//   - 信号稳定确认 (50ms 窗口)
//   - 超声波距离变化率 → 推算火箭速度
//   - 失败条件: 速度过快 / 距离跳变过大 → FAILSAFE 放弃捕获
// ===================================================================

#include "chopsticks_config.h"
#include <ESP32Servo.h>

// ======================== 状态定义 ===========================================
enum ChopState : uint8_t {
    CHOP_IDLE        = 0,  // 待机, 臂全开
    CHOP_TRACKING    = 1,  // 跟踪火箭下降
    CHOP_PRE_CAPTURE = 2,  // 预捕获, 臂半开就位
    CHOP_CAPTURE     = 3,  // 关臂捕获中
    CHOP_LOCKED      = 4,  // 已锁定
    CHOP_FAILSAFE    = 5,  // 安全模式, 放弃捕获
};

// ======================== 全局变量 ===========================================
static Servo servo_left;
static Servo servo_right;
static Servo servo_lock;

static ChopState g_state        = CHOP_IDLE;
static ChopState g_prev_state   = CHOP_IDLE;
static uint32_t  g_state_enter_ms = 0;
static uint32_t  g_last_sensor_ms = 0;
static uint32_t  g_last_serial_ms = 0;

// 传感器读数
static float    g_us_dist_m      = 999.0f;  // 当前超声波距离
static float    g_us_dist_prev   = 999.0f;  // 上一帧距离
static float    g_us_speed_ms    = 0.0f;    // 推算下降速度 (负=下降)
static bool     g_beam_low       = false;
static bool     g_beam_high      = false;
static bool     g_beam_low_prev  = false;

// 软阈值计时器
static uint32_t g_signal_stable_ms = 0;
static bool     g_signal_was_ready  = false;

// 手动标志
static bool     g_manual_override  = false;  // PC 端强制打开
static bool     g_estopped         = false;   // 急停 (传感器异常)

// ======================== CRC8 ==============================================
static uint8_t crc8(const uint8_t* data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc ^ b) & 1) crc = (crc >> 1) ^ 0x8C;
            else               crc >>= 1;
            b >>= 1;
        }
    }
    return crc;
}

// ======================== 传感器 =============================================
static float read_ultrasonic() {
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);

    long duration = pulseIn(PIN_US_ECHO, HIGH, 30000);  // 30ms ≈ 5m
    if (duration == 0) return -1.0f;

    return duration * 0.00017f;  // 声速340m/s, ÷2
}

static bool read_beam(uint8_t pin) {
    return digitalRead(pin) == LOW;  // LOW = 遮断
}

// ======================== 伺服控制 ===========================================
static void arm_open() {
    servo_left.write(CHOP_OPEN_ANGLE);
    servo_right.write(CHOP_OPEN_ANGLE);
    servo_lock.write(CHOP_OPEN_ANGLE);
}

static void arm_half() {
    servo_left.write(CHOP_HALF_ANGLE);
    servo_right.write(CHOP_HALF_ANGLE);
    servo_lock.write(CHOP_OPEN_ANGLE);  // 锁保持开
}

static void arm_close() {
    servo_left.write(CHOP_CLOSED_ANGLE);
    servo_right.write(CHOP_CLOSED_ANGLE);
}

static void arm_lock() {
    servo_left.write(CHOP_CLOSED_ANGLE);
    servo_right.write(CHOP_CLOSED_ANGLE);
    servo_lock.write(CHOP_LOCKED_ANGLE);  // 锁止
}

static void arm_stop() {
    // 急停: 立刻全开
    arm_open();
}

// ======================== 信号稳定确认 ========================================
// 信号必须持续 STABLE_MS 毫秒才算"稳定"
#define STABLE_MS 50

static bool signal_stable(bool condition, uint32_t now) {
    if (condition) {
        if (!g_signal_was_ready) {
            g_signal_was_ready = true;
            g_signal_stable_ms = now;
        }
        return (now - g_signal_stable_ms) >= STABLE_MS;
    } else {
        g_signal_was_ready = false;
        return false;
    }
}

// ======================== 传感器融合 ==========================================
// 判断火箭是否在捕获位置 (2-of-3 投票)
static bool fuse_capture_ready() {
    int votes = 0;

    // 条件A: 超声波距离在捕获窗口
    if (g_us_dist_m > CHOP_CAPTURE_WIN_MIN && g_us_dist_m < CHOP_CAPTURE_WIN_MAX) votes++;

    // 条件B: 低位对射管被遮断
    if (g_beam_low) votes++;

    // 条件C: 高位对射管被遮断 (火箭正在通过)
    if (g_beam_high) votes++;

    return votes >= 2;
}

// ======================== FSM 状态转换 =======================================
static void fsm_transition(ChopState new_state, const char* reason) {
    if (new_state == g_state) return;
    g_prev_state = g_state;
    g_state = new_state;
    g_state_enter_ms = millis();

    Serial.print("[CHOP] ");
    switch (g_state) {
        case CHOP_IDLE:        Serial.print("IDLE"); break;
        case CHOP_TRACKING:    Serial.print("TRACKING"); break;
        case CHOP_PRE_CAPTURE: Serial.print("PRE_CAPTURE"); break;
        case CHOP_CAPTURE:     Serial.print("CAPTURE"); break;
        case CHOP_LOCKED:      Serial.print("LOCKED"); break;
        case CHOP_FAILSAFE:    Serial.print("FAILSAFE"); break;
    }
    Serial.print(" ← ");
    switch (g_prev_state) {
        case CHOP_IDLE:        Serial.print("IDLE"); break;
        case CHOP_TRACKING:    Serial.print("TRACKING"); break;
        case CHOP_PRE_CAPTURE: Serial.print("PRE_CAPTURE"); break;
        case CHOP_CAPTURE:     Serial.print("CAPTURE"); break;
        case CHOP_LOCKED:      Serial.print("LOCKED"); break;
        case CHOP_FAILSAFE:    Serial.print("FAILSAFE"); break;
    }
    Serial.printf(" | %s\n", reason);
}

// ======================== FSM 主逻辑 =========================================
static void fsm_step(uint32_t now) {
    // ── 手动/急停优先 ──
    if (g_manual_override) {
        arm_open();
        fsm_transition(CHOP_IDLE, "手动打开");
        g_manual_override = false;
        return;
    }
    if (g_estopped) {
        arm_stop();
        fsm_transition(CHOP_FAILSAFE, "急停");
        return;
    }

    float dist   = g_us_dist_m;
    float speed  = g_us_speed_ms;
    bool  b_low  = g_beam_low;
    bool  b_high = g_beam_high;
    bool  sensor_ok = (dist > 0.1f && dist < 10.0f);

    // ── FAILSAFE 检查 (优先级最高) ──
    if (g_state == CHOP_FAILSAFE) {
        arm_open();
        return;  // 卡在安全模式, 需手动复位
    }

    // 传感器异常 → FAILSAFE
    if (!sensor_ok && g_state >= CHOP_TRACKING) {
        arm_stop();
        fsm_transition(CHOP_FAILSAFE, "传感器异常");
        return;
    }

    // 下降速度太快 → FAILSAFE (超声波推算速度超限)
    if (speed < -CHOP_SPEED_ABORT && dist < CHOP_DIST_ABORT && g_state >= CHOP_TRACKING) {
        arm_stop();
        fsm_transition(CHOP_FAILSAFE, "速度过快→放弃");
        return;
    }

    // ── LOCKED: 保持锁定 ──
    if (g_state == CHOP_LOCKED) {
        return;  // 已锁定, 不动
    }

    // ── CAPTURE: 关臂中, 到时间自动锁定 ──
    if (g_state == CHOP_CAPTURE) {
        if (now - g_state_enter_ms > 300) {  // 300ms 关臂时间
            arm_lock();
            fsm_transition(CHOP_LOCKED, "关臂完成→锁止");
        }
        return;
    }

    // ── 距离太远 → IDLE ──
    if (dist > CHOP_DIST_IDLE && g_state >= CHOP_TRACKING) {
        if (now - g_state_enter_ms > CHOP_IDLE_TIMEOUT_MS) {
            arm_open();
            fsm_transition(CHOP_IDLE, "距离过远");
        }
        return;  // 保持当前状态, 等待超时
    }

    // ── PRE_CAPTURE → CAPTURE ──
    //   条件: 2-of-3 投票通过 + 信号稳定 + 速度在可捕获范围
    if (g_state == CHOP_PRE_CAPTURE) {
        bool ready = fuse_capture_ready();
        bool stable = signal_stable(ready, now);

        if (stable && speed > -CHOP_CAPTURE_SPEED_MAX) {
            arm_close();
            fsm_transition(CHOP_CAPTURE, "传感器融合确认→关臂");
            return;
        }

        // 已到很低位置但还不满足 → 再等等
        if (dist < CHOP_DIST_TOO_CLOSE && !ready) {
            // 太近了还没触发对射管 → 可能偏了, 放弃
            arm_stop();
            fsm_transition(CHOP_FAILSAFE, "过近未触发→放弃");
            return;
        }
        return;
    }

    // ── TRACKING → PRE_CAPTURE ──
    //   条件: 距离进入预捕获区 + 速度在可控范围
    if (g_state == CHOP_TRACKING) {
        if (dist < CHOP_DIST_PRE_CAPTURE && speed > -CHOP_SPEED_ABORT && dist > CHOP_DIST_TOO_CLOSE) {
            arm_half();
            fsm_transition(CHOP_PRE_CAPTURE, "进入预捕获区");
            return;
        }
        return;
    }

    // ── IDLE → TRACKING ──
    //   条件: 检测到有物体从上方接近
    if (g_state == CHOP_IDLE) {
        if (dist < CHOP_DIST_TRACKING && speed < 0.0f && dist > CHOP_DIST_TRACKING_MIN) {
            fsm_transition(CHOP_TRACKING, "检测到下降物体");
            return;
        }
    }

}

// ======================== 初始化 =============================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n==============================================");
    Serial.println("  筷子架全自主捕获控制器 v2.0");
    Serial.println("  ESP32-S3 本地推理, 零PC依赖");
    Serial.println("==============================================\n");

    // 伺服
    servo_left.attach(PIN_SERVO_LEFT);
    servo_right.attach(PIN_SERVO_RIGHT);
    servo_lock.attach(PIN_LOCK_SERVO);
    arm_open();

    // 传感器
    pinMode(PIN_US_TRIG, OUTPUT);
    pinMode(PIN_US_ECHO, INPUT);
    pinMode(PIN_BEAM_LOW, INPUT_PULLUP);
    pinMode(PIN_BEAM_HIGH, INPUT_PULLUP);
    digitalWrite(PIN_US_TRIG, LOW);

    // 手动输入: 外部 GPIO (操作员物理按钮 + PC远程)
    pinMode(PIN_MANUAL_OPEN, INPUT_PULLUP);

    // 串口1: PC 远程手动打开 (任何字节 = 打开)
    Serial1.begin(115200);

    Serial.println("[CHOP] 就绪. 等待火箭下降...\n");
}

// ======================== 主循环 =============================================
void loop() {
    uint32_t now = millis();

    // ── 传感器采集 (20Hz) ──
    if (now - g_last_sensor_ms >= 50) {
        g_last_sensor_ms = now;

        // 超声波
        float new_dist = read_ultrasonic();
        if (new_dist > 0) {
            g_us_dist_prev = g_us_dist_m;
            g_us_dist_m = new_dist;
            // 从距离变化推算速度: v = Δd / Δt
            float dt = 0.05f;  // 50ms
            if (g_us_dist_prev < 900.0f) {
                g_us_speed_ms = (g_us_dist_m - g_us_dist_prev) / dt;
                // 简单低通滤波
                static float filt_speed = 0;
                filt_speed = 0.7f * g_us_speed_ms + 0.3f * filt_speed;
                g_us_speed_ms = filt_speed;
            }
        }

        // 对射管
        g_beam_low_prev = g_beam_low;
        g_beam_low  = read_beam(PIN_BEAM_LOW);
        g_beam_high = read_beam(PIN_BEAM_HIGH);
    }

    // ── PC 远程控制 (串口 \xFF=打开, \xFE=关闭) ──
    if (Serial1.available()) {
        int cmd = Serial1.read();
        while (Serial1.available()) Serial1.read();  // 清空缓冲区
        if (cmd == 0xFF) {
            g_manual_override = true;
            Serial.println("[CHOP] PC 远程手动打开!");
        } else if (cmd == 0xFE) {
            // 强制关闭: 跳转到 CAPTURE 状态
            arm_close();
            g_state = CHOP_CAPTURE;
            g_state_enter_ms = now;
            Serial.println("[CHOP] PC 远程手动关闭!");
        }
        g_last_serial_ms = now;
    }

    // ── 物理按钮手动打开 ──
    if (digitalRead(PIN_MANUAL_OPEN) == LOW) {
        delay(50);  // 去抖
        if (digitalRead(PIN_MANUAL_OPEN) == LOW) {
            g_manual_override = true;
            Serial.println("[CHOP] 物理按钮手动打开!");
        }
    }

    // ── FSM 决策 (20Hz) ──
    fsm_step(now);

    delay(5);
}
