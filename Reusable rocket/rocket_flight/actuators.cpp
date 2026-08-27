// =============================================================================
// actuators.cpp — 执行器驱动实现
// -----------------------------------------------------------------------------
// 使用 ESP32 LEDC (LED PWM Controller) 生成高精度 PWM 信号。
// ESP32-S3 有 8 个高速 LEDC 通道，这里使用 3 个:
//   CH0: 俯仰舵机  CH1: 偏航舵机  CH2: 硝酸泵
//
// 舵机:
//   - 频率 333Hz (3ms 周期), 16位分辨率 (65536 steps)
//   - 脉宽范围 500~2500us
//   - 角度映射: us = 1500 + angle_deg * (500/12)
//
// 硝酸泵:
//   - 频率 25kHz (40us 周期)
//   - 占空比 0~100%, 16位分辨率
//
// 平滑过渡:
//   - 舵机速率限幅 600°/s (避免机械冲击)
//   - 油门变化率限幅 200%/s
// =============================================================================
#include "actuators.h"
#include <Arduino.h>

#ifdef ESP32
  // Arduino-ESP32 框架使用 ledc 函数
  #define USE_LEDC 1
#else
  #warning "非 ESP32 平台: PWM 输出可能不可用 (仿真模式)"
#endif

// LEDC 通道分配 (Arduino-ESP32 v3.x 用 ledcAttach 直接绑定引脚)
#define SERVO_PITCH_CH   0   // LEDC 通道 0
#define SERVO_YAW_CH     1   // LEDC 通道 1
#define PUMP_CH          2   // LEDC 通道 2

// 舵机角度-脉宽换算
#define SERVO_US_PER_DEG   (500.0f / 12.0f)   // 约 41.67 us/度
#define SERVO_CENTER       SERVO_CENTER_US     // 1500 us
#define SERVO_MIN          SERVO_MIN_US        // 500 us
#define SERVO_MAX          SERVO_MAX_US        // 2500 us

// 速率限幅 (每控制步, 50Hz -> dt=0.02s)
#define SERVO_RATE_LIMIT   (600.0f * 0.02f)    // 12 deg/步 @ 50Hz
#define THROTTLE_RATE_LIMIT (2.0f * 0.02f)     // 0.04/步 @ 50Hz

// 当前状态
static float g_cur_pitch = 0.0f;    // 当前舵角 (度)
static float g_cur_yaw   = 0.0f;
static float g_cur_thr   = 0.0f;    // 当前油门

// 离散执行器状态
static bool g_ignited        = false;
static bool g_igniting       = false;   // 点火器通电中 (非阻塞计时)
static uint32_t g_ignite_start_ms = 0;
static bool g_chute_deployed = false;
static bool g_ethanol_open   = true;   // 默认打开
static bool g_nitric_open    = true;   // 默认打开
static bool g_relief_open    = false;
static bool g_act_ok         = false;  // 执行器初始化成功 (起飞前自检)

// 点火器通电时长
#define IGNITER_ON_MS        2000    // 2 秒


void actuators_begin() {
#ifdef USE_LEDC
    // Arduino-ESP32 v3.x LEDC API: ledcAttach(pin, freq, resolution)
    // 配置俯仰舵机 PWM: 333Hz, 16bit
    ledcAttach(PIN_SERVO_PITCH, SERVO_FREQ_HZ, 16);
    // 配置偏航舵机 PWM
    ledcAttach(PIN_SERVO_YAW, SERVO_FREQ_HZ, 16);
    // 配置泵 PWM: 25kHz, 16bit
    ledcAttach(PIN_PUMP_PWM, PUMP_FREQ_HZ, 16);

    // 初始归中 / 归零
    actuators_emergency_stop();
#endif

    // 离散执行器引脚
    pinMode(PIN_IGNITER, OUTPUT);        digitalWrite(PIN_IGNITER, LOW);
    pinMode(PIN_VALVE_ETHANOL, OUTPUT);  digitalWrite(PIN_VALVE_ETHANOL, HIGH);  // 默认打开
    pinMode(PIN_VALVE_NITRIC, OUTPUT);   digitalWrite(PIN_VALVE_NITRIC, HIGH);
    pinMode(PIN_RELIEF_VALVE, OUTPUT);   digitalWrite(PIN_RELIEF_VALVE, LOW);
    pinMode(PIN_CHUTE_SERVO, OUTPUT);    digitalWrite(PIN_CHUTE_SERVO, LOW);

    g_act_ok = true;
    Serial.println("[ACTUATORS] 初始化完成 (3通道 PWM + 5离散通道)");
}


bool actuators_is_ok() {
    return g_act_ok;
}


void actuators_set(const actuator_cmd_t *cmd) {
    // 1. 速率限幅
    float dp = CONSTRAIN(cmd->pitch_angle_deg - g_cur_pitch,
                         -SERVO_RATE_LIMIT, SERVO_RATE_LIMIT);
    float dy = CONSTRAIN(cmd->yaw_angle_deg - g_cur_yaw,
                         -SERVO_RATE_LIMIT, SERVO_RATE_LIMIT);
    float dt = CONSTRAIN(cmd->throttle - g_cur_thr,
                         -THROTTLE_RATE_LIMIT, THROTTLE_RATE_LIMIT);

    g_cur_pitch += dp;
    g_cur_yaw   += dy;
    g_cur_thr   += dt;

    // 2. 限幅安全范围
    g_cur_pitch = CONSTRAIN(g_cur_pitch, -GIMBAL_ANGLE_DEG, GIMBAL_ANGLE_DEG);
    g_cur_yaw   = CONSTRAIN(g_cur_yaw,   -GIMBAL_ANGLE_DEG, GIMBAL_ANGLE_DEG);
    g_cur_thr   = CONSTRAIN(g_cur_thr,   0.0f, THROTTLE_SOFT_LIMIT);   // 软件限流 54%

    // 3. 计算舵机脉宽
    uint16_t pitch_us = (uint16_t)(SERVO_CENTER + g_cur_pitch * SERVO_US_PER_DEG);
    uint16_t yaw_us   = (uint16_t)(SERVO_CENTER + g_cur_yaw * SERVO_US_PER_DEG);
    pitch_us = (uint16_t)CONSTRAIN((int)pitch_us, (int)SERVO_MIN, (int)SERVO_MAX);
    yaw_us   = (uint16_t)CONSTRAIN((int)yaw_us,   (int)SERVO_MIN, (int)SERVO_MAX);

    actuators_send_servo_us(pitch_us, yaw_us);

    // 4. 输出泵 PWM (16bit: 占空比范围 0~65535)
    uint32_t pump_duty = (uint32_t)(g_cur_thr * 65535.0f);
    if (g_cur_thr < 0.01f) pump_duty = 0;   // 低于1%关泵
#ifdef USE_LEDC
    ledcWrite(PIN_PUMP_PWM, pump_duty);
#endif
}


void actuators_emergency_stop() {
    // 舵机归中
    actuators_send_servo_us(SERVO_CENTER, SERVO_CENTER);
    g_cur_pitch = 0; g_cur_yaw = 0; g_cur_thr = 0;
#ifdef USE_LEDC
    ledcWrite(PIN_PUMP_PWM, 0);  // 泵归零
#endif
    Serial.println("[ACTUATORS] 紧急停止!");
}


void actuators_send_servo_us(uint16_t pitch_us, uint16_t yaw_us) {
    // 将微秒值转为 16bit LEDC 占空比
    // 周期 = 1/SERVO_FREQ_HZ = 1/333 = 3003us
    // 占空比 = us / 3003 * 65536
    const float period_us = 1000000.0f / SERVO_FREQ_HZ;  // ~3003 us
    uint32_t pitch_duty = (uint32_t)(pitch_us / period_us * 65536.0f);
    uint32_t yaw_duty   = (uint32_t)(yaw_us   / period_us * 65536.0f);

#ifdef USE_LEDC
    ledcWrite(PIN_SERVO_PITCH, pitch_duty);
    ledcWrite(PIN_SERVO_YAW,   yaw_duty);
#endif
}


void actuators_get_state(float *pitch_deg, float *yaw_deg, float *throttle) {
    *pitch_deg = g_cur_pitch;
    *yaw_deg   = g_cur_yaw;
    *throttle  = g_cur_thr;
}


bool actuators_self_test() {
    // 输出扫频信号校验舵机是否在线 (观察机械响应)
    Serial.println("[ACTUATORS 自检] 俯仰舵机扫频...");
    for (int i = -10; i <= 10; i += 2) {
        actuators_send_servo_us(
            (uint16_t)(SERVO_CENTER + i * SERVO_US_PER_DEG), SERVO_CENTER);
        delay(100);
    }
    actuators_send_servo_us(SERVO_CENTER, SERVO_CENTER);
    Serial.println("[ACTUATORS 自检] 偏航舵机扫频...");
    for (int i = -10; i <= 10; i += 2) {
        actuators_send_servo_us(
            SERVO_CENTER, (uint16_t)(SERVO_CENTER + i * SERVO_US_PER_DEG));
        delay(100);
    }
    actuators_emergency_stop();
    Serial.println("[ACTUATORS 自检] 完成");
    return true;
}

// =========================== 离散执行器 ========================================
// 非阻塞点火: 仅置高电平并记录时间, 由 actuators_update() 在 2s 后自动断电。
// (v3.1 优化: 原实现 delay(2000) 会阻塞 50Hz 控制循环 2 秒)
void actuators_ignite() {
    if (g_igniting || g_ignited) return;
    digitalWrite(PIN_IGNITER, HIGH);
    g_igniting = true;
    g_ignite_start_ms = millis();
    Serial.println("[ACT] 点火 (通电中...)");
}

// 主循环每帧调用: 点火器通电计时
void actuators_update() {
    if (g_igniting && (millis() - g_ignite_start_ms) >= IGNITER_ON_MS) {
        digitalWrite(PIN_IGNITER, LOW);
        g_igniting = false;
        g_ignited = true;
        Serial.println("[ACT] 点火完成 (断电)");
    }
}

void actuators_relief_open() {
    digitalWrite(PIN_RELIEF_VALVE, HIGH);
    g_relief_open = true;
    Serial.println("[ACT] 泄压阀打开");
}

void actuators_relief_close() {
    digitalWrite(PIN_RELIEF_VALVE, LOW);
    g_relief_open = false;
    Serial.println("[ACT] 泄压阀关闭");
}

void actuators_ethanol_valve(bool open) {
    digitalWrite(PIN_VALVE_ETHANOL, open ? HIGH : LOW);
    g_ethanol_open = open;
    Serial.printf("[ACT] 乙醇阀门: %s\n", open ? "开" : "关");
}

void actuators_nitric_valve(bool open) {
    digitalWrite(PIN_VALVE_NITRIC, open ? HIGH : LOW);
    g_nitric_open = open;
    Serial.printf("[ACT] 硝酸阀门: %s\n", open ? "开" : "关");
}

void actuators_deploy_chute() {
    if (g_chute_deployed) return;
    // 释放降落伞: 舵机拉销
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_CHUTE_SERVO, HIGH);
        delay(100);
        digitalWrite(PIN_CHUTE_SERVO, LOW);
        delay(100);
    }
    g_chute_deployed = true;
    Serial.println("[ACT] 降落伞释放!");
}

bool actuators_is_ignited()       { return g_ignited; }
bool actuators_is_chute_deployed(){ return g_chute_deployed; }
bool actuators_is_ethanol_open()  { return g_ethanol_open; }
bool actuators_is_nitric_open()   { return g_nitric_open; }
