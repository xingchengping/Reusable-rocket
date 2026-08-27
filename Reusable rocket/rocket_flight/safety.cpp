// =============================================================================
// safety.cpp — 安全与应急逻辑实现
// =============================================================================
#include "safety.h"
#include "actuators.h"
#include <Arduino.h>

#ifdef ESP32
  #include "esp_task_wdt.h"
#endif

static uint16_t g_fault_code = 0;
static uint8_t  g_consecutive_faults = 0;
static char     g_abort_reason[64] = "";
static bool     g_aborted = false;

// 看门狗状态
static bool g_wdt_enabled = false;


safety_level_t safety_check(const sensor_status_t *sen,
                             const nav_state_t *nav,
                             bool rc_kill) {
    g_fault_code = 0;
    bool any_fault = false;

    // 1. IMU 超时检测
    if (sen->imu_fault_cnt >= SAFETY_MAX_CONSECUTIVE_FAULTS) {
        g_fault_code |= FAULT_IMU_TIMEOUT;
        any_fault = true;
    }

    // 2. 气压计超时检测
    if (sen->baro_fault_cnt >= SAFETY_MAX_CONSECUTIVE_FAULTS) {
        g_fault_code |= FAULT_BARO_TIMEOUT;
        any_fault = true;
    }

    // 3. 倾角过大
    float tilt = sqrtf(nav->roll * nav->roll + nav->pitch * nav->pitch);
    if (tilt > SAFETY_MAX_TILT_RAD) {
        g_fault_code |= FAULT_TILT_EXCEED;
        any_fault = true;
    }

    // 4. 高度超限
    if (nav->pos_z > SAFETY_MAX_ALT_M || nav->pos_z < -10.0f) {
        g_fault_code |= FAULT_ALT_EXCEED;
        any_fault = true;
    }

    // 5. 水平漂移超限
    if (nav->horiz_range > SAFETY_MAX_HORIZ_M) {
        g_fault_code |= FAULT_HORIZ_EXCEED;
        any_fault = true;
    }

    // 6. 垂直速度异常
    if (fabsf(nav->vel_z) > SAFETY_MAX_VZ_MPS) {
        g_fault_code |= FAULT_VZ_EXCEED;
        any_fault = true;
    }

    // 7. 遥控急停
    if (rc_kill) {
        g_fault_code |= FAULT_RC_KILL;
        any_fault = true;
    }

    // 8. #隐患6: 发动机熄火检测 (加速度突降)
    //    待实现: 需在 IMU 世界系 Z 轴加速度 + 油门指令可用后启用,
    //    判断条件: 飞行中 && throttle>30% && |acc_z_world + g| 偏小 → 疑似熄火
    //    (原简化实现将速度差当加速度且数值错误, 已移除)

    // 9. 连续故障计数
    if (any_fault) {
        g_consecutive_faults++;
    } else {
        g_consecutive_faults = 0;
    }

    // 判定安全等级
    if (g_consecutive_faults >= SAFETY_MAX_CONSECUTIVE_FAULTS) {
        return SAFETY_ABORT;
    } else if (g_consecutive_faults >= 2) {
        return SAFETY_CRITICAL;
    } else if (any_fault) {
        return SAFETY_WARNING;
    }
    return SAFETY_OK;
}


uint16_t safety_get_fault_code() {
    return g_fault_code;
}


void safety_abort_execute(const char *reason) {
    if (g_aborted) return;   // 防止重复执行
    g_aborted = true;

    // 记录原因
    strncpy(g_abort_reason, reason, sizeof(g_abort_reason) - 1);
    g_abort_reason[sizeof(g_abort_reason) - 1] = '\0';

    // 紧急停止执行器
    actuators_emergency_stop();

    // 蜂鸣器报警 (500ms 长响)
    if (PIN_BUZZER > 0) {
        tone(PIN_BUZZER, 1000, 500);
    }

    Serial.printf("[SAFETY] 紧急终止! 原因: %s\n", reason);
}


void safety_wdt_enable() {
#ifdef ESP32
    // ESP32 任务看门狗 (Arduino-ESP32 v3.x 新 API)
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = 3000,          // 3秒超时
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_cfg);     // 初始化
    esp_task_wdt_add(NULL);          // 添加当前任务
    g_wdt_enabled = true;
    Serial.println("[SAFETY] 看门狗已启用 (3s)");
#endif
}


void safety_wdt_feed() {
#ifdef ESP32
    if (g_wdt_enabled) {
        esp_task_wdt_reset();
    }
#endif
}


const char* safety_get_abort_reason() {
    return g_aborted ? g_abort_reason : "";
}

