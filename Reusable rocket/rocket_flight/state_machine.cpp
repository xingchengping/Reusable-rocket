// =============================================================================
// state_machine.cpp — 飞行状态机实现
// =============================================================================
#include "state_machine.h"
#include "fusion.h"
#include <Arduino.h>

static flight_phase_t g_phase = FLIGHT_PHASE_IDLE;
static uint32_t g_ignite_time_ms = 0;      // 点火时刻
static uint32_t g_phase_start_ms = 0;      // 当前阶段开始时刻
static bool     g_ready = false;            // 允许发射标志 (由点火授权置位)


void state_machine_init() {
    g_phase = FLIGHT_PHASE_IDLE;
    g_ignite_time_ms = 0;
    g_phase_start_ms = millis();
    g_ready = false;
    Serial.println("[STATE] 状态机初始化 -> IDLE");
}


flight_phase_t state_machine_get_phase() {
    return g_phase;
}


uint32_t state_machine_get_flight_time_ms() {
    if (g_ignite_time_ms == 0) return 0;
    return millis() - g_ignite_time_ms;
}


void state_machine_command_ignite() {
    if (g_phase == FLIGHT_PHASE_IDLE) {
        // 点火授权: 调用方 (离散指令 0xEF/串口 FIRE) 已通过 CRC8 校验
        // 且需经地面站操作员确认, 此处直接置位 g_ready 并进入点火阶段。
        // (原实现要求 g_ready==true 但从未有任何代码置位, 导致永远无法点火)
        g_ready = true;
        g_phase = FLIGHT_PHASE_IGNITION;
        g_ignite_time_ms = millis();
        g_phase_start_ms = millis();
        Serial.println("[STATE] 点火!");
    } else {
        Serial.printf("[STATE] 拒绝点火: 当前阶段 %d 非 IDLE\n", (int)g_phase);
    }
}


bool state_machine_is_ready() {
    return g_ready;
}


void state_machine_set_abort() {
    g_phase = FLIGHT_PHASE_ABORT;
    Serial.println("[STATE] -> ABORT");
}


bool state_machine_update(const nav_state_t *nav, uint32_t flight_time_ms) {
    if (g_phase == FLIGHT_PHASE_ABORT) return false;
    if (g_phase == FLIGHT_PHASE_CAPTURE) return false;

    flight_phase_t old_phase = g_phase;
    float alt = nav->pos_z;

    // IDLE -> IGNITION (由外部命令触发, 不在这里自动转换)

    // IGNITION -> ASCENT (0.2s 后确认推力正常、有高度离开地面)
    if (g_phase == FLIGHT_PHASE_IGNITION
        && flight_time_ms > 200
        && alt > 0.3f) {
        g_phase = FLIGHT_PHASE_ASCENT;
        g_phase_start_ms = flight_time_ms;
    }

    // ASCENT -> HOVER (高度接近目标或时间到达)
    if (g_phase == FLIGHT_PHASE_ASCENT
        && (alt > TARGET_ALTITUDE_M - 5.0f || flight_time_ms > 2500)) {
        g_phase = FLIGHT_PHASE_HOVER;
        g_phase_start_ms = flight_time_ms;
    }

    // HOVER -> DESCENT (超过悬停窗口或燃料即将耗尽)
    if (g_phase == FLIGHT_PHASE_HOVER
        && (alt > TARGET_ALTITUDE_M + 3.0f || flight_time_ms > 3300)) {
        g_phase = FLIGHT_PHASE_DESCENT;
        g_phase_start_ms = flight_time_ms;
        Serial.println("[STATE] 开始下降...");
    }

    // DESCENT -> CAPTURE (触地或速度归零在合理高度)
    if (g_phase == FLIGHT_PHASE_DESCENT
        && alt < 0.2f
        && nav->vel_z > -0.5f) {
        g_phase = FLIGHT_PHASE_CAPTURE;
        Serial.println("[STATE] 捕获成功!");
    }

    // 超时保护 (总飞行超过 8 秒强制转 ABORT)
    if (flight_time_ms > 8000 && g_phase != FLIGHT_PHASE_IDLE) {
        g_phase = FLIGHT_PHASE_ABORT;
        Serial.println("[STATE] 飞行超时, 终止!");
    }

    if (g_phase != old_phase) {
        Serial.printf("[STATE] %d -> %d  (alt=%.2f t=%lums)\n",
                      old_phase, g_phase, alt, flight_time_ms);
    }
    return (g_phase != old_phase);
}
