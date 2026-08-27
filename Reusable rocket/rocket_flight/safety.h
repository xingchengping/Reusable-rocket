// =============================================================================
// safety.h — 安全与应急逻辑模块
// -----------------------------------------------------------------------------
// 功能:
//   1. 实时异常检测: 传感器失效、推力异常、超出安全边界(高度/倾角/速度)
//   2. 安全等级评估: OK / WARNING / CRITICAL / ABORT
//   3. 紧急终止流程: 关闭推力、舵机归中、记录故障码
//   4. 看门狗定时器 (WDT): ESP32 硬件 WDT 防止程序死锁
//   5. 遥控紧急切断 (RC kill switch) 检测
//
// 异常检测机制:
//   - 传感器超时: 任何传感器超过 SAFETY_SENSOR_TIMEOUT_MS 未更新
//   - 倾角过大: tilt > SAFETY_MAX_TILT_RAD (45°)
//   - 高度超限: alt > SAFETY_MAX_ALT_M (80m)
//   - 水平漂移: horiz > SAFETY_MAX_HORIZ_M (30m)
//   - 速度异常: |vz| > SAFETY_MAX_VZ_MPS
//   - 连续故障计数 ≥ SAFETY_MAX_CONSECUTIVE_FAULTS -> ABORT
// =============================================================================
#ifndef ROCKET_SAFETY_H
#define ROCKET_SAFETY_H

#include "config.h"
#include "sensors.h"
#include "fusion.h"

// 故障码 (位掩码, 便于多故障同时记录)
typedef enum {
    FAULT_NONE          = 0x00,
    FAULT_IMU_TIMEOUT   = 0x01,   // IMU 超时
    FAULT_BARO_TIMEOUT  = 0x02,   // 气压计超时
    FAULT_TILT_EXCEED   = 0x04,   // 倾角过大
    FAULT_ALT_EXCEED    = 0x08,   // 高度超限
    FAULT_HORIZ_EXCEED  = 0x10,   // 水平漂移超限
    FAULT_VZ_EXCEED     = 0x20,   // 垂直速度异常
    FAULT_THRUST_ANOM   = 0x40,   // 推力异常 (燃料耗尽等)
    FAULT_RC_KILL       = 0x80,   // 遥控急停
    FAULT_FLAMEOUT      = 0x100,  // #隐患6: 发动机熄火/失速 (加速度突降)
    FAULT_WDT           = 0x200,  // 看门狗触发
} fault_code_t;

// 获取当前安全等级
safety_level_t safety_check(const sensor_status_t *sen,
                            const nav_state_t *nav,
                            bool rc_kill);

// 获取当前故障码
uint16_t safety_get_fault_code();

// 执行紧急终止:
//   - 设定油门为0、舵面归中
//   - 记录终止原因到SD
//   - 触发蜂鸣器
void safety_abort_execute(const char *reason);

// 初始化看门狗 (WDT, 超时 3 秒)
void safety_wdt_enable();

// 喂狗 (在主循环调用)
void safety_wdt_feed();

// 获取最近一次终止原因
const char* safety_get_abort_reason();

#endif // ROCKET_SAFETY_H
