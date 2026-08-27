// =============================================================================
// state_machine.h — 飞行状态机模块
// -----------------------------------------------------------------------------
// 管理飞行全生命周期: IDLE -> IGNITION -> ASCENT -> HOVER -> DESCENT -> CAPTURE
// 每个阶段有进入/退出动作和前向条件。
//
// 状态转换图:
//   IDLE ──(点火命令)──> IGNITION ──(推力ok,t>0.2s)──> ASCENT
//   ASCENT ──(高度>45m或t>2.5s)──> HOVER
//   HOVER ──(t>3.3s或高度>52m)──> DESCENT
//   DESCENT ──(捕获条件满足)──> CAPTURE
//   任意阶段 ──(急停触发)──> ABORT
// =============================================================================
#ifndef ROCKET_STATE_MACHINE_H
#define ROCKET_STATE_MACHINE_H

#include "config.h"
#include "fusion.h"

// 获取当前飞行阶段
flight_phase_t state_machine_get_phase();

// 初始化状态机 (进入 IDLE)
void state_machine_init();

// 状态机主更新 (每控制帧 50Hz 调用)
// 输入: 融合导航状态, 时间戳
// 返回: 是否触发了状态转换
bool state_machine_update(const nav_state_t *nav, uint32_t flight_time_ms);

// 强制转至 ABORT (安全模块触发)
void state_machine_set_abort();

// 获取飞控启动后的飞行时间 (毫秒)
uint32_t state_machine_get_flight_time_ms();

// 点火命令 (由地面站或按键触发)
void state_machine_command_ignite();

// 是否允许发射 (IDLE 阶段自检通过)
bool state_machine_is_ready();

#endif // ROCKET_STATE_MACHINE_H
