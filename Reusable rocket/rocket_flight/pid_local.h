// pid_local.h — 箭载本地 PID 控制器 (内外双环, 50Hz)
// -------------------------------------------------------------------
// 内环: 姿态控制 → 保持火箭垂直 (pitch=0, yaw=0)
// 外环: 高度控制 → 飞到目标高度 50m, 含重力补偿
// LLM 地面指令以 25% 权重叠加修正
// -------------------------------------------------------------------
#pragma once
#include "config.h"
#include "fusion.h"

// 单轴 PID (与之前相同, 复用给内外环)
typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float i_max;
    float integral;
    float prev_error;
    float prev_deriv;
} pid_axis_t;

// 火箭 PID 控制器 (内外双环)
typedef struct {
    pid_axis_t  pid_att_pitch;  // 内环: 俯仰姿态 → TVC δpitch (度)
    pid_axis_t  pid_att_yaw;    // 内环: 偏航姿态 → TVC δyaw (度)
    pid_axis_t  pid_alt;        // 外环: 高度误差 → 推力 (N)
    float       target_alt;     // 目标高度 50m
    float       thrust_max;     // 最大推力 35N (65N × 54% 软限流)
    float       dry_mass;       // 干重 1.65kg
    float       fuel_initial;   // 初始燃料 0.15kg
} pid_controller_t;

// 控制输出
typedef struct {
    float throttle;       // 0~1
    float tvc_pitch_deg;  // 俯仰 TVC 角度 (度), ±12°
    float tvc_yaw_deg;    // 偏航 TVC 角度 (度), ±12°
} pid_output_t;

// API
void pid_init(pid_controller_t* pid, float thrust_max, float target_alt);
void pid_reset(pid_controller_t* pid);
void pid_step(pid_controller_t* pid,
              const nav_state_t* nav,
              float fuel_remaining_kg,
              pid_output_t* out);
