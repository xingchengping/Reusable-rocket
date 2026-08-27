// pid_local.cpp — 箭载内外双环 PID 实现 (50Hz, ES32 本地运行)
// ===================================================================
// 架构:
//   内环 (姿态): pitch/yaw → TVC 角度 (保持垂直, 目标 0°)
//       δ = Kp·θerr + Kd·θ̇err + Ki·∫θerr dt
//       推导: Iyy=0.200 kg·m², F=35N, L_arm=0.5m, ωn=8 rad/s, ζ=0.7
//       Kp = ωn²·I/(F·L) = 64/87.5 = 0.73
//       Kd = 2ζωn·I/(F·L) = 11.2/87.5 = 0.128
//       Ki = 0.1·Kp = 0.073
//
//   外环 (高度): alt error → 推力(N) → throttle, 含重力补偿
//       Fcmd = Kp·herr + Kd·ḣerr + Ki·∫herr dt + m(t)·g
//       推导: m_avg=1.67kg, ωn=2 rad/s, ζ=0.8
//       Kp = ωn²·m = 4×1.67 = 6.7 N/m
//       Kd = 2ζωn·m = 3.2×1.67 = 5.3 N·s/m
//       Ki = 0.3·Kp = 2.0 N/(m·s)
//
//   常量: g=9.81, Fmax=35N(65N×54%), h_target=50m, m_dry=1.65kg
//   120mm箭体: 起飞1.80kg, TWR 1.98
// ===================================================================
#include "pid_local.h"
#include <math.h>

#define GRAVITY 9.81f
#define RAD2DEG 57.29578f
#define DEG2RAD 0.0174533f
#define PID_DT 0.02f   // 50Hz

// ---- 单轴 PID ----
static void axis_init(pid_axis_t* a, float kp, float ki, float kd,
                       float out_min, float out_max, float i_max) {
    a->kp      = kp;
    a->ki      = ki;
    a->kd      = kd;
    a->out_min = out_min;
    a->out_max = out_max;
    a->i_max   = i_max;
    a->integral   = 0.0f;
    a->prev_error = 0.0f;
    a->prev_deriv = 0.0f;
}

static void axis_reset(pid_axis_t* a) {
    a->integral   = 0.0f;
    a->prev_error = 0.0f;
    a->prev_deriv = 0.0f;
}

static float axis_step(pid_axis_t* a, float error, float dt) {
    // 比例
    float p = a->kp * error;

    // 积分 (带限幅, 防饱和)
    a->integral += error * dt;
    if (a->integral > a->i_max)       a->integral = a->i_max;
    else if (a->integral < -a->i_max) a->integral = -a->i_max;
    float i = a->ki * a->integral;

    // 微分 (带一阶低通滤波)
    float deriv = 0.0f;
    if (dt > 1e-9f) {
        float raw = (error - a->prev_error) / dt;
        deriv = 0.7f * raw + 0.3f * a->prev_deriv;
        a->prev_deriv = deriv;
    }
    float d = a->kd * deriv;

    a->prev_error = error;

    // 限幅输出
    float out = p + i + d;
    if (out > a->out_max)       out = a->out_max;
    else if (out < a->out_min)  out = a->out_min;
    return out;
}

// =========================== 火箭 PID ==========================================
void pid_init(pid_controller_t* pid, float thrust_max, float target_alt) {
    pid->thrust_max  = thrust_max;
    pid->target_alt  = target_alt;
    pid->dry_mass    = ROCKET_DRY_MASS_KG;
    pid->fuel_initial= ROCKET_FUEL_MASS_KG;

    // ── 内环: 姿态 PID (误差=度, 输出=度) ──
    //   推导: Iyy=0.200 kg·m², F=35N, L_arm=0.5m, ωn=8 rad/s, ζ=0.7
    //   Kp = ωn²·I/(F·L) = 64/87.5 = 0.73
    //   Kd = 2ζωn·I/(F·L) = 11.2/87.5 = 0.128
    //   Ki = 0.1·Kp = 0.073 (小积分消除静态偏差)
    //   输出限幅 = ±12° (TVC 物理限制, 116mm内径下安全)
    axis_init(&pid->pid_att_pitch, 0.73f,  0.073f, 0.128f, -12.0f, 12.0f, 8.0f);
    axis_init(&pid->pid_att_yaw,   0.73f,  0.073f, 0.128f, -12.0f, 12.0f, 8.0f);

    // ── 外环: 高度 PID (误差=m, 输出=N) ──
    //   推导: m_avg=1.67kg, ωn=2 rad/s, ζ=0.8
    //   Kp = ωn²·m = 4×1.67 = 6.7 N/m
    //   Kd = 2ζωn·m = 3.2×1.67 = 5.3 N·s/m
    //   Ki = 0.3·Kp = 2.0 N/(m·s)
    //   输出限幅 = [0, Fmax*0.8] (推力不能为负)
    axis_init(&pid->pid_alt, 6.7f, 2.0f, 5.3f, -5.0f, thrust_max * 0.8f, thrust_max * 0.5f);
}

void pid_reset(pid_controller_t* pid) {
    axis_reset(&pid->pid_att_pitch);
    axis_reset(&pid->pid_att_yaw);
    axis_reset(&pid->pid_alt);
}

void pid_step(pid_controller_t* pid,
              const nav_state_t* nav,
              float fuel_remaining_kg,
              pid_output_t* out) {
    const float dt = PID_DT;

    float alt   = nav->pos_z;          // 当前高度 (m)
    float vz    = nav->vel_z;          // 垂直速度 (m/s, 向上为正)
    float pitch = nav->pitch;          // 俯仰角 (rad)
    float yaw   = nav->yaw;            // 偏航角 (rad)
    // 注: 角速度 q/r 不单独使用 — 内环 d 项由误差导数隐含 (见下方注释)

    // 转为度 (PID 以度为误差单位)
    float pitch_deg = pitch * RAD2DEG;
    float yaw_deg   = yaw * RAD2DEG;

    // ========================
    // 内环: 姿态控制
    // ========================
    //   目标: pitch=0, yaw=0 (垂直)
    //   误差 = 目标 - 当前 = 0 - current = -current
    float pitch_err = -pitch_deg;       // 角度误差 (度)
    float yaw_err   = -yaw_deg;
    // 角速度误差隐含在 axis_step 的 d 项中:
    //   de/dt = d(-current)/dt = -角速度, 等价于 kd*(-rate)
    float delta_pitch = axis_step(&pid->pid_att_pitch, pitch_err, dt);
    float delta_yaw   = axis_step(&pid->pid_att_yaw,   yaw_err,   dt);

    // ========================
    // 外环: 高度控制 (含重力补偿)
    // ========================
    float alt_err = pid->target_alt - alt;    // 高度误差: 目标 - 当前
    // 速度误差 (-vz) 由 axis_step 的 d 项隐含处理 (de/dt = -vz)

    // PID 输出 + 重力补偿
    // Fcmd = Kp_alt * alt_err + Kd_alt * vz_err + Ki_alt * ∫alt_err + m(t)·g
    float pid_force = axis_step(&pid->pid_alt, alt_err, dt);

    float mass = pid->dry_mass + fuel_remaining_kg;        // m(t) = m_dry + m_fuel
    float gravity_comp = mass * GRAVITY;                    // m(t) * g
    float f_cmd = pid_force + gravity_comp;                 // 总推力需求 (N)

    // 推力 → 油门
    if (f_cmd < 0.0f)      f_cmd = 0.0f;
    if (f_cmd > pid->thrust_max) f_cmd = pid->thrust_max;

    float throttle = f_cmd / pid->thrust_max;

    // 燃料保护: 低于 5% 时优先安全着陆 (降低推力)
    float fuel_ratio = fuel_remaining_kg / pid->fuel_initial;
    if (fuel_ratio < 0.05f) {
        if (alt < 3.0f)     throttle = 0.35f;          // 近地缓冲
        else if (alt < 15.0f) throttle = fminf(throttle, 0.3f);  // 慢降
        else                 throttle = fminf(throttle, 0.5f);
    }

    // 油门限幅
    if (throttle > 1.0f) throttle = 1.0f;
    if (throttle < 0.0f) throttle = 0.0f;

    out->throttle      = throttle;
    out->tvc_pitch_deg = delta_pitch;
    out->tvc_yaw_deg   = delta_yaw;
}
