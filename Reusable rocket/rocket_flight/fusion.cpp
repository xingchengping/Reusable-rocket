// =============================================================================
// fusion.cpp — 传感器融合实现
// -----------------------------------------------------------------------------
// 核心算法:
//   1. 姿态互补滤波: attitude = 0.98*(attitude + gyro*dt) + 0.02*(accel_angle)
//   2. 高度卡尔曼: 预测(IMU_acc_z*dt) + 更新(气压计/激光)
//   3. 水平位置: 推力投影+加速度积分, 无 GPS 所以开环但有落点参考
// =============================================================================
#include "fusion.h"
#include <Arduino.h>
#include <math.h>

// =========================== 静态变量 =========================================
static nav_state_t g_nav;                // 当前导航状态
static kalman_1d_t g_kf_alt;             // 高度卡尔曼
static float g_target_x = 0, g_target_y = 0; // 落点坐标

// 互补滤波增益 (姿态)
static float cf_gain = 0.98f;            // 陀螺权重
static float g_dt = 0.005f;              // 默认 5ms

// 高度滤波器参数
#define ALT_KF_Q_POS    0.01f            // 高度过程噪声 m^2
#define ALT_KF_Q_VEL    0.05f            // 速度过程噪声 (m/s)^2
#define ALT_KF_R_BARO   0.25f            // 气压计噪声 m^2 (均方差0.5m)
#define ALT_KF_R_TOF    0.0025f           // 激光噪声 m^2 (均方差0.05m)

// =========================== 内部函数声明 =====================================
static void att_cf_update(float ax, float ay, float az, float gx, float gy, float gz, float dt);
static void alt_kf_predict(float acc_z_world, float dt);
static void alt_kf_update_baro(float baro_alt);
static void alt_kf_update_tof(float tof_range);
static void rot_body_to_world(float ax, float ay, float az,
                              float roll, float pitch, float yaw,
                              float *awx, float *awy, float *awz);

// =========================== 初始化 ===========================================
void fusion_init(float init_roll, float init_pitch, float init_yaw) {
    memset(&g_nav, 0, sizeof(g_nav));
    g_nav.roll = init_roll;
    g_nav.pitch = init_pitch;
    g_nav.yaw = init_yaw;

    // 高度卡尔曼初始化 (2 状态: [高度, 速度])
    memset(&g_kf_alt, 0, sizeof(g_kf_alt));
    g_kf_alt.x[0] = 0.0f;   // 高度初始为0 (地面)
    g_kf_alt.x[1] = 0.0f;   // 速度初始为0
    g_kf_alt.P[0][0] = 0.1f;   // 高度不确定度 0.1m
    g_kf_alt.P[0][1] = 0.0f;
    g_kf_alt.P[1][0] = 0.0f;
    g_kf_alt.P[1][1] = 0.5f;   // 速度不确定度 (m/s)^2
    g_kf_alt.Q[0] = ALT_KF_Q_POS;
    g_kf_alt.Q[1] = ALT_KF_Q_VEL;
    g_kf_alt.R_baro = ALT_KF_R_BARO;
    g_kf_alt.R_tof = ALT_KF_R_TOF;
}

// =========================== 主融合更新 =======================================
nav_state_t fusion_update(const sensor_data_t *sensors, float dt) {
    g_dt = (dt > 0.001f) ? dt : 0.005f;

    // 1. 姿态互补滤波
    att_cf_update(sensors->ax, sensors->ay, sensors->az,
                  sensors->gx, sensors->gy, sensors->gz, g_dt);

    // 2. 角速度直接赋值 (来自陀螺)
    g_nav.p = sensors->gx;
    g_nav.q = sensors->gy;
    g_nav.r = sensors->gz;

    // 3. 加速度转到世界坐标系 (用于位置/速度积分和高度卡尔曼)
    float awx, awy, awz;
    rot_body_to_world(sensors->ax, sensors->ay, sensors->az,
                      g_nav.roll, g_nav.pitch, g_nav.yaw,
                      &awx, &awy, &awz);
    g_nav.acc_x_world = awx;
    g_nav.acc_y_world = awy;
    g_nav.acc_z_world = awz - 9.81f;   // 去除重力

    // 4. 水平速度/位置积分 (开环, 漂移累积但飞行时间短 ~5s 可接受)
    g_nav.vel_x += g_nav.acc_x_world * g_dt;
    g_nav.vel_y += g_nav.acc_y_world * g_dt;
    g_nav.pos_x += g_nav.vel_x * g_dt;
    g_nav.pos_y += g_nav.vel_y * g_dt;

    // 5. 高度/垂直速度卡尔曼 (预测 + 气压计/激光更新)
    alt_kf_predict(g_nav.acc_z_world, g_dt);
    alt_kf_update_baro(sensors->baro_alt_m);
    if (sensors->tof_valid) {
        alt_kf_update_tof(sensors->tof_range_m);
    }
    g_nav.pos_z = g_kf_alt.x[0];   // 高度状态
    g_nav.vel_z = g_kf_alt.x[1];   // 垂直速度状态 (原实现错误地使用协方差元素)

    // 6. 融合高度
    g_nav.altitude_fused = g_kf_alt.x[0];

    // 7. 离落点水平距离
    float dx = g_nav.pos_x - g_target_x;
    float dy = g_nav.pos_y - g_target_y;
    g_nav.horiz_range = sqrtf(dx * dx + dy * dy);

    g_nav.timestamp_ms = millis();
    return g_nav;
}

nav_state_t fusion_get_state() {
    return g_nav;
}

void fusion_reset_horizontal() {
    g_nav.pos_x = 0; g_nav.pos_y = 0;
    g_nav.vel_x = 0; g_nav.vel_y = 0;
}

void fusion_set_target(float target_x, float target_y) {
    g_target_x = target_x;
    g_target_y = target_y;
}

// =========================== 姿态互补滤波 =====================================
static void att_cf_update(float ax, float ay, float az,
                          float gx, float gy, float gz, float dt) {
    // 从加速度计估算姿态角 (小角度假设, 适用于近垂直火箭)
    // roll  = atan2(ay, az)
    // pitch = atan2(-ax, sqrt(ay^2 + az^2))
    float acc_roll  = atan2f(ay, az);
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    // 偏航无法从加速度计获取

    // 陀螺积分
    float gyro_roll  = g_nav.roll  + gx * dt;
    float gyro_pitch = g_nav.pitch + gy * dt;
    float gyro_yaw   = g_nav.yaw   + gz * dt;

    // 互补滤波: 陀螺主导 + 加速度修正
    g_nav.roll  = cf_gain * gyro_roll  + (1.0f - cf_gain) * acc_roll;
    g_nav.pitch = cf_gain * gyro_pitch + (1.0f - cf_gain) * acc_pitch;
    g_nav.yaw   = gyro_yaw;   // 无磁力计, 纯积分

    // 限制姿态角范围 (火箭不应大角度)
    g_nav.roll  = CONSTRAIN(g_nav.roll,  -0.8f, 0.8f);
    g_nav.pitch = CONSTRAIN(g_nav.pitch, -0.8f, 0.8f);
}

// =========================== 高度卡尔曼 =======================================
// 标准 2 状态卡尔曼 (恒定加速度模型):
//   x = [高度, 速度]
//   F = [[1, dt], [0, 1]],  B = [[0.5*dt^2], [dt]],  u = acc_z_world
//   H = [1, 0]  (只观测高度)
static void alt_kf_predict(float acc_z_world, float dt) {
    float dt2 = 0.5f * dt * dt;

    // 状态预测
    float x0 = g_kf_alt.x[0] + g_kf_alt.x[1] * dt + dt2 * acc_z_world;
    float x1 = g_kf_alt.x[1] + dt * acc_z_world;

    // 协方差预测: P_pred = F P F' + Q
    float p00 = g_kf_alt.P[0][0], p01 = g_kf_alt.P[0][1];
    float p10 = g_kf_alt.P[1][0], p11 = g_kf_alt.P[1][1];
    g_kf_alt.P[0][0] = p00 + 2.0f * dt * p01 + dt * dt * p11 + g_kf_alt.Q[0];
    g_kf_alt.P[0][1] = p01 + dt * p11;
    g_kf_alt.P[1][0] = p10 + dt * p11;
    g_kf_alt.P[1][1] = p11 + g_kf_alt.Q[1];

    g_kf_alt.x[0] = x0;
    g_kf_alt.x[1] = x1;
}

// 通用量测更新 (H = [1, 0])
static void alt_kf_update(float meas, float R) {
    // 新息协方差: S = P[0][0] + R
    float S = g_kf_alt.P[0][0] + R;
    if (S < 1e-6f) S = 1e-6f;

    // 卡尔曼增益
    float K0 = g_kf_alt.P[0][0] / S;
    float K1 = g_kf_alt.P[1][0] / S;

    // 状态更新
    float innov = meas - g_kf_alt.x[0];
    g_kf_alt.x[0] += K0 * innov;
    g_kf_alt.x[1] += K1 * innov;

    // 协方差更新: P = (I - K H) P
    float p00 = g_kf_alt.P[0][0], p01 = g_kf_alt.P[0][1];
    float p10 = g_kf_alt.P[1][0], p11 = g_kf_alt.P[1][1];
    g_kf_alt.P[0][0] = (1.0f - K0) * p00;
    g_kf_alt.P[0][1] = (1.0f - K0) * p01;
    g_kf_alt.P[1][0] = p10 - K1 * p00;
    g_kf_alt.P[1][1] = p11 - K1 * p01;
}

static void alt_kf_update_baro(float baro_alt) {
    // 气压计更新 (始终可用)
    alt_kf_update(baro_alt, g_kf_alt.R_baro);
}

static void alt_kf_update_tof(float tof_range) {
    // 激光测距更新 (高精度, 小 R, 修正权重更高)
    alt_kf_update(tof_range, g_kf_alt.R_tof);
}

// =========================== 坐标旋转 (体->世界) =============================
static void rot_body_to_world(float ax, float ay, float az,
                              float roll, float pitch, float yaw,
                              float *awx, float *awy, float *awz) {
    // ZYX 旋转矩阵 R = Rz(yaw) * Ry(pitch) * Rx(roll)
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);

    // R 矩阵元素 (简化: R[3][3])
    float R00 = cp * cy;
    float R01 = sr * sp * cy - cr * sy;
    float R02 = cr * sp * cy + sr * sy;
    float R10 = cp * sy;
    float R11 = sr * sp * sy + cr * cy;
    float R12 = cr * sp * sy - sr * cy;
    float R20 = -sp;
    float R21 = sr * cp;
    float R22 = cr * cp;

    *awx = R00 * ax + R01 * ay + R02 * az;
    *awy = R10 * ax + R11 * ay + R12 * az;
    *awz = R20 * ax + R21 * ay + R22 * az;
}
