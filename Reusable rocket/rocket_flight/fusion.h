// =============================================================================
// fusion.h / fusion.cpp — 传感器融合算法
// -----------------------------------------------------------------------------
// 功能:
//   1. 姿态估计: 互补滤波 (陀螺仪积分 + 加速度计修正)
//   2. 高度/速度估计: 卡尔曼滤波 (气压计 + 加速度计 + 激光测距)
//   3. 水平位置/速度估计: 加速度二次积分 (漂移由地面修正)
//
// 算法选型:
//   - 姿态: 互补滤波 (CF) → 计算量极小, ESP32上 ~5us, 陀螺积分主导短期精度,
//           加速度计修正长期漂移
//   - 高度: 一维卡尔曼 → 融合气压计低频精度 + IMU加速度高频响应 + 激光终段绝对基准
//
// 输出状态向量 (与世界坐标系对齐, ENU: 东-北-天):
//   position:  x(东), y(北), z(上=高度)  [m]
//   velocity:  vx, vy, vz               [m/s]
//   attitude:  roll, pitch, yaw          [rad]
//   angular_v: p, q, r                   [rad/s]
// =============================================================================
#ifndef ROCKET_FUSION_H
#define ROCKET_FUSION_H

#include "config.h"
#include "sensors.h"

// =========================== 导航状态结构 =====================================
typedef struct {
    // 位置 (ENU, m)
    float pos_x, pos_y, pos_z;       // pos_z = 高度

    // 速度 (ENU, m/s)
    float vel_x, vel_y, vel_z;

    // 姿态角 (rad, ZYX 欧拉)
    float roll, pitch, yaw;

    // 角速度 (体坐标系, rad/s)
    float p, q, r;

    // 加速度 (世界坐标系, m/s^2, 已去重力)
    float acc_x_world, acc_y_world, acc_z_world;

    // 对地高度 (融合后, m) — 核心导航变量
    float altitude_fused;

    // 离落点水平距离 (m)
    float horiz_range;

    // 时间戳
    uint32_t timestamp_ms;
} nav_state_t;

// =========================== 卡尔曼滤波器 (高度通道) ==========================
// 2 状态: x[0]=高度(m), x[1]=垂直速度(m/s, 向上为正)
typedef struct {
    float x[2];     // 状态向量: [高度, 速度]
    float P[2][2];  // 协方差矩阵
    float Q[2];     // 过程噪声 [高度, 速度]
    float R_baro;   // 气压计测量噪声
    float R_tof;    // 激光测量噪声
} kalman_1d_t;


// =========================== API 声明 =========================================

// 初始化融合器 (设置初始状态)
void fusion_init(float init_roll, float init_pitch, float init_yaw);

// 主融合更新 (每传感器周期调用)
nav_state_t fusion_update(const sensor_data_t *sensors, float dt);

// 获取当前导航状态
nav_state_t fusion_get_state();

// 归零水平位置 (地面参考)
void fusion_reset_horizontal();

// 设置落点位置
void fusion_set_target(float target_x, float target_y);

#endif // ROCKET_FUSION_H
