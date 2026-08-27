// =============================================================================
// sensors.h / sensors.cpp — 传感器数据读取与低通滤波
// -----------------------------------------------------------------------------
// 功能:
//   1. 初始化 I2C 总线上的 IMU (MPU6050/ICM42688 等) 和气压计 (BMP280/DPS310)
//   2. 初始化 UART 激光测距模块 (VL53L1X 或类似)
//   3. 读取原始数据并做低通滤波
//   4. 提供传感器健康状态检查 (超时检测)
//   5. 校准函数 (地面静止时采集零偏)
//
// 传感器接口抽象:
//   - IMU: I2C, 地址 0x68 (默认), 采样率 200Hz
//   - 气压计: I2C, 地址 0x76, 采样率 50Hz
//   - 激光测距: UART, 波特率 115200, 10Hz
// =============================================================================
#ifndef ROCKET_SENSORS_H
#define ROCKET_SENSORS_H

#include "config.h"
#include <stdint.h>

// =========================== 传感器原始数据结构 ===============================
typedef struct {
    // --- IMU (加速度计: m/s^2, 陀螺仪: rad/s) ---
    float ax, ay, az;            // 三轴加速度 (体坐标系, 含重力)
    float gx, gy, gz;            // 三轴角速度 (体坐标系)

    // --- 气压高度 ---
    float baro_alt_m;            // 气压计推算高度 (m), 起始归零
    float baro_press_hpa;        // 原始气压值 (hPa)
    float baro_temp_c;           // 温度 (°C)

    // --- 激光测距 ---
    float tof_range_m;           // 激光测距值 (m), 有效范围 0.02~4m, 超量程返回 -1
    bool  tof_valid;             // 测距数据是否有效

    // --- 时间戳 ---
    uint32_t timestamp_ms;       // 毫秒时间戳
} sensor_data_t;

// =========================== 传感器状态 =======================================
typedef struct {
    bool imu_ok;                 // IMU 通信正常
    bool baro_ok;                // 气压计通信正常
    bool tof_ok;                 // 激光测距通信正常
    uint8_t imu_fault_cnt;       // 连续故障计数
    uint8_t baro_fault_cnt;
    uint8_t tof_fault_cnt;
} sensor_status_t;

// =========================== 滤波窗口 (滑动平均) ==============================
#define LP_FILTER_WINDOW  8      // 低通滤波窗口大小
#define GYRO_CALIB_SAMPLES 500   // 陀螺零偏校准采样数

// =========================== API 声明 =========================================

// 初始化所有传感器
bool sensors_begin();

// 读取并滤波传感器数据 (200Hz 调用)
sensor_data_t sensors_read();

// 获取传感器健康状态
sensor_status_t sensors_get_status();

// 陀螺仪零偏校准 (飞机静止时调用, 阻塞执行 ~3秒)
void sensors_calibrate_gyro();

// 气压计归零 (以当前高度为0)
void sensors_baro_set_zero();

// 获取陀螺零偏值
void sensors_get_gyro_bias(float *bx, float *by, float *bz);

// 传感器自检 (启动时调用, 检查所有设备在线)
bool sensors_self_test();

#endif // ROCKET_SENSORS_H
