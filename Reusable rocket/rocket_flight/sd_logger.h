// =============================================================================
// sd_logger.h — MicroSD 数据记录模块
// -----------------------------------------------------------------------------
// 功能:
//   1. 初始化 SPI 接口的 MicroSD 卡
//   2. 写入 CSV 格式的飞行数据 (状态量 + 控制量 + 时间戳)
//   3. 提供缓冲写入, 减少 SD 卡写入频率
//   4. 飞行结束后关闭文件
//
// CSV 列定义 (20 列):
//   timestamp_ms, phase, alt, vz, pos_x, pos_y, vx, vy,
//   roll, pitch, yaw, p, q, r, throttle, fuel, pitch_cmd, yaw_cmd,
//   horiz_range, safety
// =============================================================================
#ifndef ROCKET_SD_LOGGER_H
#define ROCKET_SD_LOGGER_H

#include "config.h"
#include <stdint.h>

// SD 卡写入缓冲区大小 (行数)
#define SD_LOG_BUF_ROWS     50

// 一条日志记录 (含摄像头数据)
typedef struct {
    uint32_t timestamp_ms;       // 毫秒时间戳
    uint8_t  phase;              // 飞行阶段
    float    alt;                // 高度 (m)
    float    vz;                 // 垂直速度 (m/s)
    float    pos_x, pos_y;       // 水平位置 (m)
    float    vx, vy;             // 水平速度 (m/s)
    float    roll, pitch, yaw;   // 姿态角 (rad)
    float    p, q, r;            // 角速度 (rad/s)
    float    throttle;           // 油门 (0~1)
    float    fuel_remaining;     // 剩余燃料 (kg)
    float    pitch_cmd, yaw_cmd; // 舵机指令 (度)
    float    horiz_range;        // 离落点水平距离 (m)
    uint8_t  safety;             // 安全等级
    // 摄像头数据 (新增)
    uint8_t  cam_valid;          // 摄像头检测有效
    float    cam_dx_m, cam_dy_m; // 摄像头估计的水平偏移 (m)
    float    cam_conf;           // 摄像头置信度
    uint32_t cam_time_us;        // 摄像头处理时间 (us)

    // #11: AI 推理快照 (输入-输出)
    float    ai_input_0;          // obs[0]=height_err ★最关键
    float    ai_input_1;          // obs[1]=vel_err
    float    ai_horiz_err;        // AI看到的水平误差
    float    ai_action_thr;       // 输出: 油门指令
    float    ai_action_pitch;     // 输出: 俯仰指令 (度)
    float    ai_action_yaw;       // 输出: 偏航指令 (度)
} log_record_t;

// 初始化 SD 卡
bool sd_logger_begin();

// 写入一条日志 (内部缓冲, 满时清空到 SD)
void sd_logger_write(const log_record_t *rec);

// 手动刷新缓冲到 SD (飞行结束前调用)
void sd_logger_flush();

// 获取已写入行数
uint32_t sd_logger_rows_written();

// 关闭 SD 文件
void sd_logger_end();

#endif // ROCKET_SD_LOGGER_H
