// link_comms.h — 数传链路通信 (透明串口桥接)
// 通过 SiK / ExpressLRS / 任意串口数传模块通信
// ESP32 端仅需 UART 读写, 无线部分由外置模块处理
#pragma once
#include <Arduino.h>
#include "config.h"

// ======================== 控制指令 (PC→火箭, 15B) =============================
#define CMD_SYNC 0xAB
typedef struct __attribute__((packed)) {
    uint8_t sync;       // 0xAB
    float   throttle;   // 0~1
    float   tvc_pitch;  // -1~1
    float   tvc_yaw;    // -1~1
    uint8_t flags;      // b0=reset b1=abort
    uint8_t crc8;
} link_cmd_t;

// ======================== 离散指令 (PC→火箭, 3B) ==============================
#define DISC_SYNC 0xEF
#define DISC_IGNITE         0x01  // 远程点火
#define DISC_RELIEF_OPEN    0x02  // 打开泄压阀 (燃烧室)
#define DISC_CLOSE_ETHANOL  0x03  // 关闭乙醇阀门
#define DISC_CLOSE_NITRIC   0x04  // 关闭硝酸阀门
#define DISC_DEPLOY_CHUTE   0x05  // 打开降落伞

typedef struct __attribute__((packed)) {
    uint8_t sync;   // 0xEF
    uint8_t cmd;    // 指令码
    uint8_t crc8;
} link_disc_cmd_t;

// ======================== 遥测 (火箭→PC, 66B) ================================
// sync(1) + 14 floats(56) + 4 uint8(4) + 1 float(4) + crc8(1) = 66
#define TELEM_SYNC 0xCD
typedef struct __attribute__((packed)) {
    uint8_t sync;           // 0xCD
    float   alt;            // [1] 高度 (m)
    float   vz;             // [2] 垂直速度 (m/s)
    float   vx;             // [3] 水平X速度
    float   vy;             // [4] 水平Y速度
    float   horiz_err;      // [5] 水平距离误差 (m)
    float   roll;           // [6]
    float   pitch;          // [7]
    float   yaw;            // [8]
    float   fuel;           // [9] 燃料比例 0~1
    float   throttle;       // [10] 当前油门 0~1
    float   cam_dx;         // [11] 摄像头偏移 X (m)
    float   cam_dy;         // [12] 摄像头偏移 Y (m)
    float   cam_conf;       // [13] 摄像头置信度
    float   reserved;       // [14] 预留
    uint8_t phase;          // [15] 飞行阶段
    uint8_t safety;         // [16] 安全等级
    uint8_t cam_valid;      // [17] 摄像头有效标志
    uint8_t sys_status;     // [18] 系统自检状态位 (v3.1, 见下方位定义)
    float   v_batt;         // [19] 电池电压 (V)
    uint8_t crc8;           // [20]
} link_telem_t;

// ======================== 系统自检状态位 (sys_status) =========================
// 起飞前自检清单 (地面站"全绿才允许点火"):
#define SYS_IMU_OK     0x01   // IMU 正常 (必需)
#define SYS_BARO_OK    0x02   // 气压计正常 (必需)
#define SYS_TOF_OK     0x04   // 激光测距正常 (可选, 禁用时常为 0)
#define SYS_SERVO_OK   0x08   // 舵机/执行器初始化正常 (必需)
#define SYS_FUEL_OK    0x10   // 燃料充足 (>20%) (必需)
#define SYS_READY      0x80   // 允许点火 (所有必需项通过 且 IDLE 且安全等级 OK)

// ======================== JPEG 帧 (火箭→PC, 可变长) ===========================
#define JPEG_SYNC       0xFC
#define JPEG_MAX_CHUNK  220     // 每块最多 220 字节 JPEG 数据
typedef struct __attribute__((packed)) {
    uint8_t  sync;          // 0xFC
    uint16_t seq;           // 帧序号
    uint8_t  chunk_idx;     // 当前块索引 (0-based)
    uint8_t  total_chunks;  // 总块数
    uint16_t jpeg_len;      // JPEG 总长度 (字节)
    uint8_t  data[JPEG_MAX_CHUNK];
    // crc8 在末尾, 实际长度 = 7 + min(JPEG_MAX_CHUNK, remaining) + 1
} link_jpeg_hdr_t;

// ======================== API ================================================
void    link_begin();
void    link_send_telem(const link_telem_t* pkt);
bool    link_recv_cmd(link_cmd_t* cmd);
bool    link_recv_disc_cmd(link_disc_cmd_t* cmd);
bool    link_cmd_timeout();
void    link_send_jpeg(const uint8_t* jpeg_buf, uint16_t jpeg_len);
void    link_status_print();
