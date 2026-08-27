// =============================================================================
// config.h — 飞行控制系统全局配置与常量定义
// -----------------------------------------------------------------------------
// 本文件定义 ESP32-S3 引脚分配、飞行参数、安全阈值等全局常量。
// 所有模块通过包含此文件获取统一的系统参数，确保上下游一致。
//
// 硬件平台: ESP32-S3-N16R8-CAM (CPU 240MHz, PSRAM 8MB)
// 框架: Arduino-ESP32 或 ESP-IDF (通过宏切换)
// =============================================================================
#ifndef ROCKET_CONFIG_H
#define ROCKET_CONFIG_H

#include <stdint.h>

// =========================== 编译模式选择 ====================================
// #define ESP_IDF      // 使用 ESP-IDF 时取消注释 (默认 Arduino)

// =========================== 系统参数 ========================================
#define MAIN_LOOP_FREQ_HZ      200      // 主循环频率: 200Hz (5ms周期)
#define CONTROL_FREQ_HZ         50      // AI推理+控制输出频率: 50Hz (20ms)
#define SENSOR_FREQ_HZ         200      // 传感器采样频率
#define SD_WRITE_FREQ_HZ        10      // SD卡写入频率 (降低卡负担)
#define TELEMETRY_FREQ_HZ       20      // 遥测发送频率 (与主控 50Hz 解耦, 降频省带宽)

// =========================== 物理参数 (v3: 120mm箭体, 1.80kg起飞, 35N推力) ======
// 箭体尺寸: 外径120mm, 内径116mm, 总长1200mm, 长细比10
// 推重比: 1.98 (35N / 1.80kg·g)
// 舱段: 头锥200 + 航电150 + 储罐250 + 泵阀180 + 发动机250 + 尾段170 = 1200mm
// 储罐: HNO₃ 40mm + 乙醇 30mm 并排 (116mm内径充裕)
// TVC: ±12°下位移 37.4mm, 剩余间隙 20.6mm, 安全
#define ROCKET_DRY_MASS_KG     1.65f    // 干重 (kg) — 120mm碳纤维箭体+设备
#define ROCKET_FUEL_MASS_KG    0.15f    // 推进剂总质量 (kg)
#define THRUST_MAX_N          35.0f    // 最大推力 (N) — 65N 额定 × 54% 软件限流
#define THROTTLE_SOFT_LIMIT   0.54f    // 油门软件限流比例 (54% → 35N, TWR 1.98)
#define GIMBAL_ANGLE_DEG      12.0f    // TVC 最大偏转角 (度)
#define GIMBAL_ANGLE_RAD      (GIMBAL_ANGLE_DEG * 3.14159265f / 180.0f)
#define BURN_TIME_S            3.3f    // 额定满推力燃烧时间 (s), 54%限流≈6.1s
#define PAD_RADIUS_M           0.05f   // 筷子捕获区半径 (m)
#define MAX_LANDING_VZ_MPS     1.0f    // 安全捕获最大垂直速度 (m/s)
#define TARGET_ALTITUDE_M     50.0f    // 目标最高高度 (m)
#define LEVER_ARM_M            0.5f    // TVC 力臂 (喷管→质心, m) — 与 pid_local 推导一致 (Iyy=0.2, F=35N)
#define ROCKET_OD_M            0.12f   // 箭体外径 (m)
#define ROCKET_ID_M            0.116f  // 箭体内径 (m)
#define ROCKET_LENGTH_M        1.2f    // 箭体总长 (m)
// PID 参数见 pid_local.cpp (内环姿态: 0.73/0.073/0.128; 外环高度: 6.7/2.0/5.3)

// =========================== 飞行阶段枚举 ====================================
typedef enum {
    FLIGHT_PHASE_IDLE     = 0,   // 待机, 地面检查
    FLIGHT_PHASE_IGNITION = 1,   // 点火, 推力加速
    FLIGHT_PHASE_ASCENT   = 2,   // 爬升段 (0~2.5s)
    FLIGHT_PHASE_HOVER    = 3,   // 悬停/过渡段 (2.5~3.3s)
    FLIGHT_PHASE_DESCENT  = 4,   // 下降段 (> 3.3s)
    FLIGHT_PHASE_CAPTURE  = 5,   // 被筷子捕获
    FLIGHT_PHASE_ABORT    = 6,   // 紧急终止
} flight_phase_t;

// =========================== 安全状态枚举 ====================================
typedef enum {
    SAFETY_OK       = 0,   // 正常
    SAFETY_WARNING  = 1,   // 警告 (偏离但仍可控)
    SAFETY_CRITICAL = 2,   // 危险 (即将超出包线)
    SAFETY_ABORT    = 3,   // 必须紧急终止
} safety_level_t;

// =========================== 引脚定义 (ESP32-S3) =============================
// -----------------------------------------------------------------------------
// ⚠️ 引脚分配原则 (v3.1 重排):
//   S3-CAM 板上摄像头 DVP 接口走线固定占用 GPIO 4~18 (见下方 CAM_*),
//   因此其余外设全部重排到空闲引脚, 消除原配置中的 6 处引脚冲突:
//   原冲突: RC_KILL(4)/SD_CS(5)/SERVO_YAW(10)/PUMP(11)/TOF_TX(13)/SD_SCK(18)
//   全部与摄像头共用, 已重新分配。
//   激光测距因引脚不足默认禁用 (PIN_TOF_* = -1)。
// -----------------------------------------------------------------------------
// --- I2C 传感器 ---
#define PIN_I2C_SDA        21      // I2C 数据线 (IMU + 气压计)
#define PIN_I2C_SCL        22      // I2C 时钟线

// --- SPI SD卡 (避开摄像头 DVP 引脚) ---
#define PIN_SD_CS          14      // SD 卡片选
#define PIN_SD_MOSI        23      // MOSI
#define PIN_SD_MISO        25      // MISO
#define PIN_SD_SCK         24      // SCLK

// --- 舵机 PWM ---
#define PIN_SERVO_PITCH    38      // 俯仰舵机 (PWM)
#define PIN_SERVO_YAW      39      // 偏航舵机 (PWM)

// --- 硝酸泵控制 ---
#define PIN_PUMP_PWM       40      // 硝酸泵 PWM

// --- 离散执行器 (点火/阀门/降落伞) ---
#define PIN_IGNITER         1      // 点火器 (MOSFET/继电器)
#define PIN_VALVE_ETHANOL   2      // 乙醇阀门 (高电平=开)
#define PIN_VALVE_NITRIC    3      // 硝酸阀门 (高电平=开) — 独立于泵PWM
#define PIN_RELIEF_VALVE   46      // 燃烧室泄压阀 (高电平=泄压)
#define PIN_CHUTE_SERVO    47      // 降落伞释放舵机

// --- 状态指示 ---
#define PIN_LED_STATUS     48      // 板载 RGB LED (S3-CAM, 仅作状态指示)
#define PIN_BUZZER         -1      // 蜂鸣器 (外接可选, -1=禁用)

// --- 遥控急停 (安全接收机, 低电平=急停, 3 次确认防抖) ---
#define PIN_RC_KILL        41      // 遥控急停信号

// --- 数传链路 UART1 (显式指定引脚, 避开摄像头/SD) ---
#define LINK_RX_PIN        45      // 数传 RX
#define LINK_TX_PIN        42      // 数传 TX

// --- 激光测距 (UART, 默认禁用: S3-CAM 摄像头占用大量 GPIO) ---
// 如需启用, 改为空闲引脚并确认与上表不冲突
#define PIN_TOF_RX         -1      // 激光测距串口 RX (-1=禁用)
#define PIN_TOF_TX         -1      // 激光测距串口 TX (-1=禁用)

// --- OV2640 摄像头 (DVP 8-bit 并行接口) ---
// 注意: 不同 ESP32-S3-CAM 板子引脚可能不同, 请根据实际连线调整
// 以下为常见配置 (FREENOVE / AI-Thinker 兼容), 占用 GPIO 4~18,
// 与上方外设引脚(1,2,3,14,21~25,38~42,45~48)无冲突
#define CAM_PIN_PWDN       -1      // 不使用掉电脚
#define CAM_PIN_RESET      -1      // 不使用复位脚 (或接 GPIO15)
#define CAM_PIN_XCLK       15      // XCLK 主时钟
#define CAM_PIN_SIOD        4      // SCCB SDA (类 I2C)
#define CAM_PIN_SIOC        5      // SCCB SCL
#define CAM_PIN_D7         16      // Y9
#define CAM_PIN_D6         17      // Y8
#define CAM_PIN_D5         18      // Y7
#define CAM_PIN_D4         12      // Y6
#define CAM_PIN_D3         10      // Y5
#define CAM_PIN_D2          8      // Y4
#define CAM_PIN_D1          9      // Y3
#define CAM_PIN_D0         11      // Y2
#define CAM_PIN_VSYNC       6      // VSYNC
#define CAM_PIN_HREF        7      // HREF
#define CAM_PIN_PCLK       13      // PCLK

// =========================== PWM 参数 ========================================
#define SERVO_MIN_US       500     // 舵机最小脉宽 (us)
#define SERVO_MAX_US      2500     // 舵机最大脉宽 (us)
#define SERVO_CENTER_US   1500     // 舵机中位脉宽 (us)
#define SERVO_FREQ_HZ      333     // 舵机刷新率 (333Hz = 3ms周期, 典型数字舵机)
#define PUMP_FREQ_HZ      25000    // 泵 PWM 频率 (25kHz, 超出人耳听觉范围)

// =========================== 传感器校准 =======================================
// IMU 安装坐标系: X=前(东), Y=左(北), Z=上(天)  (如果IMU坐标不符需外部旋转)
#define IMU_GYRO_X_SIGN     1      // 陀螺仪符号修正 (±1)
#define IMU_GYRO_Y_SIGN     1
#define IMU_GYRO_Z_SIGN     1
#define IMU_ACCEL_X_SIGN    1      // 加速度计符号修正
#define IMU_ACCEL_Y_SIGN    1
#define IMU_ACCEL_Z_SIGN    1

// =========================== 滤波器参数 ======================================
#define CF_ALPHA_IMU       0.98f   // 互补滤波: 陀螺仪权重 (姿态)
#define CF_ALPHA_ALT       0.95f   // 互补滤波: 气压计+加速度权重 (高度)
#define LPF_CUTOFF_HZ      30.0f   // 低通滤波截止频率 (Hz)
#define KALMAN_Q_BIAS      0.001f  // 卡尔曼陀螺偏置过程噪声
#define KALMAN_R_GYRO      0.03f   // 卡尔曼陀螺测量噪声
#define KALMAN_Q_ALT       0.01f   // 高度过程噪声
#define KALMAN_R_BARO      0.5f    // 气压计测量噪声 (m^2)

// =========================== 安全阈值 ========================================
#define SAFETY_MAX_TILT_DEG     45.0f   // 最大允许倾角 (度)
#define SAFETY_MAX_TILT_RAD     (SAFETY_MAX_TILT_DEG * 3.14159265f / 180.0f)
#define SAFETY_MAX_ALT_M        80.0f   // 最大允许高度 (m)
#define SAFETY_MAX_HORIZ_M      30.0f   // 最大水平偏移 (m)
#define SAFETY_MAX_VZ_MPS       20.0f   // 最大垂直速率 (m/s)
#define SAFETY_FUEL_EMPTY_FRAC   0.01f  // 燃料耗尽判定比例
#define SAFETY_SENSOR_TIMEOUT_MS 200   // 传感器超时 (ms)
#define SAFETY_MAX_CONSECUTIVE_FAULTS 5 // 连续异常计数触发急停
#define ABORT_THROTTLE_IDLE      0.0f   // 急停油门值
#define ABORT_SERVO_NEUTRAL      0.0f   // 急停舵面归中

// =========================== 数传链路 (主通信) =================================
// 通过 SiK / ExpressLRS / 任意串口数传模块通信
// UART1 显式引脚: RX=45, TX=42 (见上方引脚定义, 避开摄像头/SD)
#define LINK_UART_NUM    1              // 使用 Serial1
#define LINK_SERIAL      Serial1        // 数传串口对象 (对应 UART1)
#define LINK_BAUD        57600          // 波特率 (SiK 默认 57600, ELRS 420000)
#define CMD_TIMEOUT_US   300000         // 指令超时 300ms

// =========================== 辅助宏 ==========================================
#define ARRAY_SIZE(arr)    (sizeof(arr) / sizeof((arr)[0]))
#define CONSTRAIN(val, lo, hi)  (((val) < (lo)) ? (lo) : (((val) > (hi)) ? (hi) : (val)))
#ifndef DEG_TO_RAD
  #define DEG_TO_RAD(d)      ((d) * 3.1415926535f / 180.0f)
#endif
#ifndef RAD_TO_DEG
  #define RAD_TO_DEG(r)      ((r) * 180.0f / 3.1415926535f)
#endif

#endif // ROCKET_CONFIG_H
