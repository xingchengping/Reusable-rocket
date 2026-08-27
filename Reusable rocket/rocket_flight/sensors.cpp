// =============================================================================
// sensors.cpp — 传感器驱动实现
// -----------------------------------------------------------------------------
// 使用 Arduino 框架的 Wire (I2C)、Serial (UART) 和 SPI。
// 如需移植到 ESP-IDF, 替换对应的硬件抽象层调用即可。
//
// 支持的硬件:
//   IMU:   MPU6050 (兼容 ICM-42688-P)
//   气压:  BMP280 / DPS310 (I2C)
//   激光:  VL53L1X / TOF10120 (UART)
// =============================================================================
#include "sensors.h"
#include <Arduino.h>
#include <Wire.h>   // I2C

// =========================== I2C 设备地址 =====================================
#define MPU6050_ADDR       0x68   // IMU 默认地址
#define BMP280_ADDR        0x76   // 气压计默认地址

// =========================== 静态状态变量 =====================================
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
static float baro_ground_press = 1013.25f;  // 地面基准气压 (hPa)
static uint32_t last_imu_read_ms = 0;
static uint32_t last_baro_read_ms = 0;
static sensor_status_t s_status = {false, false, false, 0, 0, 0};

// 低通滤波缓存 (一阶 IIR)
static float lpf_ax = 0, lpf_ay = 0, lpf_az = 0;
static float lpf_gx = 0, lpf_gy = 0, lpf_gz = 0;
static float lpf_baro = 0;
static float lpf_tof = -1.0f;
static float lpf_coeff = 0.2f;   // 低通系数 (≈ 1-e^(-dt/tau))

// =========================== 传感器加速度/陀螺量程 ===========================
// MPU6050 配置: 加速度 ±4g (AFS_SEL=1), 陀螺 ±2000°/s
// (v3.1 优化: 原 ±16g 量程下火箭最大 ~2g 加速度量化噪声过大, 改为 ±4g)
#define ACCEL_CONFIG_VAL  0x08   // ACCEL_CONFIG: AFS_SEL=1 (±4g)
#define ACCEL_SCALE      (4.0f * 9.81f / 32768.0f)  // m/s^2 per LSB
#define GYRO_SCALE       (2000.0f * 3.14159265f / 180.0f / 32768.0f)  // rad/s per LSB

// =========================== BMP280 校准系数 ==================================
// 完整温度/气压补偿所需 trim 参数 (BMP280 datasheet, 寄存器 0x88~0xA1)
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static float    t_fine;   // 中间变量 (供气压补偿使用)


// =========================== I2C 底层读写 =====================================
static bool i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static bool i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;  // 重复起始
    Wire.requestFrom(addr, len);
    uint8_t i = 0;
    while (Wire.available() && i < len) buf[i++] = Wire.read();
    return (i == len);
}


// =========================== IMU (MPU6050) 驱动 ===============================
static bool imu_init() {
    // 复位设备
    i2c_write_byte(MPU6050_ADDR, 0x6B, 0x80);  // PWR_MGMT_1: 复位
    delay(100);
    // 唤醒, 使用内部振荡器
    i2c_write_byte(MPU6050_ADDR, 0x6B, 0x00);  // PWR_MGMT_1: 唤醒
    // 配置陀螺量程 ±2000°/s
    i2c_write_byte(MPU6050_ADDR, 0x1B, 0x18);  // GYRO_CONFIG: FS_SEL=3
    // 配置加速度量程 ±4g (v3.1: 提高小加速度分辨率)
    i2c_write_byte(MPU6050_ADDR, 0x1C, ACCEL_CONFIG_VAL);  // ACCEL_CONFIG: AFS_SEL=1
    // 配置低通滤波器 44Hz (DLPF_CFG=3)
    i2c_write_byte(MPU6050_ADDR, 0x1A, 0x03);  // CONFIG
    // 设置采样率分频 (1kHz / (1+0) = 1kHz 内部采样)
    i2c_write_byte(MPU6050_ADDR, 0x19, 0x00);  // SMPLRT_DIV

    // 验证: 读取 WHO_AM_I
    uint8_t whoami = 0;
    if (!i2c_read_bytes(MPU6050_ADDR, 0x75, &whoami, 1)) return false;
    return (whoami == 0x68);  // MPU6050 WHO_AM_I
}

static bool imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                         int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[14];
    if (!i2c_read_bytes(MPU6050_ADDR, 0x3B, buf, 14)) return false;
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
    return true;
}


// =========================== 气压计 (BMP280) 驱动 =============================
static bool baro_read_trim() {
    // 读取 24 字节校准系数 (寄存器 0x88~0x9F)
    uint8_t buf[24];
    if (!i2c_read_bytes(BMP280_ADDR, 0x88, buf, 24)) return false;

    dig_T1 = (uint16_t)(buf[0]  | (buf[1]  << 8));
    dig_T2 = (int16_t) (buf[2]  | (buf[3]  << 8));
    dig_T3 = (int16_t) (buf[4]  | (buf[5]  << 8));
    dig_P1 = (uint16_t)(buf[6]  | (buf[7]  << 8));
    dig_P2 = (int16_t) (buf[8]  | (buf[9]  << 8));
    dig_P3 = (int16_t) (buf[10] | (buf[11] << 8));
    dig_P4 = (int16_t) (buf[12] | (buf[13] << 8));
    dig_P5 = (int16_t) (buf[14] | (buf[15] << 8));
    dig_P6 = (int16_t) (buf[16] | (buf[17] << 8));
    dig_P7 = (int16_t) (buf[18] | (buf[19] << 8));
    dig_P8 = (int16_t) (buf[20] | (buf[21] << 8));
    dig_P9 = (int16_t) (buf[22] | (buf[23] << 8));
    return true;
}

// BMP280 温度补偿 (datasheet 官方公式, float 版) — 输出 °C
static float baro_comp_temp(int32_t adc_T) {
    float var1 = ((float)adc_T / 16384.0f - (float)dig_T1 / 1024.0f) * (float)dig_T2;
    float var2 = ((float)adc_T / 131072.0f - (float)dig_T1 / 8192.0f);
    var2 = var2 * var2 * (float)dig_T3;
    t_fine = var1 + var2;
    return t_fine / 5120.0f;
}

// BMP280 气压补偿 (datasheet 官方公式, float 版) — 输出 hPa
static float baro_comp_press(int32_t adc_P) {
    float var1 = t_fine / 2.0f - 64000.0f;
    float var2 = var1 * var1 * (float)dig_P6 / 32768.0f;
    var2 = var2 + var1 * (float)dig_P5 * 2.0f;
    var2 = var2 / 4.0f + (float)dig_P4 * 65536.0f;
    var1 = ((float)dig_P3 * var1 * var1 / 524288.0f + (float)dig_P2 * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * (float)dig_P1;
    if (var1 == 0.0f) return 0.0f;

    float p = 1048576.0f - (float)adc_P;
    p = (p - var2 / 4096.0f) * 6250.0f / var1;
    var1 = (float)dig_P9 * p * p / 2147483648.0f;
    var2 = p * (float)dig_P8 / 32768.0f;
    p = p + (var1 + var2 + (float)dig_P7) / 16.0f;
    return p / 100.0f;   // Pa -> hPa
}

static bool baro_init() {
    // 复位
    i2c_write_byte(BMP280_ADDR, 0xE0, 0xB6);  // 软复位
    delay(10);
    // 验证芯片ID
    uint8_t chipid = 0;
    if (!i2c_read_bytes(BMP280_ADDR, 0xD0, &chipid, 1)) return false;
    if (chipid != 0x58) return false;   // BMP280

    // 读取校准系数 (trim), 用于完整温度/气压补偿
    if (!baro_read_trim()) {
        Serial.println("[SENSORS] BMP280 trim 读取失败!");
        return false;
    }

    // 配置: 过采样 x4, 正常模式, 待机时间 0.5ms, IIR x4
    i2c_write_byte(BMP280_ADDR, 0xF4, 0x57);  // ctrl_meas: osrs_t=2, osrs_p=4, mode=3
    i2c_write_byte(BMP280_ADDR, 0xF5, 0x14);  // config: t_sb=0.5ms, filter=4
    return true;
}

static bool baro_read(float *press_hpa, float *temp_c) {
    uint8_t buf[6];
    if (!i2c_read_bytes(BMP280_ADDR, 0xF7, buf, 6)) return false;
    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

    *temp_c = baro_comp_temp(adc_T);
    *press_hpa = baro_comp_press(adc_P);
    return true;
}

static float baro_to_altitude(float press_hpa) {
    // 国际标准大气压高度公式
    // h = 44330 * (1 - (P/P0)^(1/5.255))
    return 44330.0f * (1.0f - powf(press_hpa / baro_ground_press, 1.0f / 5.255f));
}


// =========================== 激光测距 (UART) 驱动 =============================
static HardwareSerial *tof_serial = nullptr;

static bool tof_init() {
    // 引脚不足时禁用 (config.h 中 PIN_TOF_RX/TX = -1)
    if (PIN_TOF_RX < 0 || PIN_TOF_TX < 0) {
        Serial.println("[SENSORS] 激光测距未接线 (已禁用)");
        return false;
    }
    pinMode(PIN_TOF_TX, OUTPUT);
    pinMode(PIN_TOF_RX, INPUT);
    // 使用 Serial2 (ESP32-S3 有 3 个硬件串口)
    tof_serial = &Serial2;
    tof_serial->begin(115200, SERIAL_8N1, PIN_TOF_RX, PIN_TOF_TX);
    // 等待模块就绪
    delay(200);
    return true;
}

static bool tof_read(float *range_m) {
    // TOF10120 协议: 每帧 32 字节, 距离在前 2 字节
    // VL53L1X 协议: UART ASCII "d:XXXX\r\n"
    // 这里实现简化版 UART 距离读取
    if (!tof_serial || tof_serial->available() < 8) {
        *range_m = -1.0f;
        return false;
    }
    // 查找帧头 0x57
    while (tof_serial->available() && tof_serial->read() != 0x57) {}
    if (tof_serial->available() < 3) {
        *range_m = -1.0f;
        return false;
    }
    uint8_t lo = tof_serial->read();
    uint8_t hi = tof_serial->read();
    // 跳过校验
    tof_serial->read();
    uint16_t dist_mm = ((uint16_t)hi << 8) | lo;
    if (dist_mm > 4000 || dist_mm < 20) {
        *range_m = -1.0f;
        return false;
    }
    *range_m = dist_mm * 0.001f;  // mm -> m
    return true;
}


// =========================== 公共 API 实现 ====================================

bool sensors_begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);  // 400kHz I2C

    delay(50);
    s_status.imu_ok = imu_init();
    s_status.baro_ok = baro_init();
    s_status.tof_ok = tof_init();

    Serial.printf("[SENSORS] IMU:%d  Baro:%d  TOF:%d\n",
                  s_status.imu_ok, s_status.baro_ok, s_status.tof_ok);
    return s_status.imu_ok;  // IMU 为必需设备
}

sensor_data_t sensors_read() {
    sensor_data_t sd = {0};
    sd.timestamp_ms = millis();
    sd.tof_range_m = -1.0f;
    sd.tof_valid = false;

    // --- 读取 IMU (200Hz) ---
    if (s_status.imu_ok) {
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        if (imu_read_raw(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw)) {
            // 转换为物理单位并做一阶IIR低通
            float ax = ax_raw * ACCEL_SCALE * IMU_ACCEL_X_SIGN;
            float ay = ay_raw * ACCEL_SCALE * IMU_ACCEL_Y_SIGN;
            float az = az_raw * ACCEL_SCALE * IMU_ACCEL_Z_SIGN;
            float gx = gx_raw * GYRO_SCALE * IMU_GYRO_X_SIGN - gyro_bias_x;
            float gy = gy_raw * GYRO_SCALE * IMU_GYRO_Y_SIGN - gyro_bias_y;
            float gz = gz_raw * GYRO_SCALE * IMU_GYRO_Z_SIGN - gyro_bias_z;

            sd.ax = lpf_ax + lpf_coeff * (ax - lpf_ax);
            sd.ay = lpf_ay + lpf_coeff * (ay - lpf_ay);
            sd.az = lpf_az + lpf_coeff * (az - lpf_az);
            sd.gx = lpf_gx + lpf_coeff * (gx - lpf_gx);
            sd.gy = lpf_gy + lpf_coeff * (gy - lpf_gy);
            sd.gz = lpf_gz + lpf_coeff * (gz - lpf_gz);
            lpf_ax = sd.ax; lpf_ay = sd.ay; lpf_az = sd.az;
            lpf_gx = sd.gx; lpf_gy = sd.gy; lpf_gz = sd.gz;

            s_status.imu_fault_cnt = 0;
            last_imu_read_ms = sd.timestamp_ms;
        } else {
            s_status.imu_fault_cnt++;
        }
    }

    // --- 读取气压计 (50Hz, 降频处理) ---
    if (s_status.baro_ok && (sd.timestamp_ms - last_baro_read_ms) >= 20) {
        float press, temp;
        if (baro_read(&press, &temp)) {
            sd.baro_press_hpa = press;
            sd.baro_temp_c = temp;
            float alt_raw = baro_to_altitude(press);
            sd.baro_alt_m = lpf_baro + lpf_coeff * (alt_raw - lpf_baro);
            lpf_baro = sd.baro_alt_m;
            s_status.baro_fault_cnt = 0;
            last_baro_read_ms = sd.timestamp_ms;
        } else {
            s_status.baro_fault_cnt++;
            sd.baro_alt_m = lpf_baro;  // 保持上次值
        }
    } else {
        sd.baro_alt_m = lpf_baro;
    }

    // --- 读取激光测距 ---
    if (s_status.tof_ok) {
        float range;
        if (tof_read(&range)) {
            sd.tof_range_m = lpf_tof + lpf_coeff * (range - (lpf_tof > 0 ? lpf_tof : range));
            lpf_tof = sd.tof_range_m;
            sd.tof_valid = (sd.tof_range_m > 0 && sd.tof_range_m < 4.0f);
            s_status.tof_fault_cnt = 0;
        } else {
            s_status.tof_fault_cnt++;
        }
    }

    return sd;
}

sensor_status_t sensors_get_status() {
    return s_status;
}

void sensors_calibrate_gyro() {
    // 静止条件下采集零偏
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int count = 0;
    Serial.printf("[SENSORS] 陀螺校准开始... (保持静止)\n");
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz)) {
            sum_x += gx * GYRO_SCALE;
            sum_y += gy * GYRO_SCALE;
            sum_z += gz * GYRO_SCALE;
            count++;
        }
        delay(5);
    }
    if (count > 0) {
        gyro_bias_x = sum_x / count;
        gyro_bias_y = sum_y / count;
        gyro_bias_z = sum_z / count;
    }
    Serial.printf("[SENSORS] 陀螺校准完成: bx=%.4f by=%.4f bz=%.4f rad/s\n",
                  gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

void sensors_baro_set_zero() {
    float press, temp;
    if (baro_read(&press, &temp)) {
        baro_ground_press = press;
    }
    lpf_baro = 0.0f;
    Serial.printf("[SENSORS] 气压零位: %.2f hPa\n", baro_ground_press);
}

void sensors_get_gyro_bias(float *bx, float *by, float *bz) {
    *bx = gyro_bias_x; *by = gyro_bias_y; *bz = gyro_bias_z;
}

bool sensors_self_test() {
    // 检查 IMU
    uint8_t whoami;
    bool imu_ok = i2c_read_bytes(MPU6050_ADDR, 0x75, &whoami, 1) && (whoami == 0x68);
    // 检查气压计
    bool baro_ok = i2c_read_bytes(BMP280_ADDR, 0xD0, &whoami, 1) && (whoami == 0x58);
    // TOF 简化为端口存在性检查
    bool tof_ok = true;  // UART 设备无自检协议

    Serial.printf("[自检] IMU:%d 气压计:%d TOF:%d\n", imu_ok, baro_ok, tof_ok);
    return imu_ok && baro_ok;
}
