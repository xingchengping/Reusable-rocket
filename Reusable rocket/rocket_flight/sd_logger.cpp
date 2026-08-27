// =============================================================================
// sd_logger.cpp — SD 卡数据记录实现
// -----------------------------------------------------------------------------
// 使用 Arduino SD 库 (SPI 模式)。文件名按序号递增: FLIGHT_001.CSV, FLIGHT_002.CSV...
// 缓冲机制: 先写入 RAM 缓冲, 满 SD_LOG_BUF_ROWS 行后一次写入 SD, 减少 IO 时间。
// =============================================================================
#include "sd_logger.h"
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

static File g_log_file;
static bool  g_sd_ready = false;
static log_record_t g_buf[SD_LOG_BUF_ROWS];
static uint16_t g_buf_idx = 0;
static uint32_t g_total_rows = 0;


bool sd_logger_begin() {
    // 初始化 SPI SD 卡
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

    if (!SD.begin(PIN_SD_CS)) {
        Serial.println("[SD] 初始化失败! 检查SD卡");
        g_sd_ready = false;
        return false;
    }

    // 查找可用文件名
    char fname[32];
    for (int i = 1; i <= 999; i++) {
        snprintf(fname, sizeof(fname), "/FLIGHT_%03d.CSV", i);
        if (!SD.exists(fname)) break;
    }

    g_log_file = SD.open(fname, FILE_WRITE);
    if (!g_log_file) {
        Serial.println("[SD] 无法创建文件!");
        g_sd_ready = false;
        return false;
    }

    // CSV 表头
    g_log_file.println("timestamp_ms,phase,alt,vz,pos_x,pos_y,vx,vy,"
                       "roll,pitch,yaw,p,q,r,throttle,fuel,"
                       "pitch_cmd,yaw_cmd,horiz_range,safety,"
                       "cam_valid,cam_dx_m,cam_dy_m,cam_conf,cam_time_us,"
                       "ai_height_err,ai_vel_err,ai_horiz_err,"
                       "ai_thr,ai_pitch,ai_yaw");
    g_log_file.flush();

    g_sd_ready = true;
    g_buf_idx = 0;
    g_total_rows = 0;
    Serial.printf("[SD] 日志文件: %s\n", fname);
    return true;
}


void sd_logger_write(const log_record_t *rec) {
    if (!g_sd_ready) return;

    g_buf[g_buf_idx++] = *rec;
    g_total_rows++;

    // 缓冲区满时刷新
    if (g_buf_idx >= SD_LOG_BUF_ROWS) {
        sd_logger_flush();
    }
}


void sd_logger_flush() {
    if (!g_sd_ready || g_buf_idx == 0) return;

    for (uint16_t i = 0; i < g_buf_idx; i++) {
        log_record_t *r = &g_buf[i];
        g_log_file.printf("%lu,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                          "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                          "%.3f,%.4f,%.2f,%.2f,%.3f,%u,"
                          "%u,%.3f,%.3f,%.2f,%lu\n",
            r->timestamp_ms, r->phase, r->alt, r->vz,
            r->pos_x, r->pos_y, r->vx, r->vy,
            r->roll, r->pitch, r->yaw, r->p, r->q, r->r,
            r->throttle, r->fuel_remaining,
            r->pitch_cmd, r->yaw_cmd, r->horiz_range, r->safety,
            r->cam_valid, r->cam_dx_m, r->cam_dy_m,
            r->cam_conf, r->cam_time_us,
            r->ai_input_0, r->ai_input_1, r->ai_horiz_err,
            r->ai_action_thr, r->ai_action_pitch, r->ai_action_yaw);
    }

    g_buf_idx = 0;
    g_log_file.flush();
}


uint32_t sd_logger_rows_written() {
    return g_total_rows;
}


void sd_logger_end() {
    sd_logger_flush();
    if (g_log_file) {
        g_log_file.close();
        g_sd_ready = false;
        Serial.printf("[SD] 日志关闭, 共 %lu 行\n", g_total_rows);
    }
}
