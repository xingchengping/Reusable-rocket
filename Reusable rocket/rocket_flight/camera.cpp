// =============================================================================
// camera.cpp — OV2640 摄像头驱动 + 筷子架标记点检测
// =============================================================================
#include "camera.h"
#include <Arduino.h>

#ifdef ESP32
  // esp32-camera 库头文件 (Arduino 库管理器安装: "ESP32 Camera" by Espressif)
  #include "esp_camera.h"
  #define HAS_ESP_CAM 1
#else
  #warning "非ESP32平台: 摄像头功能不可用"
  #define HAS_ESP_CAM 0
#endif

// =========================== OV2640 引脚配置 ==================================
// ESP32-S3-CAM 板的 DVP 接口引脚 (请根据实际板子调整)
// 常见配置 (ESP32-S3-EYE / FREENOVE ESP32-S3-CAM):
#ifdef HAS_ESP_CAM
static camera_config_t cam_config = {
    .pin_pwdn     = CAM_PIN_PWDN,     // -1 表示不使用
    .pin_reset    = CAM_PIN_RESET,
    .pin_xclk     = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,     // SCCB (I2C-like) SDA
    .pin_sccb_scl = CAM_PIN_SIOC,     // SCCB SCL
    .pin_d7       = CAM_PIN_D7,
    .pin_d6       = CAM_PIN_D6,
    .pin_d5       = CAM_PIN_D5,
    .pin_d4       = CAM_PIN_D4,
    .pin_d3       = CAM_PIN_D3,
    .pin_d2       = CAM_PIN_D2,
    .pin_d1       = CAM_PIN_D1,
    .pin_d0       = CAM_PIN_D0,
    .pin_vsync    = CAM_PIN_VSYNC,
    .pin_href     = CAM_PIN_HREF,
    .pin_pclk     = CAM_PIN_PCLK,

    .xclk_freq_hz    = 20000000,          // XCLK 20MHz
    .ledc_timer      = LEDC_TIMER_0,
    .ledc_channel    = LEDC_CHANNEL_0,
    .pixel_format    = PIXFORMAT_GRAYSCALE, // 灰度图 (省内存, 快处理)
    .frame_size      = FRAMESIZE_QVGA,      // 320x240
    .jpeg_quality    = 10,                  // JPEG 质量 (用灰度时不相关)
    .fb_count        = 2,                   // 双缓冲 (采集/处理并行)
    .grab_mode       = CAMERA_GRAB_WHEN_EMPTY,
};
#endif

// 上一次检测结果缓存
static cam_result_t g_last_result = {0};

// 标记亮度阈值 (0-255)
// 反光板/白色标记在灰度图中像素值 > 200
#define MARKER_BRIGHTNESS_THRESHOLD  200
// 最少需要多少个像素过阈值才认为有效
#define MIN_BRIGHT_PIXELS            20
// 焦距 (像素, 与 camera_sim.py 保持一致)
#define CAM_FX_PIX  410.0f
#define CAM_FY_PIX  410.0f
#define CAM_CX_PIX  160.0f   // 320/2
#define CAM_CY_PIX  120.0f   // 240/2


// =========================== 初始化 ==========================================
bool camera_begin() {
#ifdef HAS_ESP_CAM
    esp_err_t err = esp_camera_init(&cam_config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] 初始化失败! 错误码: 0x%x\n", err);
        return false;
    }

    // 获取传感器并微调参数
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // 增大亮度/对比度使反光板更明显
        s->set_brightness(s, 1);     // +1 亮度
        s->set_contrast(s, 2);       // +2 对比度
        s->set_saturation(s, -2);    // -2 饱和度 (用灰度, 不相关)
        // 禁用白平衡/自动曝光 → 固定参数, 标记亮度稳定
        s->set_whitebal(s, 0);
        s->set_awb_gain(s, 0);
        s->set_exposure_ctrl(s, 0);
    }

    Serial.println("[CAM] OV2640 初始化成功 (320x240 灰度)");
    return true;
#else
    Serial.println("[CAM] 非 ESP32 平台, 摄像头不可用");
    return false;
#endif
}


// =========================== 标记检测 ========================================
bool camera_detect_markers(cam_result_t *result, float altitude_m) {
    if (!result) return false;

    memset(result, 0, sizeof(cam_result_t));
    uint32_t t_start = micros();

#ifdef HAS_ESP_CAM
    // 1. 获取一帧
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        result->valid = false;
        result->proc_time_us = micros() - t_start;
        return false;
    }

    // 2. 灰度阈值 + 质心计算
    // PIXFORMAT_GRAYSCALE: 每个像素 1 字节 (0-255)
    const uint8_t *pixels = fb->buf;
    int w = fb->width;   // 320
    int h = fb->height;  // 240

    uint64_t sum_x = 0, sum_y = 0;
    uint32_t count = 0;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t val = pixels[y * w + x];
            if (val >= MARKER_BRIGHTNESS_THRESHOLD) {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }

    // 3. 释放帧缓冲
    esp_camera_fb_return(fb);

    // 4. 判定有效性
    if (count < MIN_BRIGHT_PIXELS) {
        result->valid = false;
        result->bright_pixels = count;
        result->proc_time_us = micros() - t_start;
        memcpy(&g_last_result, result, sizeof(cam_result_t));
        return false;
    }

    // 5. 计算质心
    float cx = (float)sum_x / (float)count;
    float cy = (float)sum_y / (float)count;

    result->centroid_u = cx;
    result->centroid_v = cy;
    result->du_pix = cx - CAM_CX_PIX;   // 正=右侧
    result->dv_pix = cy - CAM_CY_PIX;   // 正=下方
    result->bright_pixels = count;

    // 6. 像素 → 米 (小孔成像)
    if (altitude_m > 0.2f) {
        result->dx_m = result->du_pix * altitude_m / CAM_FX_PIX;
        result->dy_m = result->dv_pix * altitude_m / CAM_FY_PIX;
    } else {
        result->dx_m = 0.0f;
        result->dy_m = 0.0f;
    }

    // 7. 置信度: 像素越多置信度越高, 最多 5000 像素算满分
    result->confidence = (float)count / 5000.0f;
    if (result->confidence > 1.0f) result->confidence = 1.0f;
    if (result->confidence < 0.1f && count >= MIN_BRIGHT_PIXELS)
        result->confidence = 0.1f;

    result->valid = true;
    result->proc_time_us = micros() - t_start;
    memcpy(&g_last_result, result, sizeof(cam_result_t));

    return true;

#else
    result->valid = false;
    result->proc_time_us = micros() - t_start;
    return false;
#endif
}


void camera_get_last_result(cam_result_t *result) {
    if (result) memcpy(result, &g_last_result, sizeof(cam_result_t));
}


void camera_end() {
#ifdef HAS_ESP_CAM
    esp_camera_deinit();
#endif
    memset(&g_last_result, 0, sizeof(g_last_result));
}

// =========================== JPEG 帧获取 (数传回传) =============================
camera_fb_t* camera_get_jpeg() {
#ifdef HAS_ESP_CAM
    // 临时切换到 JPEG 模式获取一帧, 再切回灰度 (灰度用于标记检测)
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_pixformat(s, PIXFORMAT_JPEG);

    // 降低分辨率以减小 JPEG 体积
    if (s) s->set_framesize(s, FRAMESIZE_QQVGA);  // 160x120

    camera_fb_t *fb = esp_camera_fb_get();

    // 恢复灰度模式
    if (s) s->set_framesize(s, FRAMESIZE_QVGA);     // 320x240
    if (s) s->set_pixformat(s, PIXFORMAT_GRAYSCALE);

    return fb;
#else
    return nullptr;
#endif
}

void camera_self_test() {
    Serial.println("[CAM 自检] 开始...");

    cam_result_t res;
    for (int i = 0; i < 5; i++) {
        camera_detect_markers(&res, 10.0f);
        Serial.printf("  #%d: valid=%d bright=%d du=%.1f dv=%.1f conf=%.2f "
                      "time=%lu us\n",
                      i, res.valid, res.bright_pixels,
                      res.du_pix, res.dv_pix, res.confidence,
                      res.proc_time_us);
        delay(50);
    }
    Serial.println("[CAM 自检] 完成");
}
