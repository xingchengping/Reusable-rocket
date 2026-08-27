// =============================================================================
// camera.h — 箭载下视摄像头驱动 + 筷子架标记检测
// -----------------------------------------------------------------------------
// 硬件: ESP32-S3-CAM + OV2640 (或 OV5640)
// 库:   esp32-camera (Arduino)
//
// 功能:
//   1. 初始化 OV2640 摄像头 (JPEG + RGB565 双模式)
//   2. 采集一帧并转为灰度图 (高效阈值处理)
//   3. 检测地面 4 个高对比度标记点 (亮度阈值 + 质心)
//   4. 输出筷子架中心的像素偏移、实际水平偏移(m)、置信度
//
// 标记检测算法 (极简但有效, 适合 ESP32-S3 @240MHz):
//   1. 灰度化 + 固定阈值 (200/255, 标记为白色反光板)
//   2. 统计所有过阈值像素的 x/y 总和及数量
//   3. 质心 = (sum_x/N, sum_y/N)
//   4. 像素偏移 = 质心 - 图像中心
//   5. 实际偏移 = 像素偏移 * (高度 / 焦距)
//
// 性能:
//   - QVGA(320x240)灰度阈值+质心: <2ms
//   - 可每 5 帧处理一次 (30fps → 6Hz 检测率), 满足 50Hz 控制
//
// 用法:
//   #include "camera.h"
//   camera_begin();
//   cam_result_t res;
//   if (camera_detect_markers(&res, altitude_m)) {
//     // res.dx_m, res.dy_m 为水平偏移, res.confidence 为置信度
//   }
// =============================================================================
#ifndef ROCKET_CAMERA_H
#define ROCKET_CAMERA_H

#include "config.h"
#include <stdint.h>

#ifdef ESP32
  #include "esp_camera.h"   // camera_fb_t 类型定义
#endif

// 检测结果
typedef struct {
    bool    valid;           // 检测有效 (找到足够标记)
    float   centroid_u;      // 质心像素 X (图像坐标)
    float   centroid_v;      // 质心像素 Y
    float   du_pix;          // 像素偏移 X (质心 - 中心)
    float   dv_pix;          // 像素偏移 Y
    float   dx_m;            // 水平偏移 X (米)
    float   dy_m;            // 水平偏移 Y (米)
    float   confidence;      // 置信度 [0, 1]
    int     bright_pixels;   // 过阈值的像素数
    uint32_t proc_time_us;   // 本帧处理耗时 (微秒)
} cam_result_t;

// 初始化摄像头
// 返回 true 成功
bool camera_begin();

// 采集一帧并检测标记
// altitude_m: 当前高度 (米), 用于像素→米换算
// 返回 true 表示检测到有效标记
bool camera_detect_markers(cam_result_t *result, float altitude_m);

// 获取最近一帧的检测结果 (不重新采集)
void camera_get_last_result(cam_result_t *result);

// 关闭摄像头 (省电)
void camera_end();

// 获取 JPEG 帧 (用于回传地面)
// 返回 camera_fb_t 指针, 调用者负责 esp_camera_fb_return()
// 注: 仅在 ESP32 平台可用 (camera_fb_t 类型由 esp_camera.h 提供)
#ifdef ESP32
camera_fb_t* camera_get_jpeg();
#endif

// 自检: 采集一帧并输出统计 (调试用)
void camera_self_test();

#endif // ROCKET_CAMERA_H
