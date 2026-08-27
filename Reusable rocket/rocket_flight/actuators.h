// =============================================================================
// actuators.h — 执行器驱动 (舵机 PWM + 硝酸泵 + 离散执行器)
// =============================================================================
#ifndef ROCKET_ACTUATORS_H
#define ROCKET_ACTUATORS_H

#include "config.h"

// 舵机指令结构
typedef struct {
    float pitch_angle_deg;   // 俯仰舵角 (度, -12~+12)
    float yaw_angle_deg;     // 偏航舵角 (度, -12~+12)
    float throttle;          // 油门 (0.0~1.0)
} actuator_cmd_t;

// 连续执行器
void actuators_begin();
void actuators_set(const actuator_cmd_t *cmd);
void actuators_emergency_stop();
void actuators_send_servo_us(uint16_t pitch_us, uint16_t yaw_us);
void actuators_get_state(float *pitch_deg, float *yaw_deg, float *throttle);
bool actuators_self_test();

// 周期更新 (非阻塞点火计时, 在主循环每帧调用)
void actuators_update();

// 执行器初始化是否成功 (起飞前自检用)
bool actuators_is_ok();

// 离散执行器 (一键操作)
void actuators_ignite();            // 点火 (非阻塞: 通电 2s 后自动断电)
void actuators_relief_open();       // 打开燃烧室泄压阀
void actuators_relief_close();      // 关闭泄压阀
void actuators_ethanol_valve(bool open);   // 乙醇阀门
void actuators_nitric_valve(bool open);    // 硝酸阀门
void actuators_deploy_chute();      // 释放降落伞

// 查询离散执行器状态
bool actuators_is_ignited();
bool actuators_is_chute_deployed();
bool actuators_is_ethanol_open();
bool actuators_is_nitric_open();

#endif // ROCKET_ACTUATORS_H
