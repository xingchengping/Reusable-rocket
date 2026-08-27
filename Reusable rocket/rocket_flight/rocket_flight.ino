// =============================================================================
// rocket_flight.ino — 可回收火箭飞行控制主程序 (本地 PID + 远程 LLM)
// -----------------------------------------------------------------------------
// 硬件: ESP32-S3-N16R8-CAM, Arduino-ESP32 框架
//
// 架构:
//   ESP32 端 (火箭):
//     传感器采集 → 融合 → 本地 PID (50Hz, 主控)
//     ← 接收 PC LLM 修正指令 (25% 权重融合)
//     摄像头检测 + JPEG 回传
//     → 发送遥测 (50Hz) + JPEG (1fps)
//     离散指令: 点火/阀门/泄压/开伞
//
//   PC 端 (web_control.py):
//     接收遥测 → LLM 推理 → 下发修正指令
//     Web 控制面板 + 离散指令按钮
//
// 通信: SiK/ELRS 数传电台 UART 57.6kbps
// =============================================================================

#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "fusion.h"
#include "actuators.h"
#include "sd_logger.h"
#include "state_machine.h"
#include "safety.h"
#include "camera.h"
#include "link_comms.h"
#include "pid_local.h"

// =========================== 全局变量 =========================================
static sensor_data_t   g_sensors;
static nav_state_t     g_nav;
static actuator_cmd_t  g_act_cmd;
static cam_result_t    g_cam_result;
static link_cmd_t   g_ctrl_cmd;
static pid_controller_t g_pid;
static pid_output_t     g_pid_out;
static bool            g_sensors_ok = false;
static bool            g_camera_ok  = false;
static bool            g_comms_ok   = false;
static float           g_fuel_remaining = ROCKET_FUEL_MASS_KG;

static uint32_t        g_last_control_ms = 0;
static uint32_t        g_last_sd_ms      = 0;
static uint32_t        g_last_cam_ms     = 0;
static uint32_t        g_last_jpeg_ms    = 0;
static uint32_t        g_last_telem_ms   = 0;   // v3.1: 遥测降频节流
static uint32_t        g_last_kill_check = 0;

// 急停
static bool    g_rc_kill          = false;
static uint8_t g_rc_kill_cntr     = 0;
static bool    g_rc_kill_confirmed = false;

// 摄像头 Kalman 预测 (#3)
static uint32_t g_cam_lost_ms   = 0;
static float    g_cam_pred_x    = 0.0f;
static float    g_cam_pred_y    = 0.0f;
static bool     g_cam_warning   = false;

// =========================== setup() ==========================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=============================================");
    Serial.println("  回收火箭飞控 — 本地PID+远程LLM版 v3.0");
    Serial.println("  平台: ESP32-S3-CAM + 数传电台 + PC端LLM");
    Serial.println("=============================================\n");

    // 状态灯
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_RC_KILL, INPUT_PULLUP);
    digitalWrite(PIN_LED_STATUS, LOW);

    // 传感器
    Serial.println("[BOOT] 初始化传感器...");
    g_sensors_ok = sensors_begin();
    if (!g_sensors_ok) { while(1) delay(1000); }

    sensors_calibrate_gyro();
    sensors_baro_set_zero();

    // 融合
    fusion_init(0, 0, 0);
    fusion_reset_horizontal();
    fusion_set_target(0, 0);

    // PID 控制器 (本地主控)
    Serial.println("[BOOT] 初始化 PID...");
    pid_init(&g_pid, THRUST_MAX_N, TARGET_ALTITUDE_M);
    pid_reset(&g_pid);

    // 通信 (数传电台 UART)
    Serial.println("[BOOT] 初始化数传链路...");
    link_begin();

    // 执行器
    Serial.println("[BOOT] 初始化执行器...");
    actuators_begin();

    // 摄像头
    Serial.println("[BOOT] 初始化摄像头...");
    g_camera_ok = camera_begin();
    if (g_camera_ok) camera_self_test();
    else Serial.println("[BOOT] 警告: 摄像头不可用");

    // SD 卡
    Serial.println("[BOOT] 初始化 SD 卡...");
    sd_logger_begin();

    // 状态机
    state_machine_init();

    // 看门狗 (3s 超时)
    safety_wdt_enable();

    Serial.println("\n[BOOT] 就绪, 等待 PC 连接和点火命令...");
    Serial.println("[BOOT] 指令: 'FIRE'=点火, 'ABORT'=急停, 'STATUS'=状态\n");
}

// =========================== loop() ===========================================
void loop() {
    uint32_t now = millis();
    safety_wdt_feed();

    // ── 传感器 (200Hz) ──
    g_sensors = sensors_read();
    sensor_status_t sen_status = sensors_get_status();
    g_nav = fusion_update(&g_sensors, 0.005f);

    // ── 接收 PC 指令 (50Hz, 非阻塞) ──
    if (link_recv_cmd(&g_ctrl_cmd)) {
        if (g_ctrl_cmd.flags & 0x02) {
            state_machine_set_abort();
            safety_abort_execute("PC 端急停");
        }
    }

    // ── 离散指令 (点火/阀门/泄压/降落伞) ──
    {
        link_disc_cmd_t disc;
        if (link_recv_disc_cmd(&disc)) {
            handle_disc_cmd(disc.cmd);
        }
    }

    // 串口指令回退模式 (数传不可用时的本地调试通道: FIRE/STATUS/ABORT)
    check_serial_cmds();

    // ── 控制帧 (50Hz) ──
    if (now - g_last_control_ms >= 20) {
        g_last_control_ms = now;
        flight_phase_t phase = state_machine_get_phase();
        uint32_t ft_ms = state_machine_get_flight_time_ms();

        // --- 摄像头 ---
        update_camera(now, phase);

        // --- 状态机 ---
        state_machine_update(&g_nav, ft_ms);

        // --- 急停 ---
        update_kill_switch(now);

        // --- 安全 ---
        safety_level_t sl = safety_check(&sen_status, &g_nav, g_rc_kill);
        if (sl == SAFETY_ABORT) {
            state_machine_set_abort();
            safety_abort_execute("安全检测");
        }

        // --- 离散执行器周期更新 (非阻塞点火计时) ---
        actuators_update();

        // --- 应用控制指令 ---
        apply_control(phase);

        // --- 执行 ---
        actuators_set(&g_act_cmd);

        // --- 摄像头 JPEG 回传 (0.5fps, 避免占满数传带宽; 仅飞行阶段) ---
        // v3.1: 1fps→0.5fps 且只在非点火/非急停阶段, 配合无逐块 flush 的发送
#ifdef ESP32
        if (g_camera_ok && phase >= FLIGHT_PHASE_ASCENT && phase < FLIGHT_PHASE_CAPTURE
            && now - g_last_jpeg_ms >= 2000) {
            g_last_jpeg_ms = now;
            camera_fb_t *jpg = camera_get_jpeg();
            if (jpg && jpg->buf && jpg->len > 0) {
                link_send_jpeg(jpg->buf, jpg->len);
                esp_camera_fb_return(jpg);
            }
        }
#endif

        // --- 发送遥测 (20Hz, 与主控解耦, 降低数传带宽占用) ---
        if (now - g_last_telem_ms >= (1000 / TELEMETRY_FREQ_HZ)) {
            g_last_telem_ms = now;
            send_telem(phase, sl);
        }

        // --- SD 日志 (10Hz) ---
        if (now - g_last_sd_ms >= 100) {
            g_last_sd_ms = now;
            log_flight(phase, sl);
        }
    }
}

// =========================== 控制指令应用 ======================================
// 本地 PID (每步运行) + LLM 修正 (25% 权重融合)
void apply_control(flight_phase_t phase) {
    // 控制范围: IGNITION ~ DESCENT; CAPTURE(已被筷子捕获)后立即收油停止
    if (phase >= FLIGHT_PHASE_IGNITION && phase < FLIGHT_PHASE_CAPTURE) {
        // ── 第1步: 本地 PID 计算 (内外双环, 始终运行) ──
        pid_step(&g_pid, &g_nav, g_fuel_remaining, &g_pid_out);

        // ── 第2步: LLM 修正融合 (如有新指令) ──
        float llm_thr = g_pid_out.throttle;
        float llm_pit = g_pid_out.tvc_pitch_deg;
        float llm_yaw = g_pid_out.tvc_yaw_deg;

        bool have_cmd = !link_cmd_timeout();
        if (have_cmd) {
            #define LLM_ALPHA 0.25f
            // LLM 指令: throttle[-1,1]→直接融合, tvc[-1,1]→转度再融合
            float llm_tvc_pit_deg = g_ctrl_cmd.tvc_pitch * GIMBAL_ANGLE_DEG;
            float llm_tvc_yaw_deg = g_ctrl_cmd.tvc_yaw   * GIMBAL_ANGLE_DEG;

            llm_thr = g_pid_out.throttle + LLM_ALPHA * (g_ctrl_cmd.throttle - g_pid_out.throttle);
            llm_pit = g_pid_out.tvc_pitch_deg + LLM_ALPHA * (llm_tvc_pit_deg - g_pid_out.tvc_pitch_deg);
            llm_yaw = g_pid_out.tvc_yaw_deg   + LLM_ALPHA * (llm_tvc_yaw_deg - g_pid_out.tvc_yaw_deg);
        }

        // ── 第3步: 应用 (软件限流 + PID 已在度单位) ──
        g_act_cmd.throttle        = CONSTRAIN(llm_thr, 0.0f, THROTTLE_SOFT_LIMIT);
        g_act_cmd.pitch_angle_deg = llm_pit;   // PID 已经输出度
        g_act_cmd.yaw_angle_deg   = llm_yaw;   // PID 已经输出度

        // 摄像头不可靠时降水平增益
        float h_gain = 1.0f;
        if (g_cam_lost_ms > 200) {
            h_gain = 1.0f - 0.7f * fminf(1.0f, (g_cam_lost_ms - 200) / 300.0f);
        }
        if (g_cam_warning || !g_cam_result.valid || g_cam_result.confidence < 0.2f) {
            h_gain = fminf(h_gain, 0.3f);
        }
        g_act_cmd.pitch_angle_deg *= h_gain;
        g_act_cmd.yaw_angle_deg   *= h_gain;

        // 燃料更新
        float mdot = ROCKET_FUEL_MASS_KG / BURN_TIME_S;
        g_fuel_remaining -= mdot * g_act_cmd.throttle * 0.02f;
        if (g_fuel_remaining < 0) g_fuel_remaining = 0;
    } else {
        // IDLE / CAPTURE / ABORT
        g_act_cmd.throttle        = 0.0f;
        g_act_cmd.pitch_angle_deg = 0.0f;
        g_act_cmd.yaw_angle_deg   = 0.0f;
    }

    if (phase == FLIGHT_PHASE_ABORT) {
        g_act_cmd.throttle        = ABORT_THROTTLE_IDLE;
        g_act_cmd.pitch_angle_deg = ABORT_SERVO_NEUTRAL;
        g_act_cmd.yaw_angle_deg   = ABORT_SERVO_NEUTRAL;
    }
}

// =========================== 系统自检状态 ======================================
// 起飞前自检: 生成 sys_status 位掩码 (地面站"全绿才允许点火"依据)
// 必需项: IMU + 气压计 + 执行器 + 燃料 (>20%); 外加 IDLE 状态 + 安全等级 OK
uint8_t build_sys_status(flight_phase_t phase, safety_level_t sl) {
    sensor_status_t ss = sensors_get_status();
    uint8_t st = 0;
    if (ss.imu_ok)          st |= SYS_IMU_OK;
    if (ss.baro_ok)         st |= SYS_BARO_OK;
    if (ss.tof_ok)          st |= SYS_TOF_OK;
    if (actuators_is_ok())  st |= SYS_SERVO_OK;
    if (g_fuel_remaining > ROCKET_FUEL_MASS_KG * 0.2f) st |= SYS_FUEL_OK;

    const uint8_t required = SYS_IMU_OK | SYS_BARO_OK | SYS_SERVO_OK | SYS_FUEL_OK;
    if ((st & required) == required && phase == FLIGHT_PHASE_IDLE && sl == SAFETY_OK) {
        st |= SYS_READY;
    }
    return st;
}

// =========================== 遥测发送 ==========================================
void send_telem(flight_phase_t phase, safety_level_t sl) {
    link_telem_t pkt;
    pkt.sync      = TELEM_SYNC;
    pkt.alt       = g_nav.pos_z;
    pkt.vz        = g_nav.vel_z;
    pkt.vx        = g_nav.vel_x;
    pkt.vy        = g_nav.vel_y;
    pkt.horiz_err = g_nav.horiz_range;
    pkt.roll      = g_nav.roll;
    pkt.pitch     = g_nav.pitch;
    pkt.yaw       = g_nav.yaw;
    pkt.fuel      = g_fuel_remaining / ROCKET_FUEL_MASS_KG;
    pkt.throttle  = g_act_cmd.throttle;
    pkt.phase     = (uint8_t)phase;
    pkt.safety    = (uint8_t)sl;
    pkt.cam_valid = g_cam_result.valid ? 1 : 0;
    pkt.sys_status= build_sys_status(phase, sl);   // v3.1: 起飞前自检
    pkt.cam_dx    = g_cam_result.dx_m;
    pkt.cam_dy    = g_cam_result.dy_m;
    pkt.cam_conf  = g_cam_result.confidence;
    pkt.reserved  = 0.0f;
    pkt.v_batt    = 0.0f;   // TODO: 读取实际电池电压
    pkt.crc8      = 0;

    link_send_telem(&pkt);
}

// =========================== 辅助 =============================================

void update_camera(uint32_t now, flight_phase_t phase) {
    if (!g_camera_ok || phase < FLIGHT_PHASE_ASCENT || phase >= FLIGHT_PHASE_CAPTURE) return;
    if (now - g_last_cam_ms < 50) return;
    g_last_cam_ms = now;

    camera_detect_markers(&g_cam_result, g_nav.pos_z);
    if (g_cam_result.valid && g_cam_result.confidence > 0.3f) {
        fusion_set_target(-g_cam_result.dx_m, -g_cam_result.dy_m);
        g_cam_pred_x  = -g_cam_result.dx_m;
        g_cam_pred_y  = -g_cam_result.dy_m;
        g_cam_lost_ms = 0;
        g_cam_warning  = false;
    } else {
        g_cam_lost_ms += 50;
        g_cam_pred_x += g_nav.vel_x * 0.05f;
        g_cam_pred_y += g_nav.vel_y * 0.05f;
        if (g_cam_lost_ms > 500) g_cam_warning = true;
    }
}

void update_kill_switch(uint32_t now) {
    if (now - g_last_kill_check < 20) return;
    g_last_kill_check = now;
    if (digitalRead(PIN_RC_KILL) == LOW) {
        g_rc_kill_cntr++;
        if (g_rc_kill_cntr >= 3) g_rc_kill_confirmed = true;
    } else {
        g_rc_kill_cntr = 0;
        g_rc_kill_confirmed = false;
    }
    g_rc_kill = g_rc_kill_confirmed;
}

// =========================== SD 日志 ==========================================
void log_flight(flight_phase_t phase, safety_level_t sl) {
    log_record_t rec;
    rec.timestamp_ms  = millis();
    rec.phase         = (uint8_t)phase;
    rec.alt           = g_nav.pos_z;
    rec.vz            = g_nav.vel_z;
    rec.pos_x         = g_nav.pos_x;
    rec.pos_y         = g_nav.pos_y;
    rec.vx            = g_nav.vel_x;
    rec.vy            = g_nav.vel_y;
    rec.roll          = g_nav.roll;
    rec.pitch         = g_nav.pitch;
    rec.yaw           = g_nav.yaw;
    rec.p             = g_nav.p;
    rec.q             = g_nav.q;
    rec.r             = g_nav.r;
    rec.throttle      = g_act_cmd.throttle;
    rec.fuel_remaining = g_fuel_remaining;
    rec.pitch_cmd     = g_act_cmd.pitch_angle_deg;
    rec.yaw_cmd       = g_act_cmd.yaw_angle_deg;
    rec.horiz_range   = g_nav.horiz_range;
    rec.safety        = (uint8_t)sl;
    rec.cam_valid     = g_cam_result.valid ? 1 : 0;
    rec.cam_dx_m      = g_cam_result.dx_m;
    rec.cam_dy_m      = g_cam_result.dy_m;
    rec.cam_conf      = g_cam_result.confidence;
    rec.cam_time_us   = g_cam_result.proc_time_us;

    // LLM 指令快照 (替代旧 AI 快照)
    rec.ai_input_0     = g_ctrl_cmd.throttle;
    rec.ai_input_1     = g_ctrl_cmd.tvc_pitch;
    rec.ai_horiz_err   = g_nav.horiz_range;
    rec.ai_action_thr  = g_act_cmd.throttle;
    rec.ai_action_pitch= g_act_cmd.pitch_angle_deg;
    rec.ai_action_yaw  = g_act_cmd.yaw_angle_deg;

    sd_logger_write(&rec);
}

// =========================== 离散指令处理 ======================================
void handle_disc_cmd(uint8_t cmd) {
    switch (cmd) {
        case DISC_IGNITE:
            Serial.println("[CMD] 远程点火!");
            state_machine_command_ignite();
            actuators_ignite();
            break;
        case DISC_RELIEF_OPEN:
            Serial.println("[CMD] 打开泄压阀!");
            actuators_relief_open();
            break;
        case DISC_CLOSE_ETHANOL:
            Serial.println("[CMD] 关闭乙醇阀门!");
            actuators_ethanol_valve(false);
            break;
        case DISC_CLOSE_NITRIC:
            Serial.println("[CMD] 关闭硝酸阀门!");
            actuators_nitric_valve(false);
            // 同时关泵
            {
                actuator_cmd_t stop = {0, 0, 0};
                actuators_set(&stop);
            }
            break;
        case DISC_DEPLOY_CHUTE:
            Serial.println("[CMD] 释放降落伞!");
            actuators_deploy_chute();
            // 强制关油门
            state_machine_set_abort();
            safety_abort_execute("手动开伞");
            break;
        default:
            Serial.printf("[CMD] 未知离散指令: 0x%02X\n", cmd);
    }
}

// =========================== 串口指令 (回退模式) ===============================
void check_serial_cmds() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "FIRE" || cmd == "IGNITE") {
        Serial.println("[CMD] 点火!");
        state_machine_command_ignite();
        actuators_ignite();   // v3.1: 补上与离散指令一致的点火动作 (非阻塞)
    }
    else if (cmd == "STATUS") {
        Serial.println("=== 状态 ===");
        Serial.printf("WiFi: %s\n", g_comms_ok ? "已连接" : "未连接");
        Serial.printf("高度: %.2fm  燃料: %.0f%%\n",
                      g_nav.pos_z, g_fuel_remaining * 100 / ROCKET_FUEL_MASS_KG);
        Serial.printf("油门: %.0f%%  TVC: p=%.0f° y=%.0f°\n",
                      g_act_cmd.throttle * 100,
                      g_act_cmd.pitch_angle_deg, g_act_cmd.yaw_angle_deg);
    }
    else if (cmd == "ABORT") {
        Serial.println("[CMD] 急停!");
        state_machine_set_abort();
        safety_abort_execute("串口急停");
    }
}
