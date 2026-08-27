"""
PID 备援控制器 — 地面 AI (LLM) 的双保险
----------------------------------------
当 LLM 推理超时 (>50ms)、推理失败、或通信中断时，
PID 立即接管火箭控制，确保安全着陆/悬停。

控制策略:
  - 高度 PID → 油门 (throttle)
  - 水平位置 PID → TVC 俯仰/偏航

用法:
  pid = PIDController(thrust_max=22.1, target_alt=50.0)
  action = pid.step(state)  # {"throttle": ..., "tvc_pitch": ..., "tvc_yaw": ...}
"""
from __future__ import annotations
import math
from typing import Optional


class PID:
    """单轴 PID 控制器 (带积分限幅 + 微分滤波)"""
    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = -1.0, out_max: float = 1.0,
                 i_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_max = i_max
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_deriv = 0.0

    def step(self, error: float, dt: float) -> float:
        # 比例
        p = self.kp * error

        # 积分 (带限幅, 防饱和)
        self._integral += error * dt
        self._integral = max(-self.i_max, min(self.i_max, self._integral))
        i = self.ki * self._integral

        # 微分 (带一阶低通滤波)
        if dt > 1e-9:
            raw_deriv = (error - self._prev_error) / dt
            alpha = 0.7  # 滤波系数
            deriv = alpha * raw_deriv + (1.0 - alpha) * self._prev_deriv
            self._prev_deriv = deriv
        else:
            deriv = 0.0
        d = self.kd * deriv

        self._prev_error = error

        out = p + i + d
        return max(self.out_min, min(self.out_max, out))


class PIDController:
    """火箭着陆 PID 备援控制器

    变量说明:
      - alt: 当前高度 (m)
      - vz: 垂直速度 (m/s), 负值=下降
      - vx, vy: 水平速度 (m/s)
      - pos_x, pos_y: 水平位置 (m), 相对于回收点
      - fuel: 剩余燃料比例 [0, 1]
      - phase: 飞行阶段 (str)
    """

    # 35N 推力下火箭参数 (120mm箭体, 1.8kg起飞):
    #   推力/重力比 = 35 / (1.8 * 9.81) ≈ 1.98
    #   悬停油门 ≈ mg/T_max ≈ 1.8*9.81 / 35 ≈ 0.505
    #   重力补偿由 ESP32 本地 pid_local.cpp 处理, 此文件仅用于仿真测试
    HOVER_THROTTLE = 0.52   # 抵消重力所需油门 (含效率损失)

    def __init__(self, thrust_max: float = 35.0, target_alt: float = 50.0,
                 dt: float = 0.02):
        self.thrust_max = thrust_max
        self.target_alt = target_alt
        self.dt = dt

        # 高度 → 油门 PID (核心: 先飞上去, 再慢慢降下来)
        self.pid_alt = PID(kp=0.06, ki=0.008, kd=0.02,
                           out_min=-0.5, out_max=0.5,
                           i_max=0.3)

        # 垂直速度 → 油门修正 PID (阻尼)
        self.pid_vz = PID(kp=0.15, ki=0.02, kd=0.03,
                          out_min=-0.25, out_max=0.25,
                          i_max=0.15)

        # 水平位置 X → pitch PID
        self.pid_x = PID(kp=0.12, ki=0.01, kd=0.06,
                         out_min=-1.0, out_max=1.0,
                         i_max=0.5)

        # 水平位置 Y → yaw PID
        self.pid_y = PID(kp=0.12, ki=0.01, kd=0.06,
                         out_min=-1.0, out_max=1.0,
                         i_max=0.5)

        self._phase_initialized = False

    def reset(self):
        for pid in [self.pid_alt, self.pid_vz, self.pid_x, self.pid_y]:
            pid.reset()
        self._phase_initialized = False

    def step(self, state: dict, prev_action: Optional[dict] = None) -> dict:
        """输入火箭状态 (与 LLM 同格式), 输出控制指令"""
        alt = float(state.get("alt", 0))
        vz = float(state.get("vz", 0))
        vh = float(state.get("vh", 0))
        horiz_err = float(state.get("horiz_err", 0))
        fuel = float(state.get("fuel", 1.0))
        phase = state.get("phase", "ascending")

        # --- 从水平误差反推 x/y ---
        pos_x = float(state.get("pos_x", horiz_err))
        pos_y = float(state.get("pos_y", 0.0))

        dt = self.dt

        # ============================
        # 高度/油门控制
        # ============================
        # 目标: 飞到目标高度, 然后平稳下降着陆
        if phase in ("ascending", "ignition"):
            # 上升阶段: 全力飞向目标高度
            alt_error = self.target_alt - alt
            if alt < self.target_alt * 0.5:
                # 低空: 大油门爬升
                throttle = 0.85
            else:
                # 接近目标: PID 精确控高
                throttle = self.HOVER_THROTTLE + self.pid_alt.step(alt_error, dt)
        elif phase == "hovering":
            # 悬停: 维持高度
            alt_error = self.target_alt - alt
            throttle = self.HOVER_THROTTLE + self.pid_alt.step(alt_error, dt)
        elif phase in ("descending", "landing"):
            # 下降: 逐渐减小油门, 控制下降速度
            if alt < 5.0:
                # 近地: 缓冲降落
                target_vz = -0.8 * (alt / 5.0)  # alt=5m→vz=-0.8, alt=0→vz=0
            elif alt < 15.0:
                target_vz = -1.5
            else:
                target_vz = -2.5

            vz_error = target_vz - vz
            throttle = self.HOVER_THROTTLE + self.pid_vz.step(vz_error, dt)
        else:
            throttle = self.HOVER_THROTTLE

        # 燃料保护: 低于 5% 时优先着陆
        if fuel < 0.05 and phase != "ascending":
            if alt < 3.0:
                throttle = 0.35  # 缓冲
            else:
                throttle = max(0.15, min(throttle, 0.3))  # 缓慢下降

        # 硬限幅
        throttle = max(0.0, min(1.0, throttle))

        # ============================
        # 水平位置 → TVC 控制
        # ============================
        # TVC 方向: 正误差 → 负推力 (往回收点推)
        pitch_cmd = -self.pid_x.step(pos_x, dt)
        yaw_cmd   = -self.pid_y.step(pos_y, dt)

        # 近地时降低水平增益 (避免翻倒)
        if alt < 3.0:
            gain = alt / 3.0
            pitch_cmd *= gain
            yaw_cmd *= gain

        return {
            "throttle": round(throttle, 3),
            "tvc_pitch": round(pitch_cmd, 3),
            "tvc_yaw": round(yaw_cmd, 3),
        }
