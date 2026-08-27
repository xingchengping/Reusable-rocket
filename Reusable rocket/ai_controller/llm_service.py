"""
LLM 火箭着陆控制器
-------------------
通过系统提示词让 GGUF 量化模型直接输出控制指令。
推理引擎: llama-cpp-python (llama.cpp 的 Python 绑定)

用法:
  python llm_service.py --model-path ./models/gemma-2-2b-it.Q4_K_M.gguf
  python llm_service.py --model-path ./models/qwen2.5-1.5b-instruct-q4.gguf --n-gpu-layers 99
"""
from __future__ import annotations
import json, re, time, argparse, os
from typing import Optional

# =========================== 系统提示词 =========================================
# v3.1: 精简冗余表述, 物理参数与 config.h 对齐 (TVC力臂 0.5m, 推重比 1.98)
SYSTEM_PROMPT = """你是火箭着陆控制器。根据遥测数据输出油门和TVC指令，只输出JSON。

## 火箭物理限制 (v3: 120mm箭体, 1.80kg起飞, 35N推力)
- 干重 1.65kg, 燃料 0.15kg, 起飞总重 ≈ 1.80kg
- 最大推力 35N (65N发动机 × 54% 软件限流), 推重比 ≈ 1.98
- 悬停油门 ≈ 0.50~0.53 (随燃料消耗略微下降)
- 转动惯量 Iyy=0.200 kg·m², TVC力臂 0.5m, TVC最大偏转 ±12°
- 油门死区: <0.15 无推力; 响应延迟: 油门100ms, 舵机75ms
- 着陆精度要求: 水平误差 <5cm, 垂直速度 <1m/s

## 输入格式
{"alt": 45.2, "vz": -3.1, "vh": 1.2, "horiz_err": 0.35, "fuel": 0.65, "phase": 2}
- alt: 高度(m); vz: 垂直速度(m/s), 负=下降; vh: 水平合速度(m/s)
- horiz_err: 距筷子架水平距离(m); fuel: 剩余燃料比例(0~1); phase: 1点火 2爬升 3悬停 4下降

## 输出格式 (纯JSON, 无其他文字)
{"throttle": 0.6, "tvc_pitch": -0.1, "tvc_yaw": 0.05}
- throttle: 0~1 (0.15以下无推力); tvc_pitch: -1~1 (负=向前倾斜); tvc_yaw: -1~1 (负=向左)

## 控制策略 (全部用软过渡, 避免跳跃)
1. 爬升 (alt < 45m): 油门 0.70~0.95, TVC保持垂直
2. 接近目标 (alt 45~52m): 逐步收油至 0.50~0.60, 用TVC修正水平误差
3. 悬停/下降: 油门 0.40~0.55, TVC主动将水平误差推向0
4. 着陆前 (alt < 2m): 油门 0.38~0.52缓冲, 不归零, TVC继续修正

## TVC方向规则
- horiz_err为正(偏右) → tvc_yaw为负(向左推); 反之亦然
- 速度同理, 按相反方向修正

## 硬性约束 (必须遵守)
- 油门连续变化: 每帧不超过 0.20; TVC连续变化: 每帧不超过 0.12
- 垂直下降速度: 任何时候不超过 2.5m/s
- 燃料 < 8% 时: 优先安全着陆, 放弃精度
- 姿态角绝对值不能超过 15°, 否则TVC全力修正姿态

只输出JSON, 不要解释。"""
# =============================================================================


class LLMController:
    """LLM 推理封装: 发状态, 收指令。基于 llama-cpp-python"""

    def __init__(self, model_path: str, n_gpu_layers: int = 0,
                 n_ctx: int = 512, verbose: bool = False):
        self.model_path = model_path
        self.n_gpu_layers = n_gpu_layers
        self.n_ctx = n_ctx
        self.verbose = verbose
        self._llm = None
        self._inference_times = []
        self._prev_throttle = 0.5
        self._prev_pitch = 0.0
        self._prev_yaw = 0.0

    def load(self):
        from llama_cpp import Llama

        # 同目录自动检测 .gguf 文件
        if not os.path.isabs(self.model_path):
            local = os.path.join(os.path.dirname(__file__), self.model_path)
            if os.path.exists(local):
                self.model_path = local
        if not os.path.exists(self.model_path):
            # 搜索同目录下的 .gguf 文件
            d = os.path.dirname(__file__)
            ggufs = [f for f in os.listdir(d) if f.endswith(".gguf")]
            if ggufs:
                self.model_path = os.path.join(d, ggufs[0])
                print(f"  [LLM] 自动检测到模型: {self.model_path}")
            else:
                raise FileNotFoundError(
                    f"模型文件不存在: {self.model_path}\n"
                    f"请将 .gguf 模型放在 {d}/ 目录下"
                )

        print(f"  [LLM] 加载模型: {self.model_path}")
        print(f"  [LLM] GPU 层数: {self.n_gpu_layers} (0=纯CPU, -1=全部GPU)")
        print(f"  [LLM] 上下文: {self.n_ctx} tokens")

        self._llm = Llama(
            model_path=self.model_path,
            n_gpu_layers=self.n_gpu_layers,
            n_ctx=self.n_ctx,
            verbose=self.verbose,
        )
        print(f"  [LLM] 加载完成")

    def step(self, state: dict, prev_action: Optional[dict] = None) -> dict:
        """输入火箭状态, 输出控制指令 JSON"""
        t0 = time.perf_counter()

        # 构建用户消息
        user_msg = f"当前状态:\n{json.dumps(state, ensure_ascii=False)}"
        if prev_action:
            user_msg += f"\n上一帧动作: {json.dumps(prev_action)}"

        result = self._call_llama(user_msg)

        # 提取 JSON
        action = self._parse_action(result)

        # 平滑
        action = self._smooth(action)

        dt = time.perf_counter() - t0
        self._inference_times.append(dt * 1000)

        self._prev_throttle = action["throttle"]
        self._prev_pitch = action["tvc_pitch"]
        self._prev_yaw = action["tvc_yaw"]

        return action

    def _call_llama(self, user_msg: str) -> str:
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_msg},
        ]
        output = self._llm.create_chat_completion(
            messages=messages,
            temperature=0.1,
            max_tokens=80,
            top_p=0.9,
        )
        return output["choices"][0]["message"]["content"]

    def _parse_action(self, text: str) -> dict:
        """从 LLM 输出中提取 JSON 指令"""
        for pattern in [r'\{[^{}]*"throttle"[^{}]*\}',
                         r'\{[^{}]*\}',
                         r'\{.+\}']:
            m = re.search(pattern, text, re.DOTALL)
            if m:
                try:
                    obj = json.loads(m.group(0))
                    if "throttle" in obj:
                        return {
                            "throttle": float(obj.get("throttle", 0.5)),
                            "tvc_pitch": float(obj.get("tvc_pitch", 0.0)),
                            "tvc_yaw": float(obj.get("tvc_yaw", 0.0)),
                        }
                except json.JSONDecodeError:
                    continue
        # 解析失败: 维持当前
        print(f"  [LLM] 解析失败, 保持当前. 输出: {text[:200]}")
        return {"throttle": self._prev_throttle, "tvc_pitch": 0.0, "tvc_yaw": 0.0}

    def _smooth(self, action: dict) -> dict:
        """动作平滑: 限制帧间变化率"""
        MAX_DTHR = 0.20   # 匹配提示词约束
        MAX_DTVC = 0.12   # 匹配提示词约束
        action["throttle"] = self._prev_throttle + max(-MAX_DTHR,
            min(MAX_DTHR, action["throttle"] - self._prev_throttle))
        action["tvc_pitch"] = self._prev_pitch + max(-MAX_DTVC,
            min(MAX_DTVC, action["tvc_pitch"] - self._prev_pitch))
        action["tvc_yaw"] = self._prev_yaw + max(-MAX_DTVC,
            min(MAX_DTVC, action["tvc_yaw"] - self._prev_yaw))
        action["throttle"] = max(0.0, min(1.0, action["throttle"]))
        action["tvc_pitch"] = max(-1.0, min(1.0, action["tvc_pitch"]))
        action["tvc_yaw"] = max(-1.0, min(1.0, action["tvc_yaw"]))
        return action

    @property
    def avg_inference_ms(self) -> float:
        if not self._inference_times:
            return 0.0
        return sum(self._inference_times) / len(self._inference_times)

    def stats(self) -> dict:
        return {
            "avg_inference_ms": round(self.avg_inference_ms, 1),
            "total_calls": len(self._inference_times),
            "model": self.model_path,
            "gpu_layers": self.n_gpu_layers,
        }

    def unload(self):
        if self._llm:
            del self._llm
            self._llm = None


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-path", required=True, help="GGUF 模型文件路径")
    parser.add_argument("--n-gpu-layers", type=int, default=0,
                        help="GPU 层数 (0=CPU, -1=全部GPU, 建议 99)")
    parser.add_argument("--n-ctx", type=int, default=512)
    args = parser.parse_args()

    ctrl = LLMController(args.model_path, args.n_gpu_layers, args.n_ctx)
    ctrl.load()
    print("LLM 控制器已就绪. 测试推理...\n")

    test_state = {"alt": 45.0, "vz": -2.5, "vh": 0.8, "horiz_err": 0.3,
                   "fuel": 0.60, "phase": "descending"}
    action = ctrl.step(test_state)
    print(f"状态: {json.dumps(test_state)}")
    print(f"指令: {json.dumps(action)}")
    print(f"推理耗时: {ctrl.avg_inference_ms:.0f}ms")
