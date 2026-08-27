"""
地面控制站 — 一站式 (Web面板 + LLM控制 + 筷子架 + 离散指令 + 视频)
------------------------------------------------------------------
发射时只跑这一个脚本即可。

架构:
  火箭 ESP32 ←UART→ 数传模块 ←RF→ 数传模块 ←USB→ PC (COM3)
  筷子架 ESP32 ←UART──────────────────────←USB→ PC (COM4)
                                    │
                         ┌──────────┴──────────┐
                         │  web_control.py     │
                         │  ├─ 火箭串口线程    │ ← 遥测+LLM 10Hz
                         │  ├─ 筷子架串口线程  │ ← 传感器+FSM 10Hz
                         │  ├─ Flask HTTP      │ ← 面板/API/视频流
                         │  └─ 指令发送        │ ← 离散 + 控制
                         └─────────────────────┘

用法:
  python web_control.py --port COM3 --model-path ./model.gguf --http-port 8080
  python web_control.py --port COM3 --chop-port COM4  (无 LLM 模式)
"""
from __future__ import annotations
import sys, os, json, time, struct, threading, argparse, math
from collections import deque
from typing import Optional

import serial
import serial.tools.list_ports
from flask import Flask, Response, render_template, jsonify, request

sys.path.insert(0, os.path.dirname(__file__))
from llm_service import LLMController

# ======================== 协议常量 =============================================
DISC_SYNC         = 0xEF
DISC_IGNITE       = 0x01
DISC_RELIEF_OPEN  = 0x02
DISC_CLOSE_ETHANOL= 0x03
DISC_CLOSE_NITRIC = 0x04
DISC_DEPLOY_CHUTE = 0x05

# 控制指令 flags (0xAB 包): b0=reset, b1=abort
FLAG_ABORT = 0x02

# 遥测 (火箭→PC, 66B): sync(1) + 14f(56) + 4B(4) + f(4) + crc(1) = 66
TELEM_SYNC   = 0xCD
TELEM_SIZE   = 66
TELEM_FMT    = "<B 14f 4B f B"

# 系统自检状态位 (对应火箭端 link_comms.h 的 SYS_* 定义)
SYS_IMU_OK   = 0x01
SYS_BARO_OK  = 0x02
SYS_TOF_OK   = 0x04
SYS_SERVO_OK = 0x08
SYS_FUEL_OK  = 0x10
SYS_READY    = 0x80

# 控制指令 (PC→火箭, 15B): sync(1) + 3f(12) + B(1) + crc(1) = 15
CMD_SYNC     = 0xAB
CMD_FMT      = "<B 3f B B"
CMD_SIZE     = 15

JPEG_SYNC      = 0xFC
JPEG_MAX_CHUNK = 220

PHASE_NAMES = {0:"idle",1:"ignition",2:"ascending",3:"hovering",
               4:"descending",5:"capture",6:"abort"}

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        for _ in range(8):
            if (crc ^ b) & 1: crc = (crc >> 1) ^ 0x8C
            else:             crc >>= 1
            b >>= 1
    return crc & 0xFF

# ======================== 全局状态 =============================================
app = Flask(__name__)
ser: serial.Serial | None = None
ser_lock = threading.Lock()

# 最新遥测
latest_telem: dict = {}
telem_lock = threading.Lock()

# LLM 实例
llm_ctrl: Optional[LLMController] = None

# 统计
ctrl_stats = {
    "cmd_tx": 0, "telem_rx": 0, "crc_err": 0,
    "llm_calls": 0, "llm_fails": 0,
}

# ======================== 筷子架 (只发一键打开) =================================
chop_ser: serial.Serial | None = None  # 只写不读, 用于一键打开

# JPEG 帧缓存
class JpegAssembler:
    def __init__(self):
        self._chunks: dict = {}
        self._latest_frame = b""
        self._frame_lock = threading.Lock()
        self._max_frames = 5

    def add_chunk(self, seq, chunk_idx, total_chunks, jpeg_len, data):
        key = seq
        if key not in self._chunks:
            self._chunks[key] = {"total_chunks":total_chunks, "jpeg_len":jpeg_len, "chunks":{}}
            if len(self._chunks) > self._max_frames:
                del self._chunks[min(self._chunks.keys())]
        self._chunks[key]["chunks"][chunk_idx] = data
        info = self._chunks[key]
        if len(info["chunks"]) == info["total_chunks"]:
            full = bytearray()
            for i in range(info["total_chunks"]):
                full.extend(info["chunks"][i])
            jpg = bytes(full[:info["jpeg_len"]])
            if jpg[:2] == b'\xff\xd8' and jpg[-2:] == b'\xff\xd9':
                with self._frame_lock:
                    self._latest_frame = jpg
            del self._chunks[key]

    def get_latest(self) -> bytes:
        with self._frame_lock:
            return self._latest_frame

jpeg_asm = JpegAssembler()

# ======================== 串口 + 控制线程 =======================================
RECONNECT_INTERVAL_S = 2.0   # 断线重连间隔

def control_thread(port: str, baudrate: int, model_path: str, n_gpu_layers: int):
    global ser, llm_ctrl

    # ── 加载 LLM (仅一次) ──
    if model_path:
        print(f"[INIT] 加载 LLM 模型: {model_path}")
        llm_ctrl = LLMController(model_path, n_gpu_layers)
        llm_ctrl.load()
    else:
        llm_ctrl = None
        print("[INIT] 无 LLM 模型, 仅做监控面板")

    last_llm_ms = 0
    LLM_INTERVAL_MS = 100  # 10Hz

    # ── 连接循环: 断线后自动重连 (v3.1 新增) ──
    while True:
        # 打开串口
        try:
            ser = serial.Serial(port, baudrate, timeout=0.01)
            print(f"[LINK] {port} @ {baudrate} 已连接")
            print(f"[CTRL] 地面 LLM 推理 (火箭端本地 PID 主控, LLM 10Hz 修正)")
        except Exception as e:
            print(f"[ERROR] 串口打开失败: {e}")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device} - {p.description}")
            time.sleep(RECONNECT_INTERVAL_S)
            continue

        buf = bytearray()
        try:
            while ser and ser.is_open:
                t0 = time.perf_counter()

                # ── 1. 读取串口 ──
                try:
                    if ser.in_waiting:
                        with ser_lock:
                            buf.extend(ser.read(ser.in_waiting))
                    buf = _parse_buf(buf)
                    if len(buf) > 8192:
                        buf = buf[-4096:]
                except (serial.SerialException, OSError):
                    break

                # ── 2. LLM 推理 (10Hz) ──
                if llm_ctrl is not None:
                    with telem_lock:
                        state = dict(latest_telem)

                    if state and state.get("alt", -1) >= 0:
                        now_ms = int(time.perf_counter() * 1000)
                        if now_ms - last_llm_ms >= LLM_INTERVAL_MS:
                            try:
                                llm_action = llm_ctrl.step(state)
                                _send_ctrl_cmd(llm_action)
                                ctrl_stats["llm_calls"] += 1
                            except Exception as e:
                                ctrl_stats["llm_fails"] += 1
                                # LLM 失败: 火箭端 PID 完全自主控制, 不发送任何指令
                            last_llm_ms = now_ms

                # ── 3. 保持 50Hz ──
                elapsed = time.perf_counter() - t0
                sleep_t = max(0, 0.019 - elapsed)
                time.sleep(sleep_t)
        except (serial.SerialException, OSError):
            pass

        print("[LINK] 串口断开, 重连中...")
        try:
            if ser: ser.close()
        except Exception:
            pass
        ser = None
        time.sleep(RECONNECT_INTERVAL_S)


def _parse_buf(buf_in: bytearray) -> bytearray:
    """解析遥测和 JPEG 帧"""
    global latest_telem

    while len(buf_in) >= 3:
        sync = buf_in[0]

        if sync == TELEM_SYNC and len(buf_in) >= TELEM_SIZE:
            pkt = buf_in[:TELEM_SIZE]
            if crc8(pkt[:-1]) == pkt[-1]:
                buf_in = buf_in[TELEM_SIZE:]
                try:
                    vals = struct.unpack(TELEM_FMT, pkt)
                except struct.error:
                    continue
                with telem_lock:
                    vx = float(vals[3])
                    vy = float(vals[4])
                    vh = math.sqrt(vx*vx + vy*vy)
                    latest_telem = {
                        "alt":       round(float(vals[1]), 2),
                        "vz":        round(float(vals[2]), 2),
                        "vx":        round(vx, 2),
                        "vy":        round(vy, 2),
                        "vh":        round(vh, 2),
                        "horiz_err": round(float(vals[5]), 3),
                        "roll":      round(float(vals[6]), 1),
                        "pitch":     round(float(vals[7]), 1),
                        "yaw":       round(float(vals[8]), 1),
                        "fuel":      round(float(vals[9]), 3),
                        "throttle":  round(float(vals[10]), 2),
                        "phase":     int(vals[15]),
                        "safety":    int(vals[16]),
                        "cam_valid": int(vals[17]) > 0,
                        "sys_status": int(vals[18]),   # v3.1: 起飞前自检位
                        "v_batt":    round(float(vals[19]), 2),
                    }
                ctrl_stats["telem_rx"] += 1
            else:
                buf_in = buf_in[1:]
                ctrl_stats["crc_err"] += 1
            continue

        elif sync == JPEG_SYNC and len(buf_in) >= 8:
            seq  = buf_in[1] | (buf_in[2] << 8)
            idx  = buf_in[3]
            total= buf_in[4]
            jlen = buf_in[5] | (buf_in[6] << 8)
            chunk_data_len = (jlen - idx * JPEG_MAX_CHUNK) if idx == total - 1 else JPEG_MAX_CHUNK
            pkt_total = 7 + chunk_data_len + 1
            if len(buf_in) < pkt_total:
                break
            pkt = buf_in[:pkt_total]
            if crc8(pkt[:-1]) == pkt[-1]:
                jpeg_asm.add_chunk(seq, idx, total, jlen, bytes(pkt[7:7+chunk_data_len]))
            buf_in = buf_in[pkt_total:]
            continue

        else:
            buf_in = buf_in[1:]

    return buf_in


def _send_ctrl_cmd(action: dict, flags: int = 0):
    """发送 0xAB 控制指令 (油门+TVC), flags: b0=reset b1=abort"""
    global ser
    if not ser or not ser.is_open:
        return
    raw = struct.pack(CMD_FMT, CMD_SYNC,
                      float(action["throttle"]),
                      float(action["tvc_pitch"]),
                      float(action["tvc_yaw"]),
                      flags, 0)
    pkt = raw[:-1] + bytes([crc8(raw[:-1])])
    try:
        with ser_lock:
            ser.write(pkt)
        ctrl_stats["cmd_tx"] += 1
    except (serial.SerialException, OSError):
        pass


def _send_abort() -> bool:
    """发送带 abort flag 的 0xAB 控制指令 (火箭端 flags&0x02 → 立即急停)"""
    global ser
    if not ser or not ser.is_open:
        return False
    action = {"throttle": 0.0, "tvc_pitch": 0.0, "tvc_yaw": 0.0}
    _send_ctrl_cmd(action, flags=FLAG_ABORT)
    return True


def _send_disc_cmd(cmd: int) -> bool:
    """发送 0xEF 离散指令 (点火/阀门/泄压/开伞)"""
    global ser
    if not ser or not ser.is_open:
        return False
    raw = bytes([DISC_SYNC, cmd])
    pkt = raw + bytes([crc8(raw)])
    try:
        with ser_lock:
            ser.write(pkt)
        return True
    except (serial.SerialException, OSError):
        return False


# ======================== 筷子架一键打开 ========================================
def _chop_open():
    """发送 0xFF 到筷子架串口 = 手动打开"""
    global chop_ser
    if not chop_ser or not chop_ser.is_open:
        return False
    try:
        chop_ser.write(b'\xFF')
        return True
    except (serial.SerialException, OSError):
        return False

def _chop_close():
    """发送 0xFE 到筷子架串口 = 手动关闭"""
    global chop_ser
    if not chop_ser or not chop_ser.is_open:
        return False
    try:
        chop_ser.write(b'\xFE')
        return True
    except (serial.SerialException, OSError):
        return False


# ======================== Flask 路由 ===========================================
@app.route("/")
def index():
    return render_template("control_panel.html")

@app.route("/api/status")
def api_status():
    with telem_lock:
        telem = dict(latest_telem)
    llm_stats = llm_ctrl.stats() if llm_ctrl else {}

    # 起飞前自检清单 (从 sys_status 位解析)
    st = telem.get("sys_status", 0)
    health = {
        "imu":   bool(st & SYS_IMU_OK),
        "baro":  bool(st & SYS_BARO_OK),
        "tof":   bool(st & SYS_TOF_OK),
        "servo": bool(st & SYS_SERVO_OK),
        "fuel":  bool(st & SYS_FUEL_OK),
        "ready": bool(st & SYS_READY),
    }

    return jsonify({
        "telem": telem,
        "health": health,
        "control": {
            "cmd_tx": ctrl_stats["cmd_tx"],
            "telem_rx": ctrl_stats["telem_rx"],
            "llm_calls": ctrl_stats["llm_calls"],
            "llm_fails": ctrl_stats["llm_fails"],
            "llm_avg_ms": llm_stats.get("avg_inference_ms", 0),
        }
    })

@app.route("/api/ignite", methods=["POST"])
def api_ignite():
    ok = _send_disc_cmd(DISC_IGNITE)
    return jsonify({"ok": ok, "action": "ignite"})

@app.route("/api/relief", methods=["POST"])
def api_relief():
    ok = _send_disc_cmd(DISC_RELIEF_OPEN)
    return jsonify({"ok": ok, "action": "relief_open"})

@app.route("/api/close_ethanol", methods=["POST"])
def api_close_ethanol():
    ok = _send_disc_cmd(DISC_CLOSE_ETHANOL)
    return jsonify({"ok": ok, "action": "close_ethanol"})

@app.route("/api/close_nitric", methods=["POST"])
def api_close_nitric():
    ok = _send_disc_cmd(DISC_CLOSE_NITRIC)
    return jsonify({"ok": ok, "action": "close_nitric"})

@app.route("/api/deploy_chute", methods=["POST"])
def api_deploy_chute():
    ok = _send_disc_cmd(DISC_DEPLOY_CHUTE)
    return jsonify({"ok": ok, "action": "deploy_chute"})

# ── 急停 (ABORT): 通过 0xAB flags b1 通知火箭端立即终止 (v3.1 新增) ──
@app.route("/api/abort", methods=["POST"])
def api_abort():
    ok = _send_abort()
    return jsonify({"ok": ok, "action": "abort"})

# ── 筷子架: 手动控制 ──
@app.route("/api/chop_open", methods=["POST"])
def api_chop_open():
    ok = _chop_open()
    return jsonify({"ok": ok, "action": "chop_open"})

@app.route("/api/chop_close", methods=["POST"])
def api_chop_close():
    ok = _chop_close()
    return jsonify({"ok": ok, "action": "chop_close"})

@app.route("/video_feed")
def video_feed():
    def generate():
        last_frame = b""
        while True:
            frame = jpeg_asm.get_latest()
            if frame and frame != last_frame:
                last_frame = frame
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.1)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


# ======================== 主入口 ===============================================
def list_ports():
    print("可用串口:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device} - {p.description}")

def main():
    p = argparse.ArgumentParser(description="火箭地面控制站 (一站式)")
    p.add_argument("--port", help="COM 端口 (火箭数传)")
    p.add_argument("--baudrate", type=int, default=57600)
    p.add_argument("--chop-port", default=None, help="筷子架 COM 端口")
    p.add_argument("--model-path", default=None, help="GGUF 模型路径 (不指定则只做面板)")
    p.add_argument("--n-gpu-layers", type=int, default=0, help="GPU 层数")
    p.add_argument("--http-port", type=int, default=8080, help="Web 端口")
    p.add_argument("--list", action="store_true")
    a = p.parse_args()

    if a.list: list_ports(); return
    if not a.port:
        print("请用 --port COM3 指定端口"); list_ports(); return

    # 启动火箭串口 + 控制线程
    if a.model_path:
        print(f"\n  🚀 地面控制站 (LLM)")
        print(f"  火箭串口: {a.port} @ {a.baudrate}")
        print(f"  筷子架:   {a.chop_port or '未连接(无远程一键打开)'}")
        print(f"  模型: {a.model_path}")
        print(f"  Web:  http://localhost:{a.http_port}\n")
    else:
        print(f"\n  📊 地面监控面板")
        print(f"  火箭串口: {a.port} @ {a.baudrate}")
        print(f"  筷子架:   {a.chop_port or '未连接'}")
        print(f"  Web:  http://localhost:{a.http_port}\n")

    t_rocket = threading.Thread(target=control_thread,
                         args=(a.port, a.baudrate, a.model_path or "", a.n_gpu_layers),
                         daemon=True)
    t_rocket.start()
    time.sleep(0.5)

    # 筷子架: 只初始化串口 (不启动线程, 不读数据)
    if a.chop_port:
        try:
            chop_ser = serial.Serial(a.chop_port, 115200, timeout=0.01)
            print(f"  [CHOP] {a.chop_port} 已连接 (一键打开就绪)")
        except Exception as e:
            print(f"  [CHOP] 串口打开失败: {e}")
    else:
        chop_ser = None

    app.run(host="0.0.0.0", port=a.http_port, debug=False, threaded=True)

if __name__ == "__main__":
    main()
