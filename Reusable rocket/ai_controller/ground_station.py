# ⚠️ DEPRECATED (v3.1) — 本文件已被 web_control.py 完全取代。
#    请改用 web_control.py (含 Web 面板/LLM/筷子架/视频/急停 一站式功能)。
#    保留本文件仅为兼容旧工作流, 不再维护新功能。
"""
地面控制站 — 数传链路版 (SiK/ELRS/任意串口模块) [旧版, 见上方弃用说明]
------------------------------------------------
架构:
  火箭 ESP32 ←UART→ 数传模块 ←RF→ 数传模块 ←USB→ PC

二进制协议 (带 CRC8):
  遥测 (火箭→PC): 0xCD + 56B data + CRC8  = 58B
  指令 (PC→火箭): 0xAB + 13B data + CRC8  = 15B

用法:
  python ground_station.py --port COM3 --model-path ./model.gguf
  python ground_station.py --list   # 列出可用串口
"""
from __future__ import annotations
import sys, os, json, time, struct, argparse
import serial
import serial.tools.list_ports

sys.path.insert(0, os.path.dirname(__file__))
from llm_service import LLMController

# ==== 包结构 ====
# 遥测: sync(1B) + 14 floats(56B) + 4 uint8(4B) + 1 float(4B) + CRC8(1B) = 66B
TELEM_FMT   = "<B 14f 4B f B"   # sync + 14 floats + 4 bytes + 1 float + crc
TELEM_SYNC  = 0xCD
TELEM_SIZE  = 66

# 指令: sync(1B) + 3 floats + 1B + crc(1B)  = 15B
CMD_FMT     = "<B 3f B B"
CMD_SYNC    = 0xAB
CMD_SIZE    = 15

PHASE_NAMES = {0:"idle",1:"ignition",2:"ascending",3:"hovering",4:"descending",5:"capture",6:"abort"}


def crc8(data: bytes) -> int:
    """CRC8 Dallas/Maxim (与 ESP32 端一致)"""
    crc = 0
    for b in data:
        for _ in range(8):
            if (crc ^ b) & 1:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
            b >>= 1
    return crc & 0xFF


class GroundStation:
    def __init__(self, port: str, model_path: str, n_gpu_layers: int = 0,
                 baudrate: int = 57600):
        self.port = port
        self.baudrate = baudrate
        self.llm = LLMController(model_path, n_gpu_layers)
        self.ser = None
        self.running = False
        self.stats = {"telem_rx": 0, "cmd_tx": 0, "crc_err": 0, "sync_err": 0}
        self._buf = b""

    def start(self):
        print("=" * 56)
        print("  地面控制站 (数传链路)")
        print(f"  串口: {self.port} @ {self.baudrate}")
        print(f"  LLM:  {self.llm.model_path}")
        print(f"  PID:  备援已就绪 (双保险, 推力 22.1N)")
        print("=" * 56)

        self.llm.load()

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
        except Exception as e:
            print(f"  串口打开失败: {e}")
            for p in serial.tools.list_ports.comports():
                print(f"    {p.device} - {p.description}")
            return

        self.running = True
        print("\n等待遥测...\n")

        try:
            while self.running:
                if self.ser.in_waiting:
                    self._buf += self.ser.read(self.ser.in_waiting)
                self._parse_telem()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.ser: self.ser.close()

        print(f"\n遥测RX={self.stats['telem_rx']} 指令TX={self.stats['cmd_tx']}")
        print(f"CRC错={self.stats['crc_err']} 推理={self.llm.avg_inference_ms:.0f}ms")
        self.llm.unload()

    def _parse_telem(self):
        while len(self._buf) >= TELEM_SIZE:
            if self._buf[0] != TELEM_SYNC:
                self._buf = self._buf[1:]
                self.stats["sync_err"] += 1
                continue

            pkt = self._buf[:TELEM_SIZE]
            if crc8(pkt[:-1]) != pkt[-1]:
                self._buf = self._buf[1:]
                self.stats["crc_err"] += 1
                continue

            self._buf = self._buf[TELEM_SIZE:]
            self.stats["telem_rx"] += 1

            try:
                vals = struct.unpack(TELEM_FMT, pkt)
            except struct.error:
                continue

            # vals[0]=sync, [1]=alt, [2]=vz, [3]=vx, [4]=vy, [5]=horiz_err,
            # [6]=roll, [7]=pitch, [8]=yaw, [9]=fuel, [10]=throttle,
            # [11]=cam_dx, [12]=cam_dy, [13]=cam_conf, [14]=reserved,
            # [15]=phase, [16]=safety, [17]=cam_valid, [18]=v_batt, [19]=crc8
            alt       = float(vals[1])
            vz        = float(vals[2])
            vx        = float(vals[3])
            vy        = float(vals[4])
            horiz_err = float(vals[5])
            fuel      = float(vals[9])
            phase     = int(vals[15])

            vh = (vx * vx + vy * vy) ** 0.5

            llm_state = {
                "alt":       round(alt, 2),
                "vz":        round(vz, 2),
                "vh":        round(vh, 2),
                "horiz_err": round(horiz_err, 3),
                "fuel":      round(fuel, 3),
                "phase":     PHASE_NAMES.get(phase, "unknown"),
            }

            # 火箭端本地 PID 主控, LLM 10Hz 下发修正
            try:
                action = self.llm.step(llm_state)
            except Exception:
                continue  # LLM 失败: 火箭端 PID 完全自主, 不发送指令

            self._send_cmd(action)

            if self.stats["cmd_tx"] % 50 == 1:
                print(f"  [{self.stats['cmd_tx']:5d}] 高{alt:5.1f}m "
                      f"Vz{vz:+.1f} 油{action['throttle']:.0%}  "
                      f"推理{self.llm.avg_inference_ms:.0f}ms")

    def _send_cmd(self, action: dict):
        flags = 0
        raw = struct.pack(CMD_FMT, CMD_SYNC,
                          float(action["throttle"]),
                          float(action["tvc_pitch"]),
                          float(action["tvc_yaw"]),
                          flags, 0)
        pkt = raw[:-1] + bytes([crc8(raw[:-1])])
        self.ser.write(pkt)
        self.stats["cmd_tx"] += 1


def list_ports():
    print("可用串口:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device} - {p.description}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", help="COM 端口")
    p.add_argument("--model-path", required=True, help="GGUF 模型路径")
    p.add_argument("--n-gpu-layers", type=int, default=0)
    p.add_argument("--baudrate", type=int, default=57600)
    p.add_argument("--list", action="store_true")
    a = p.parse_args()

    if a.list: list_ports(); return
    if not a.port:
        print("请用 --port COM3 指定端口"); list_ports(); return

    GroundStation(a.port, a.model_path, a.n_gpu_layers, a.baudrate).start()


if __name__ == "__main__":
    main()
