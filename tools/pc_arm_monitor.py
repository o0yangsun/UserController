#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pc_arm_monitor.py —— PC 侧：读主臂板（0x60 帧）/ 下发命令（0x61 / 0x62）

协议（与车端 Engineering_Robot_H723_ 完全一致）：
    帧: [0]0xFF [1]0x05 [2]命名ID [3]数据长度N [4..]数据 [N+4]sum [N+5]addr
    帧长 = 4 + N + 2
    校验: sum = Σbuf[0..N+3] & 0xFF ; addr = Σsum & 0xFF

    0x60  主臂 → PC   6 关节角(弧度×10000, int32 小端) + 使能状态(int8)   数据长 25，帧长 31
    0x61  PC → 主臂   6 个目标关节角(弧度×10000) + 生效标志(int8)          数据长 25
    0x62  PC → 主臂   使能/失能(int8)                                      数据长 1

用法：
    python pc_arm_monitor.py                     # 列出所有串口
    python pc_arm_monitor.py COM7                # 持续监视（Ctrl+C 退出）
    python pc_arm_monitor.py COM7 --deg          # 同时显示角度制
    python pc_arm_monitor.py COM7 --raw          # 额外打印每帧原始十六进制
    python pc_arm_monitor.py COM7 --enable 1     # 发 0x62 使能（0=失能），发完退出
    python pc_arm_monitor.py COM7 --target 0.1 0.2 0.3 0.4 0.5 0.6   # 发 0x61（弧度），发完退出

依赖：pyserial      安装：pip install pyserial
"""

import argparse
import struct
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少依赖 pyserial。请先执行：pip install pyserial")
    sys.exit(2)

HEAD = 0xFF
ADDR = 0x05
ID_UP = 0x60
ID_TARGET = 0x61
ID_ENABLE = 0x62
SCALE = 10000.0


# ----------------------------------------------------------------------------- 打包
def build_frame(nid: int, payload: bytes) -> bytes:
    """按协议组帧（含 sum + addr 校验）"""
    dlen = len(payload)
    body = bytes([HEAD, ADDR, nid, dlen]) + payload
    s = 0
    a = 0
    for b in body:
        s = (s + b) & 0xFF
        a = (a + s) & 0xFF
    return body + bytes([s, a])


def make_enable_frame(enable: int) -> bytes:
    return build_frame(ID_ENABLE, bytes([1 if enable else 0]))


def make_target_frame(rad_list, flag: int = 1) -> bytes:
    payload = b"".join(struct.pack("<i", int(round(r * SCALE))) for r in rad_list)
    return build_frame(ID_TARGET, payload + bytes([flag & 0xFF]))


# ----------------------------------------------------------------------------- 解析
class StreamParser:
    """字节流解析器：能吃任意切分的数据（不假设一次 read 就是一整帧）"""

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.bad = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            # 找帧头
            while len(self.buf) >= 2 and not (self.buf[0] == HEAD and self.buf[1] == ADDR):
                del self.buf[0]
            if len(self.buf) < 6:
                break
            nid = self.buf[2]
            dlen = self.buf[3]
            total = 4 + dlen + 2
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            # 校验
            s = 0
            a = 0
            for b in frame[: 4 + dlen]:
                s = (s + b) & 0xFF
                a = (a + s) & 0xFF
            if frame[4 + dlen] == s and frame[4 + dlen + 1] == a:
                out.append((nid, frame[4 : 4 + dlen]))
                self.frames += 1
                del self.buf[:total]
            else:
                self.bad += 1
                del self.buf[0]  # 只丢一个字节，重新同步
        return out


def decode_0x60(payload: bytes):
    """返回 (6 个弧度值, 使能状态)"""
    vals = struct.unpack("<6i", payload[:24])
    return [v / SCALE for v in vals], payload[24]


# ----------------------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="主臂板 USB CDC 监视 / 命令工具")
    ap.add_argument("port", nargs="?", help="串口号，如 COM7")
    ap.add_argument("--baud", type=int, default=115200, help="USB CDC 下波特率无实际意义，随便填")
    ap.add_argument("--deg", action="store_true", help="同时显示角度制")
    ap.add_argument("--raw", action="store_true", help="额外打印每帧原始十六进制")
    ap.add_argument("--enable", type=int, choices=(0, 1), help="发 0x62：1=使能 0=失能，发完退出")
    ap.add_argument("--target", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                    help="发 0x61：6 个目标角【弧度】，发完退出")
    args = ap.parse_args()

    if not args.port:
        print("可用串口：")
        for p in list_ports.comports():
            print("   %-8s %s" % (p.device, p.description))
        print("\n用法： python pc_arm_monitor.py COM7")
        return 0

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print("打开串口失败：%s" % e)
        return 1

    # ---- 一次性命令模式 ----
    if args.enable is not None:
        ser.write(make_enable_frame(args.enable))
        ser.flush()
        print("已发送 0x62：%s" % ("使能" if args.enable else "失能"))
        time.sleep(0.2)
        ser.close()
        return 0

    if args.target:
        ser.write(make_target_frame(args.target))
        ser.flush()
        print("已发送 0x61：%s rad" % [round(x, 4) for x in args.target])
        time.sleep(0.2)
        ser.close()
        return 0

    # ---- 监视模式 ----
    print("监听 %s （Ctrl+C 退出）" % args.port)
    hdr = "  时间   fps  使能  " + "  ".join("J%d" % i for i in range(1, 7))
    print(hdr)
    print("-" * len(hdr) * 2)

    parser = StreamParser()
    t0 = time.time()
    last_print = 0.0
    latest = None

    try:
        while True:
            data = ser.read(256)
            if data:
                for nid, payload in parser.feed(data):
                    if nid == ID_UP and len(payload) >= 25:
                        latest = decode_0x60(payload)
                        if args.raw:
                            print("   RAW:", bytes([HEAD, ADDR, nid, len(payload)]).hex(" "),
                                  payload.hex(" "))
            now = time.time()
            if latest and (now - last_print) >= 0.5:
                rad, en = latest
                el = now - t0
                fps = parser.frames / el if el > 0 else 0
                if args.deg:
                    vals = "  ".join("%7.2f" % (r * 57.29577951) for r in rad)
                else:
                    vals = "  ".join("%7.4f" % r for r in rad)
                print("%7.1fs %5.1f  %-4s  %s" % (el, fps, en, vals))
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        el = time.time() - t0
        print("\n共收 %d 帧，校验失败 %d 帧，用时 %.1fs（平均 %.1f fps）"
              % (parser.frames, parser.bad, el, parser.frames / el if el else 0))
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
