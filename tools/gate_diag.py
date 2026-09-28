#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""闸门定位：发一段目标，对比前后计数器，判定是哪一道闸拦住了。

用法: _gate_diag.py COM10 [关节1-based=5] [目标角度=0] [秒数=3]
"""
import os
import importlib.util
import json
import sys
import time

MON = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pc_arm_monitor.py')
OCTO = os.environ.get(
    'OCTO_CLIENT',
    r'C:\Users\12054\.workbuddy\skills\octolink-servo-bus-id\scripts\octo_client.py')

s1 = importlib.util.spec_from_file_location('m', MON)
m = importlib.util.module_from_spec(s1)
s1.loader.exec_module(m)
s2 = importlib.util.spec_from_file_location('oc', OCTO)
oc = importlib.util.module_from_spec(s2)
s2.loader.exec_module(oc)
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM10'
JI = (int(sys.argv[2]) - 1) if len(sys.argv) > 2 else 4
TGT = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
SECS = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
RTOD = 57.29577951
D2R = 1.0 / RTOD

NAMES = ['usb_pc_rx_cnt', 'usb_pc_write_cnt', 'usb_pc_write_stale_cnt',
         'usb_pc_write_reject_cnt', 'usb_pc_write_stall_cnt',
         'usb_pc_write_stall_skip_cnt', 'usb_pc_stall_unload_cnt',
         'usb_pc_stall_retry_cnt', 'usb_pc_consist_err_cnt',
         'usb_pc_consist_err_ticks', 'usb_pc_hold_cnt',
         's_stalled', 's_stall_ep', 's_stall_ref_deg', 's_stall_ref_ms',
         's_stall_tgt_deg', 's_stall_ms']


def octo_read(names):
    b = oc.Bridge(timeout=12)
    b.open()
    out = {}
    try:
        b.call('debug_session_takeover', {'authorization': 'allow_halt', 'confirm': True})
        r = b.call('gdb_read_expressions_safe',
                   {'expressions': list(names), 'haltIfRunning': True,
                    'resumeAfterRead': True, 'timeoutMs': 9000})
        txt = oc.result_text(r)
        try:
            out = {it['expression']: it.get('value')
                   for it in json.loads(txt).get('results', [])}
        except Exception:
            print('  解析失败:', txt[:300])
    finally:
        try:
            b.call('target_continue', {'confirm': True})
            b.call('release_debug_lease', {})
        except Exception:
            pass
        b.close()
    return out


def fv(s):
    try:
        return float(str(s).split()[0])
    except Exception:
        return None


ser = serial.Serial(PORT, 115200, timeout=0.02)
p = m.StreamParser()


def drain(secs):
    rows = []
    t0 = time.time()
    while time.time() - t0 < secs:
        d = ser.read(4096)
        if d:
            for nid, pl in p.feed(d):
                if nid == 0x60 and len(pl) == 25:
                    rows.append(m.decode_0x60(pl))
    if not rows:
        return None, None
    return [v * RTOD for v in rows[-1][0]], rows[-1][1]


cur, en = drain(0.8)
print('使能位=%s  当前姿态: %s' % (en, '  '.join('J%d=%+.3f' % (i + 1, cur[i]) for i in range(6))))
print('被试 J%d：%.3f° → 目标 %.1f°' % (JI + 1, cur[JI], TGT))

c0 = octo_read(NAMES)

tgt = list(cur)
tgt[JI] = TGT
frame = m.make_target_frame([x * D2R for x in tgt])
n = 0
t0 = time.time()
last = t0
while time.time() - t0 < SECS:
    if time.time() - last >= 0.03:
        ser.write(frame)
        ser.flush()
        last = time.time()
        n += 1
    drain(0.005)
print('发送 %d 帧（%.1fs）' % (n, SECS))

fin, en2 = drain(0.8)
print('结果: J%d = %.3f°（位移 %+.3f°）  使能位=%s' % (JI + 1, fin[JI], fin[JI] - cur[JI], en2))

c1 = octo_read(NAMES)
print()
print('%-30s %12s %12s %10s' % ('计数器', '前', '后', 'Δ'))
for k in NAMES:
    a, b_ = fv(c0.get(k)), fv(c1.get(k))
    d = ('%+g' % (b_ - a)) if (a is not None and b_ is not None) else '?'
    print('%-30s %12s %12s %10s' % (k, c0.get(k), c1.get(k), d))

ser.close()
print()
print('done.')
