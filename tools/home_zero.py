#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把六路送回【基准 0°】—— 收工前必做的家务。

为什么不能直接发目标 0：
  J2 / J3 的基准 0° 落在软限位**之外**（J2 软上限 −3°、J3 软下限 +3°，内缩 3° 得来）
  ⇒ 目标 0 会被 `clamp_soft` 钳成 ∓3° ⇒ **软件永远回不到 0°**，
    实测姿态偏 3° ⇒ 下次复位会把整组软限位跟着带偏 3°（README §14.18）。
做法：对"0° 不可达"的关节**临时放宽那一侧软限位到 0.0**
      （0 就是实测机械端点，写到端点本身是安全的），送目标 0° 走过去，再写回原值。
      所有关节的机械行程都包含 0°，所以命令 0° 本身是安全的。

用法:
    python home_zero.py COM10            # 回零
    python home_zero.py COM10 --dry      # 只打印计划，不动机械
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
DRY = '--dry' in sys.argv
RATE = 0.03
RTOD = 57.29577951
D2R = 1.0 / RTOD

# 固件默认软限位（内缩 3°）
SOFT_MIN = [-87.0, -167.0, 3.0, -177.0, -22.0, -177.0]
SOFT_MAX = [87.0, -3.0, 177.0, 177.0, 87.0, 177.0]
# 实测机械行程（0 必须在其中，否则不该命令 0）
MECH = [(-90.0, 90.0), (-170.0, 0.0), (0.0, 180.0),
        (-180.0, 180.0), (-25.0, 90.0), (-180.0, 180.0)]


def octo_session(fn):
    b = oc.Bridge(timeout=12)
    b.open()
    try:
        b.call('debug_session_takeover', {'authorization': 'allow_halt', 'confirm': True})
        return fn(b)
    finally:
        try:
            b.call('target_continue', {'confirm': True})
            b.call('release_debug_lease', {})
        except Exception:
            pass
        b.close()


def octo_read(names):
    def fn(b):
        r = b.call('gdb_read_expressions_safe',
                   {'expressions': list(names), 'haltIfRunning': True,
                    'resumeAfterRead': True, 'timeoutMs': 9000})
        try:
            return {it['expression']: it.get('value')
                    for it in json.loads(oc.result_text(r)).get('results', [])}
        except Exception:
            return {}
    return octo_session(fn) or {}


def octo_write(expr, val):
    def fn(b):
        w = oc.result_text(b.call('debug_write_and_verify',
                                  {'expression': expr, 'value': str(val),
                                   'confirm': True, 'timeoutMs': 6000}))
        return ('"success": true' in w) or ('error' not in w.lower())
    try:
        return bool(octo_session(fn))
    except Exception as e:
        print('   （写 %s 失败：%s）' % (expr, type(e).__name__))
        return False


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
        return None
    return [v * RTOD for v in rows[-1][0]]


cur = drain(0.8)
if cur is None:
    print('❌ 收不到 0x60 帧')
    sys.exit(1)
print('当前姿态: %s' % '  '.join('J%d=%+8.3f' % (i + 1, cur[i]) for i in range(6)))

# ---- 哪些关节需要临时放宽软限位才能到 0° ----
loosen = []          # [(expr, 原值, 放宽值, 说明)]
for i in range(6):
    if abs(cur[i]) < 1.0:
        continue
    need = False
    newv = None
    if SOFT_MIN[i] > 0.0:            # 0° 在下限之下（如 J3）
        need, newv = True, ('usb_pc_soft_min_deg[%d]' % i, SOFT_MIN[i], 0.0)
    elif SOFT_MAX[i] < 0.0:          # 0° 在上限之上（如 J2）
        need, newv = True, ('usb_pc_soft_max_deg[%d]' % i, SOFT_MAX[i], 0.0)
    if need:
        ml, mh = MECH[i]
        assert ml <= 0.0 <= mh, 'J%d 的机械行程不含 0°，不该命令 0°' % (i + 1)
        loosen.append(newv + ('J%d' % (i + 1),))

print()
if loosen:
    print('需要临时放宽软限位（0° 在软限位之外）:')
    for expr, orig, newv, jn in loosen:
        print('   %-26s %+.1f° → %+.1f°   （%s）' % (expr, orig, newv, jn))
else:
    print('无需放宽：六路的软限位都包含 0°')

travel = max(abs(c) for c in cur)
secs = max(6.0, travel / 8.0 + 3.0)
print()
print('计划：送固定目标 0°/0°/0°/0°/0°/0°，持续 %.1fs（最大行程 %.1f°）' % (secs, travel))

if DRY:
    print()
    print('--dry：不动机械，结束。')
    ser.close()
    sys.exit(0)

print()
try:
    for expr, orig, newv, jn in loosen:
        ok = octo_write(expr, newv)
        back = fv(octo_read([expr]).get(expr))
        print('   放宽 %-26s → %+.1f  %s（读回 %s）'
              % (expr, newv, 'OK' if ok else 'FAIL', back))
        if back is None or abs(back - newv) > 0.05:
            print('   ❌ 放宽未生效，中止（避免发出到不了的目标）')
            sys.exit(1)

    frame = m.make_target_frame([0.0] * 6)     # ★ 固定目标 0（全靠固件的 10°/步限幅）
    t0 = time.time()
    last = t0
    while time.time() - t0 < secs:
        if time.time() - last >= RATE:
            ser.write(frame)
            ser.flush()
            last = time.time()
        drain(0.005)
    ser.write(m.make_hold_frame())             # 0x63 刹车
    ser.flush()
    time.sleep(0.6)
    fin = drain(0.8)
    print()
    print('回零结果:')
    print('   %-5s %11s %11s %10s' % ('关节', '起点', '终点', '残差'))
    worst = 0.0
    for i in range(6):
        r = fin[i] - 0.0
        worst = max(worst, abs(r))
        print('   J%-4d %11.3f %11.3f %+10.3f%s'
              % (i + 1, cur[i], fin[i], r, '   ← 未回零' if abs(r) > 1.5 else ''))
    print('   最大残差 %.3f°' % worst)
finally:
    print()
    print('还原软限位:')
    for expr, orig, newv, jn in loosen:
        ok = octo_write(expr, orig)
        back = fv(octo_read([expr]).get(expr))
        print('   %-26s → %+.1f  %s（读回 %s）'
              % (expr, orig, '✅' if ok and back is not None and abs(back - orig) < 0.05 else '❌', back))
    ser.close()
    print()
    print('done.')
