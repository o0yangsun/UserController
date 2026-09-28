#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""J3 负载诊断：分辨"到不了位"是【扭矩不足】还是【速度/时间不够】。

思路：给定充足时间（每程 8s，远大于 15° 所需的 ~4s），看两个方向（重力相助 / 重力相抗）
的表现差异。若某方向**一路都在动但就是追不上目标** ⇒ 速度上限问题；
若某方向**中途彻底停住而残差很大** ⇒ 扭矩不足（或被堵转保护卸载）。

用法: test_j3_load.py [COM] [方向行程度=15] [每程秒数=8]
默认 COM10 15 8   （被试关节固定为 J3；行程会自动夹在软限位内）
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
TRAVEL = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
SECS = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0
RATE = 0.03
JI = 2                              # J3，0-based
RTOD = 57.29577951
D2R = 1.0 / RTOD
MECH = (0.0, 180.0)
SOFT = (3.0, 177.0)

CNT = ['usb_pc_write_cnt', 'usb_pc_write_stall_cnt', 'usb_pc_stall_unload_cnt',
       'usb_pc_stall_retry_cnt', 'usb_pc_write_stall_skip_cnt',
       'usb_pc_consist_err_cnt', 'usb_pc_rx_cnt']


def octo_read(names):
    """独立会话：读一次就还回 lease（避免 300s 过期把 CPU 留在 halt）。"""
    b = oc.Bridge(timeout=12)
    b.open()
    out = {}
    try:
        b.call('debug_session_takeover', {'authorization': 'allow_halt', 'confirm': True})
        r = b.call('gdb_read_expressions_safe',
                   {'expressions': list(names), 'haltIfRunning': True,
                    'resumeAfterRead': True, 'timeoutMs': 9000})
        try:
            out = {it['expression']: it.get('value')
                   for it in json.loads(oc.result_text(r)).get('results', [])}
        except Exception:
            pass
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


def octo_write(expr, val):
    """独立会话写一个 volatile 变量（写完立刻 continue + release）。"""
    b = oc.Bridge(timeout=12)
    b.open()
    ok = False
    try:
        b.call('debug_session_takeover', {'authorization': 'allow_halt', 'confirm': True})
        w = oc.result_text(b.call('debug_write_and_verify',
                                  {'expression': expr, 'value': str(val),
                                   'confirm': True, 'timeoutMs': 6000}))
        ok = ('"success": true' in w) or ('error' not in w.lower())
    except Exception as e:
        print('   （写 %s 失败：%s）' % (expr, type(e).__name__))
    finally:
        try:
            b.call('target_continue', {'confirm': True})
            b.call('release_debug_lease', {})
        except Exception:
            pass
        b.close()
    return ok


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


cur = drain(1.0)
if cur is None:
    print('❌ 收不到 0x60 帧')
    sys.exit(1)
print('起始姿态: %s' % '  '.join('J%d=%+.3f' % (i + 1, cur[i]) for i in range(6)))

base = cur[JI]
lo = max(SOFT[0], base - TRAVEL)
hi = min(SOFT[1], base + TRAVEL)
print('J3 起点 %+.3f°   软限位 [%+.0f, %+.0f]   本测目标区间 [%+.3f, %+.3f]'
      % (base, SOFT[0], SOFT[1], lo, hi))
if hi - lo < 1.0:
    print('❌ J3 距软限位太近，展不开')
    sys.exit(1)

if '--dry' in sys.argv:
    print()
    print('--dry：只校验流程与限位，不动机械。计划：')
    print('  ① J3 %+.3f° → %+.3f°（+%.0f°）跑 %.0fs' % (base, hi, TRAVEL, SECS))
    print('  ② J3 %+.3f° → %+.3f°（−%.0f°）跑 %.0fs' % (hi, lo, TRAVEL * 2, SECS))
    print('  ③ J3 %+.3f° → %+.3f°（回起点）跑 %.0fs' % (lo, base, SECS))
    print('  每程带 0x63 刹车 + 0.6s 落定；全程夹在软限位 [%+.0f, %+.0f] 内 ✅'
          % (SOFT[0], SOFT[1]))
    print('  ④ 回基准零位：临时放宽软下限 3→0 → 送 0° → 写回 3'
          '（J3 基准 0° 在软限位之外，否则回不去）')
    ser.close()
    sys.exit(0)

print()


def phase(tag, target):
    global base
    c0 = octo_read(CNT)
    start = drain(0.4)[JI]
    trace = []
    frame = None
    tgt = list(cur)
    tgt[JI] = target
    frame = m.make_target_frame([x * D2R for x in tgt])

    t0 = time.time()
    last = t0
    while time.time() - t0 < SECS:
        if time.time() - last >= RATE:
            ser.write(frame)
            ser.flush()
            last = time.time()
        a = drain(0.004)
        if a:
            trace.append((time.time() - t0, a[JI]))
    ser.write(m.make_hold_frame())
    ser.flush()
    time.sleep(0.6)
    fin = drain(0.8)[JI]
    c1 = octo_read(CNT)

    # 最后 2s 的平均速度（判断"还在动"还是"已经停住"）
    tail = [x for x in trace if x[0] >= SECS - 2.0]
    if len(tail) >= 2:
        v_tail = (tail[-1][1] - tail[0][1]) / max(1e-6, tail[-1][0] - tail[0][0])
    else:
        v_tail = 0.0
    # 峰值速度（相邻采样，去噪：取 95 分位）
    vs = []
    for i in range(1, len(trace)):
        dt = trace[i][0] - trace[i - 1][0]
        if dt > 1e-4:
            vs.append(abs(trace[i][1] - trace[i - 1][1]) / dt)
    vs.sort()
    v_peak = vs[int(len(vs) * 0.95)] if vs else 0.0

    moved = fin - start
    resid = target - fin
    print('【%s】目标 %+.2f°' % (tag, target))
    print('   起点 %+.3f° → 终点 %+.3f°   位移 %+.3f°（目标 %+.3f°）  残差 %+.3f°'
          % (start, fin, moved, target - start, resid))
    print('   速度：峰值 %.2f°/s   末2s 平均 %.2f°/s   %s'
          % (v_peak, v_tail,
             '← 仍在动（速度受限/时间不够）' if abs(v_tail) > 0.3 else '← 已停住（扭矩受限或已到位）'))
    d = {k: (fv(c1.get(k)) - fv(c0.get(k)))
         for k in CNT if fv(c0.get(k)) is not None and fv(c1.get(k)) is not None}
    print('   计数Δ: write=%s stall=%s unload=%s retry=%s skip=%s consist=%s'
          % (d.get('usb_pc_write_cnt'), d.get('usb_pc_write_stall_cnt'),
             d.get('usb_pc_stall_unload_cnt'), d.get('usb_pc_stall_retry_cnt'),
             d.get('usb_pc_write_stall_skip_cnt'), d.get('usb_pc_consist_err_cnt')))
    print()
    return fin, resid, v_tail


def home_zero(secs=6.0):
    """把 J3 送回【基准 0°】。

    ⚠️ J3 的基准 0° 落在软限位（+3°，内缩 3° 得来）**之外** ⇒ 直接送目标 0
    会被 clamp 成 3 ⇒ 永远回不到 0 ⇒ 实测姿态偏 3° ⇒ 下次复位会把整组软限位带偏 3°。
    做法：临时把 soft_min[2] 写到 **0.0**（0 就是实测机械端点，写到端点本身是安全的），
    送目标 0 让它真的走过去，再写回 3.0。
    """
    print('【回基准零位】临时放宽 J3 软下限 3.0 → 0.0（= 实测机械端点）')
    if not octo_write('usb_pc_soft_min_deg[2]', 0.0):
        print('   ⚠️ 放宽失败，跳过回零（J3 会停在软下限附近）')
        return
    try:
        cur2 = drain(0.5)
        tgt = list(cur2)
        tgt[JI] = 0.0
        frame = m.make_target_frame([x * D2R for x in tgt])
        t0 = time.time()
        last = t0
        while time.time() - t0 < secs:
            if time.time() - last >= RATE:
                ser.write(frame)
                ser.flush()
                last = time.time()
            drain(0.005)
        ser.write(m.make_hold_frame())
        ser.flush()
        time.sleep(0.5)
        fin = drain(0.8)[JI]
        print('   J3 = %+.3f°（目标 0.000°，残差 %+.3f°）' % (fin, fin))
    finally:
        ok = octo_write('usb_pc_soft_min_deg[2]', 3.0)
        back = octo_read(['usb_pc_soft_min_deg[2]']).get('usb_pc_soft_min_deg[2]')
        print('   软下限已还原 = %s %s' % (back, '✅' if ok else '❌'))
    print()


r1, res1, v1 = phase('J3 正向 (+)', hi)
r2, res2, v2 = phase('J3 反向 (−)', lo)
r3, res3, v3 = phase('J3 回起点', base)

print('=' * 88)
print('判读')
print('=' * 88)
print('  正向：残差 %+.3f°  末速 %+.2f°/s' % (res1, v1))
print('  反向：残差 %+.3f°  末速 %+.2f°/s' % (res2, v2))
print('  回位：残差 %+.3f°  末速 %+.2f°/s' % (res3, v3))
print()
if abs(v1) > 0.3 or abs(v2) > 0.3:
    print('  ⇒ 有方向"8s 后仍在动"⇒ 主要是【速度/时间】问题，不是扭矩不足')
    print('     （加重力致残差更大的那一侧 = 重力相抗方向）')
else:
    print('  ⇒ 两个方向 8s 后都停住了 ⇒ 若残差仍大，才是【扭矩不足】')
print('  ⚠️ 两侧残差之差 ≈ %+.3f°（同号说明整体偏置，异号且一大一小说明重力影响）'
      % (abs(res1) - abs(res2)))
print()

# ★ 必须真的把 J3 送回基准 0° —— 否则它停在 +30° 左右，
#   下次复位会把整组软限位带偏（见 README §14.18）。
home_zero()

ser.close()
print()
print('done.')
