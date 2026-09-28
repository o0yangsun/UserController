#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""失能/使能切换的物理后果测量（重力坠落）。

背景：回归脚本的 ② 会做一次 0x62 失能→使能。发现 J5 在失能期间会明显下掉，
     而"位置伺服只到位阈值内才修正"⇒ 每次切换都留一点偏移 ⇒ 多轮后累积漂移。
      本脚本量化：失能后各路掉多少、掉多快、使能后能否回到原位。

用法: test_enable_toggle.py [COM] [失能观察秒数=2.0]
"""
import importlib.util
import os
import sys
import time

MON = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pc_arm_monitor.py')
if not os.path.exists(MON):
    MON = r'C:\Users\12054\Desktop\Custom Controller\STM32H7232_UserController_1\tools\pc_arm_monitor.py'
spec = importlib.util.spec_from_file_location('m', MON)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM10'
FREE_SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
RTOD = 57.29577951
D2R = 1.0 / RTOD

ser = serial.Serial(PORT, 115200, timeout=0.02)
p = m.StreamParser()


def sample(secs):
    """采样，返回 (角度轨迹 list[list[6]], 末使能位)"""
    rows = []
    en = None
    t0 = time.time()
    while time.time() - t0 < secs:
        d = ser.read(4096)
        if d:
            for nid, pl in p.feed(d):
                if nid == 0x60 and len(pl) == 25:
                    dec = m.decode_0x60(pl)
                    rows.append([v * RTOD for v in dec[0]])
                    en = dec[1]
    return rows, en


def send(frame):
    ser.write(frame)
    ser.flush()


print('=' * 84)
print('失能/使能切换 · 重力坠落测量   %s' % PORT)
print('=' * 84)

rows, en = sample(1.2)
if not rows:
    print('❌ 收不到 0x60 帧')
    sys.exit(1)
base = rows[-1]
print('① 切换前：使能位=%s' % en)
print('   %s' % '  '.join('J%d=%+7.3f' % (i + 1, base[i]) for i in range(6)))

print()
print('② 发 0x62 = 0（失能，扭矩放开），观察 %.1fs…' % FREE_SECS)
send(m.make_enable_frame(0))
rows_free, en_free = sample(FREE_SECS)
if not rows_free:
    print('   ❌ 失能后收不到帧')
    sys.exit(1)
low = [min(r[i] for r in rows_free) for i in range(6)]
fin_free = rows_free[-1]
print('   使能位=%s' % en_free)
print('   %-5s %10s %10s %10s' % ('关节', '切换前', '失能后', '变化'))
for i in range(6):
    print('   J%-4d %10.3f %10.3f %+10.3f%s'
          % (i + 1, base[i], fin_free[i], fin_free[i] - base[i],
             '   ← 明显坠落' if abs(fin_free[i] - base[i]) > 2.0 else ''))

print()
print('③ 发 0x62 = 1（重新使能，锁死在当前位置），观察 1.2s…')
send(m.make_enable_frame(1))
rows_lock, en_lock = sample(1.2)
fin_lock = rows_lock[-1] if rows_lock else fin_free
print('   使能位=%s' % en_lock)
print('   %-5s %10s %10s' % ('关节', '失能末', '锁死后'))
for i in range(6):
    print('   J%-4d %10.3f %10.3f' % (i + 1, fin_free[i], fin_lock[i]))

print()
print('=' * 84)
print('结论')
print('=' * 84)
worst = max(range(6), key=lambda i: abs(fin_free[i] - base[i]))
print('  失能期间最大坠落：J%d  %+.3f°（落差 %.3f°）'
      % (worst + 1, fin_free[worst] - base[worst], abs(fin_free[worst] - base[worst])))
print('  ⚠️ 注意：失能会让重力负载关节【直接掉到机械下限】，'
      '而"重新使能"只是锁死在【当时的位置】，不会回到原位。')
print('  ⇒ 任何"失能→使能"循环都会引入净位移；做数据采集要避开，或用 0x63 原地刹车代替。')
ser.close()
print()
print('done.')
