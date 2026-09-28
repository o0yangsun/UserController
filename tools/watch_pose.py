#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""姿态 + 使能位长时间监视（带可选时序动作），用于抓"自发位移/坠落"和测 PA15 按键。

用法:
    watch_pose.py COM10 45            # 被动监视 45s（可用来按 PA15 测按键）
    watch_pose.py COM10 45 --seq      # 先按时间线做一轮动作，再被动监视

--seq 时间线：
    2.0s  0x62 = 0（失能）
    3.6s  0x62 = 1（使能）
    5.0s  0x63（刹车）
    6.5s  固定目标 J5 = 当前+5°（持续 2s）
    9.0s  0x63（刹车）
    之后  被动监视到结束

输出：每当"某路相对上一采样变化 >0.3°"或"使能位跳变"就打印一行（带时间戳），
      结束时给每路的 min/max/末值 + 事件计数。
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
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
SEQ = '--seq' in sys.argv
RTOD = 57.29577951
D2R = 1.0 / RTOD
JI = 4                      # J5

ser = serial.Serial(PORT, 115200, timeout=0.02)
p = m.StreamParser()

events = []
last = None
last_en = None
t0 = time.time()
seq_frame = None
seq_note = ''

lo = [1e9] * 6
hi = [-1e9] * 6
frame_target = None
sent_stop = time.time() + 1e9

print('=' * 84)
print('姿态监视   %s   时长 %.0fs   %s' % (PORT, SECS, '（含时序动作）' if SEQ else ''))
if SEQ:
    print('  时间线: 2.0s 失能(0x62=0) / 3.6s 使能(0x62=1) / 5.0s 刹车(0x63) / '
          '6.5s 固定目标+5° 2s / 9.0s 刹车')
print('  ★ 若要测 PA15 按键：请在监视期间按一下按键，这里会记录使能位跳变')
print('=' * 84)

n_move = 0
while time.time() - t0 < SECS:
    t = time.time() - t0

    # ---- 时序动作 ----
    if SEQ:
        if 2.0 <= t < 2.0 + 0.1 and not seq_note.startswith('off'):
            ser.write(m.make_enable_frame(0)); ser.flush()
            seq_note = 'off'; print('%6.2fs  >>> 发 0x62=0（失能）' % t)
        if 3.6 <= t < 3.7 and seq_note == 'off':
            ser.write(m.make_enable_frame(1)); ser.flush()
            seq_note = 'on'; print('%6.2fs  >>> 发 0x62=1（使能）' % t)
        if 5.0 <= t < 5.1 and seq_note == 'on':
            ser.write(m.make_hold_frame()); ser.flush()
            seq_note = 'hold1'; print('%6.2fs  >>> 发 0x63（刹车）' % t)
        if 6.5 <= t < 6.6 and last is not None:
            tg = list(last)
            tg[JI] = last[JI] + 5.0
            frame_target = m.make_target_frame([x * D2R for x in tg])
            sent_stop = t + 2.0
            seq_note = 'tgt'
            print('%6.2fs  >>> 下发单帧固定目标 J%d = %+.2f°（位置伺服会走过去）'
                  % (t, JI + 1, tg[JI]))
        if t >= sent_stop and seq_note == 'tgt':
            ser.write(m.make_hold_frame()); ser.flush()
            seq_note = 'hold2'; print('%6.2fs  >>> 发 0x63（刹车）' % t)

    if frame_target is not None and t < sent_stop:
        ser.write(frame_target)
        ser.flush()
        frame_target = None

    d = ser.read(4096)
    if d:
        for nid, pl in p.feed(d):
            if nid != 0x60 or len(pl) != 25:
                continue
            dec = m.decode_0x60(pl)
            cur = [v * RTOD for v in dec[0]]
            en = dec[1]
            for i in range(6):
                lo[i] = min(lo[i], cur[i]); hi[i] = max(hi[i], cur[i])
            if last is None:
                last = cur
                last_en = en
                print('%6.2fs  基线: %s   使能位=%d'
                      % (t, ' '.join('J%d=%+7.3f' % (i + 1, cur[i]) for i in range(6)), en))
                continue
            if en != last_en:
                events.append((t, 'enable %d→%d' % (last_en, en)))
                print('%6.2fs  ★★ 使能位 %d → %d' % (t, last_en, en))
                last_en = en
            moves = [(abs(cur[i] - last[i]), i) for i in range(6)]
            mx, mi = max(moves)
            if mx > 0.3:
                n_move += 1
                events.append((t, 'J%d %+.3f' % (mi + 1, cur[mi] - last[mi])))
                print('%6.2fs  Δ J%d %+7.3f°  （现 %+.3f°，上一 %.3f°）'
                      % (t, mi + 1, cur[mi] - last[mi], cur[mi], last[mi]))
            last = cur
    time.sleep(0.02)

print()
print('=' * 84)
print('汇总')
print('=' * 84)
print('  %-5s %11s %11s %11s %10s' % ('关节', 'min', 'max', '末值', '极差'))
for i in range(6):
    print('  J%-4d %11.3f %11.3f %11.3f %10.3f'
          % (i + 1, lo[i], hi[i], last[i], hi[i] - lo[i]))
print()
print('  使能位最终 = %s   位移事件 %d 次   校验失败 %d' % (last_en, n_move, p.bad))
print('  事件明细：')
for t, s in events:
    print('    %6.2fs  %s' % (t, s))
ser.close()
print()
print('done.')
