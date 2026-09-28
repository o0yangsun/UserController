#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""烧录后只读固件体检：确认烧的是哪版（用新符号判据）、软限位是否正确、计数是否归零。

只用 GDB 只读，不写任何变量。
"""
import os
import importlib.util
import json
import sys

spec = importlib.util.spec_from_file_location(
    'oc', r'C:\Users\12054\.workbuddy\skills\octolink-servo-bus-id\scripts\octo_client.py')
oc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(oc)

# ---- 分组：先读"新固件指纹"，再读配置，最后读计数 ----
FINGERPRINT = [
    's_stall_ep',            # ★ 只有 a37b029 才有（episode 锚点）
]
CONFIG = [
    'usb_pc_report_ms',
    'usb_pc_soft_min_deg',
    'usb_pc_soft_max_deg',
    'usb_pc_consist_tol_ticks',
    'usb_pc_write_limit_deg',
    'usb_pc_write_eps_deg',
    'usb_pc_write_speed',
    'usb_pc_stall_window_ms',
    'usb_pc_stall_eps_deg',
    'usb_pc_stall_unload',
    'usb_pc_stall_retry_ms',
    'usb_pc_stall_retry_deg',
]
COUNTERS = [
    'usb_pc_tx_cnt', 'usb_pc_rx_cnt', 'usb_pc_rx_bad_cnt',
    'usb_pc_write_cnt', 'usb_pc_write_reject_cnt', 'usb_pc_write_stale_cnt',
    'usb_pc_write_stall_cnt', 'usb_pc_write_stall_skip_cnt',
    'usb_pc_stall_unload_cnt', 'usb_pc_stall_retry_cnt',
    'usb_pc_consist_err_cnt', 'usb_pc_hold_cnt',
]
STATE = [
    's_stalled', 's_stall_ep',
    's_stall_ref_ms', 's_stall_ref_deg',
]

b = oc.Bridge()
b.open()

t = oc.result_text(b.call('debug_session_takeover',
                          {'authorization': 'allow_halt', 'confirm': True}))
print('debug lease :', 'OK' if '"success": true' in t else t[:200])

def read(exprs, title):
    r = b.call('gdb_read_expressions_safe',
               {'expressions': exprs, 'haltIfRunning': True,
                'resumeAfterRead': True, 'timeoutMs': 8000})
    txt = oc.result_text(r)
    print()
    print('== %s ==' % title)
    try:
        res = json.loads(txt).get('results', [])
        if not res:
            print('   （无结果）', txt[:300])
        for it in res:
            print('  %-34s = %s' % (it.get('expression'), it.get('value')))
    except Exception as e:
        print('  解析失败:', e)
        print('  ', txt[:500])

read(FINGERPRINT, '固件指纹（symbol 存在 = a37b029 已烧录）')
read(CONFIG, '配置值（软限位应为内缩 3° 那组）')
read(STATE, '堵转状态（复位后应全 0 / ref_ms 递增）')
read(COUNTERS, '计数器（刚烧录应接近全 0）')

try:
    b.call('target_continue', {'confirm': True})
    b.call('release_debug_lease', {})
except Exception:
    pass
b.close()
print()
print('done.')
