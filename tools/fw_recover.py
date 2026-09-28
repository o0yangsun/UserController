#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""探测/恢复：查目标是否被 halt，必要时放跑。不做任何写变量操作（第二步才写）。"""
import os
import importlib.util
import json
import sys
import time

OCTO = os.environ.get(
    'OCTO_CLIENT',
    r'C:\Users\12054\.workbuddy\skills\octolink-servo-bus-id\scripts\octo_client.py')
spec = importlib.util.spec_from_file_location('oc', OCTO)
oc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(oc)

step = sys.argv[1] if len(sys.argv) > 1 else 'status'

b = oc.Bridge(timeout=8)
b.open()
print('bridge open OK, 工具数 = %d' % len(b.list_tools()))

if step == 'status':
    for name in ('get_gdb_status', 'get_backend_status'):
        r = b.call(name, {})
        txt = oc.result_text(r)
        print()
        print('=== %s ===' % name)
        print(txt[:900] if txt else '(无文本返回)')
        if not txt and r:
            print(json.dumps(r, ensure_ascii=False)[:900])
elif step == 'resume':
    r = b.call('target_continue', {'confirm': True})
    print('target_continue ->', oc.result_text(r)[:400])
    time.sleep(0.5)
    print('recheck:', oc.result_text(b.call('get_gdb_status', {}))[:400])
elif step == 'go':
    # 重新 takeover（拿新 lease）→ 放跑 → 立刻还回 lease
    print('takeover:', oc.result_text(
        b.call('debug_session_takeover', {'authorization': 'allow_halt', 'confirm': True}))[:200])
    print('continue:', oc.result_text(b.call('target_continue', {'confirm': True}))[:300])
    time.sleep(1.0)
    print('recheck:', oc.result_text(b.call('get_gdb_status', {}))[:300])
    print('release :', oc.result_text(b.call('release_debug_lease', {}))[:200])
elif step == 'set':
    # 通用在线写一个 volatile 变量并读回：_recover.py set usb_pc_write_speed 600
    expr = sys.argv[2]
    val = sys.argv[3]
    print('takeover:', oc.result_text(
        b.call('debug_session_takeover', {'authorization': 'allow_halt', 'confirm': True}))[:80])
    w = oc.result_text(b.call('debug_write_and_verify',
                              {'expression': expr, 'value': val,
                               'confirm': True, 'timeoutMs': 6000}))
    print('  write %-26s = %-6s -> %s' % (expr, val, 'OK' if '"success": true' in w else w[:120]))
    rd = oc.result_text(b.call('gdb_read_expressions_safe',
                               {'expressions': [expr], 'haltIfRunning': True,
                                'resumeAfterRead': True, 'timeoutMs': 9000}))
    print('  readback ->', rd[:200])
    print('continue:', oc.result_text(b.call('target_continue', {'confirm': True}))[:80])
    print('release :', oc.result_text(b.call('release_debug_lease', {}))[:80])
elif step == 'unstick':
    # 在线解除 J5 的堵转锁存（volatile，烧录/复位即失效）
    JI = int(sys.argv[2]) - 1 if len(sys.argv) > 2 else 4
    print('takeover:', oc.result_text(
        b.call('debug_session_takeover', {'authorization': 'allow_halt', 'confirm': True}))[:120])
    for expr, val in (('s_stalled[%d]' % JI, '0'),
                      ('s_stall_ep[%d]' % JI, '0'),
                      ('s_stall_ref_deg[%d]' % JI, '-500'),
                      ('s_stall_tgt_deg[%d]' % JI, '-500')):
        w = oc.result_text(b.call('debug_write_and_verify',
                                  {'expression': expr, 'value': val,
                                   'confirm': True, 'timeoutMs': 6000}))
        print('  write %-22s = %-6s -> %s' % (expr, val, 'OK' if '"success": true' in w else w[:120]))
    rd = oc.result_text(b.call('gdb_read_expressions_safe',
                               {'expressions': ['s_stalled', 's_stall_ep', 's_stall_ref_deg'],
                                'haltIfRunning': True, 'resumeAfterRead': True, 'timeoutMs': 9000}))
    print('readback ->', rd[:400])
    print('continue:', oc.result_text(b.call('target_continue', {'confirm': True}))[:120])
    print('release :', oc.result_text(b.call('release_debug_lease', {}))[:120])
elif step == 'restore':
    # 需要 lease 才能写；写回 J5 软上限 87
    print('takeover:', oc.result_text(
        b.call('debug_session_takeover', {'authorization': 'allow_halt', 'confirm': True}))[:200])
    w = oc.result_text(b.call('debug_write_and_verify',
                              {'expression': 'usb_pc_soft_max_deg[4]', 'value': '87.0',
                               'confirm': True, 'timeoutMs': 6000}))
    print('write 87.0 ->', w[:300])
    rd = oc.result_text(b.call('gdb_read_expressions_safe',
                               {'expressions': ['usb_pc_soft_min_deg', 'usb_pc_soft_max_deg'],
                                'haltIfRunning': True, 'resumeAfterRead': True, 'timeoutMs': 9000}))
    print('readback ->', rd[:400])
    print('continue:', oc.result_text(b.call('target_continue', {'confirm': True}))[:200])
    print('release :', oc.result_text(b.call('release_debug_lease', {}))[:200])

b.close()
print()
print('done.')
