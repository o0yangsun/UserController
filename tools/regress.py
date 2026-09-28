# -*- coding: utf-8 -*-
"""
主臂控制器 · 一键回归验证（烧录新固件后跑这一条即可）

覆盖 8 个验收阶段 + 收尾，每阶段独立判定 PASS/FAIL，最后给汇总表：

  ① 上行链路   0x60 帧质量：帧率 / 校验失败 / 帧长 / 命名 ID
  ② 下行链路   0x62 使能/失能：用上行帧里的使能位自证（发出去必须回得来）
  ③ episode 锚点  ★本次修复的核心验收点
      先发 0x63（它会污染六路 s_stall_ref_ms）→ 静置 >600ms → 发【固定目标】
      必须：① 立即动作（修复前 0.000°）② stall_cnt / unload_cnt 不涨
      ⚠️ 必须夹 0x63：否则 ref_ms 仍为 0，会被更弱的"哨兵"（5aa1cc0）掩盖，
         测不出 a37b029 的 episode 锚点是否真的生效。
  ④ 软限位     【在线收紧】软上限到离机械限位很远处，再发【远超收紧值】的目标
      ⇒ 必须停在收紧后的边界（不是请求值），其余五路不动；用完自动恢复原值。
      为什么不直接发"边界外 8°"：软限位是实测行程内缩 3°得出的，
      边界外 8° 已落在机械行程之外，无法区分"被钳位"与"只是被 eps 提前停住"。
  ⑤ 0x63 刹车  运动中发 0x63 → 位移应当场归零（对照：只停发会继续走完旧目标）
  ⑦ 堵转保护   命令一个"到不了"的目标 → 应判堵转 + 卸载推力；
                目标再变 ≥3° → 应放行一次重试并真的动起来（§14.10 的两个修复）
  ⑧ 超时门     停发后 >200ms 不得再驱动（用 write_cnt 增量自证）
  ⑨ 收尾       恢复姿态 + 刹车；可选失能

用法：
    python regress.py COM10                 # 默认用 J5 做被试关节
    python regress.py COM10 --joint 1       # 换关节
    python regress.py COM10 --skip-softlimit --skip-brake     # 只跑链路
    python regress.py COM10 --octo          # 额外读固件计数器（③/⑦/⑧ 的严格判据需要它）
    python regress.py COM10 --no-hardening  # 不做"先发 0x63"的严格化（旧行为）

安全设计：
  · 任何目标都不允许超出【实测机械行程】（超出直接拒绝执行）
  · ④ 的越界请求也保证落在机械行程内 ⇒ 即使钳位失效也不会撞硬限位
  · Ctrl-C / 异常 → finally 里一定发 0x63 刹车 + 恢复被改写的软限位
  · 默认【不失能】（失能会让重力负载臂塌落）
"""
import os
import argparse
import importlib.util
import sys
import time
import traceback

MON = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pc_arm_monitor.py')
OCTO = os.environ.get(
    'OCTO_CLIENT',
    r'C:\Users\12054\.workbuddy\skills\octolink-servo-bus-id\scripts\octo_client.py')

spec = importlib.util.spec_from_file_location('m', MON)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
import serial  # noqa: E402

RTOD = 57.29577951
DTOR = 1.0 / RTOD

ARRIVE_EPS = 1.5        # 与固件 usb_pc_write_eps_deg 对齐
BAND = ARRIVE_EPS + 0.8  # 判定"落在边界上"的容差
SOFT_INSET = 3.0         # 软限位相对实测行程的内缩量

# ---- 实测机械行程（OctoLink 读 total_angle 两端所得，用于硬性拦截）----
MECH = [(-90.0, 90.0), (-170.0, 0.0), (0.0, 180.0),
        (-180.0, 180.0), (-25.0, 90.0), (-180.0, 180.0)]
# ---- 固件里的软限位默认值（内缩 3°）----
SOFT = [(-87.0, 87.0), (-167.0, -3.0), (3.0, 177.0),
        (-177.0, 177.0), (-22.0, 87.0), (-177.0, 177.0)]

COUNTER_NAMES = ['usb_pc_write_stall_cnt', 'usb_pc_stall_unload_cnt',
                 'usb_pc_stall_retry_cnt', 'usb_pc_write_stall_skip_cnt',
                 'usb_pc_write_cnt', 'usb_pc_rx_cnt', 'usb_pc_consist_err_cnt']

RESULTS = []


def rec(name, ok, detail):
    RESULTS.append((name, ok, detail))
    print('  %s  %s' % ('✅ PASS' if ok else '❌ FAIL', detail))


# ================= OctoLink 会话（读计数 / 改写软限位）=================
class Octo:
    """★ 每次操作都开一个【独立会话】：takeover → 操作 → continue → release → close。

    为什么不复用一条长会话（实测踩过一次，值得记住）：
      · debug lease 只有 **300 s**（实测 `expiresAtMs - acquiredAtMs = 300000`）
      · lease 一过期，`gdb_read_expressions_safe` 会**跳过 resumeAfterRead**，
        且 `target_continue` 也被门禁拒绝 ⇒ **CPU 留在 halt 状态**
      · CPU 一停，固件不再取 USB CDC 数据 ⇒ Windows 上 `ser.write` **永久阻塞**
        （pyserial 的 timeout 只管读，不管写）
      ⇒ 表现为整个回归脚本在 ④ 阶段凭空冻结 6 分钟，且软限位没恢复。
      独立会话下每次操作都持有新鲜 lease，最长只差 300 s 也不会撞上。
    """

    def __init__(self, want=True):
        self.enabled = False
        self.oc = None
        if not want:
            return
        try:
            s = importlib.util.spec_from_file_location('oc', OCTO)
            self.oc = importlib.util.module_from_spec(s)
            s.loader.exec_module(self.oc)
            self.enabled = True
        except Exception as e:
            print('   （OctoLink 不可用：%s）' % type(e).__name__)

    def _session(self, fn):
        """开独立会话执行 fn(bridge)；无论成败都 continue + release + close。"""
        if not self.enabled:
            return None
        b = None
        try:
            b = self.oc.Bridge(timeout=12)
            b.open()
            t = self.oc.result_text(b.call('debug_session_takeover',
                                           {'authorization': 'allow_halt', 'confirm': True}))
            if '"success": true' not in t and 'leaseId' not in t:
                return None
            try:
                return fn(b)
            finally:
                try:
                    b.call('target_continue', {'confirm': True})
                except Exception:
                    pass
                try:
                    b.call('release_debug_lease', {})
                except Exception:
                    pass
        except Exception as e:
            print('   （OctoLink 会话失败：%s）' % type(e).__name__)
            return None
        finally:
            if b is not None:
                try:
                    b.close()
                except Exception:
                    pass

    def read(self, names):
        def fn(b):
            import json
            r = b.call('gdb_read_expressions_safe',
                       {'expressions': list(names), 'haltIfRunning': True,
                        'resumeAfterRead': True, 'timeoutMs': 9000})
            txt = self.oc.result_text(r)
            return {it['expression']: it.get('value')
                    for it in json.loads(txt).get('results', [])}
        try:
            out = self._session(fn)
        except Exception as e:
            print('   （读变量失败：%s）' % type(e).__name__)
            return {}
        return out or {}

    def write(self, expr, val):
        def fn(b):
            w = self.oc.result_text(b.call('debug_write_and_verify',
                                           {'expression': expr, 'value': str(val),
                                            'confirm': True, 'timeoutMs': 6000}))
            return ('"success": true' in w) or ('error' not in w.lower())
        try:
            return bool(self._session(fn))
        except Exception as e:
            print('   （写变量失败：%s）' % type(e).__name__)
            return False

    def ensure_running(self):
        """兜底：若目标被留在 halt，放跑它（否则后面的串口写会永久阻塞）。"""
        def fn(b):
            import json
            txt = self.oc.result_text(b.call('get_gdb_status', {}))
            try:
                st = json.loads(txt).get('target_state')
            except Exception:
                st = None
            return st
        st = self._session(fn)
        return st

    def close(self):
        pass


def fnum(v):
    """GDB 返回值转 float；失败返回 None"""
    try:
        return float(str(v).split()[0])
    except Exception:
        return None


# ================= 串口链路 =================
class Link:
    def __init__(self, port):
        # ★ timeout 取 20ms（< 上报周期 16ms 的一倍多）：这样 drain(0.02) 这种
        #   短窗口也能做多次读尝试，不会因为"一次 50ms 阻塞读没等到数据"就误判无帧。
        #   （实测踩到：timeout=0.05 时 ⑥ 收尾的 drain(0.02) 返回 None，
        #     导致收尾循环第一轮就 break，机械臂没回基准姿态。）
        self.ser = serial.Serial(port, 115200, timeout=0.02)
        self.p = m.StreamParser()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def drain(self, secs):
        """收 secs 秒，返回 (末尾角列表, 末尾使能位, 帧数, 累计校验失败数)"""
        rows = []
        t0 = time.time()
        while time.time() - t0 < secs:
            d = self.ser.read(4096)
            if d:
                for nid, pl in self.p.feed(d):
                    if nid == 0x60 and len(pl) == 25:
                        rows.append(m.decode_0x60(pl))
        if not rows:
            return None, None, 0, self.p.bad
        return [v * RTOD for v in rows[-1][0]], rows[-1][1], len(rows), self.p.bad

    def drain_ok(self, secs, retry=3, gap=0.25):
        """要一帧读数：短暂无帧时重试（供收尾等必须拿到读数的场合）。"""
        for _ in range(retry):
            cur, en, nf, bad = self.drain(secs)
            if cur is not None:
                return cur, en, nf, bad
            time.sleep(gap)
        return None, None, 0, self.p.bad

    def send(self, frame, secs, rate):
        n = 0
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ser.write(frame)
            self.ser.flush()
            n += 1
            time.sleep(rate)
        return n

    def target(self, degs, secs, rate):
        return self.send(m.make_target_frame([d * DTOR for d in degs]), secs, rate)

    def target_two(self, d1, secs1, d2, secs2, rate):
        """★ 连续发送两段目标，中间【不停发】，返回 [(t, [6 路角])] 轨迹。

        为什么必须不停发（实测踩到）：只要中间停发一下（>200ms 超时），
        固件就会把"本轮持续命令"结束掉（§14.16 的 `stall_round_end`），
        下一帧命令于是走【新一轮重新锚定】路径 —— 它会**静默解除堵转锁存**
        （`s_stalled=0` 但不计 `retry_cnt`）⇒ 看起来"目标一变就恢复"，
        却**测不到 retry 重试机制本身**。要测 retry，目标必须在一轮连续命令内变化。
        """
        f1 = m.make_target_frame([d * DTOR for d in d1])
        f2 = m.make_target_frame([d * DTOR for d in d2])
        trace = []
        t0 = time.time()
        last = t0
        while time.time() - t0 < secs1 + secs2:
            if time.time() - last >= rate:
                self.ser.write(f1 if (time.time() - t0) < secs1 else f2)
                self.ser.flush()
                last = time.time()
            d = self.ser.read(4096)
            if d:
                for nid, pl in self.p.feed(d):
                    if nid == 0x60 and len(pl) == 25:
                        trace.append((time.time() - t0,
                                      [v * RTOD for v in m.decode_0x60(pl)[0]]))
        return trace

    def enable(self, on):
        self.send(m.make_enable_frame(on), 0.05, 0.03)
        time.sleep(0.35)

    def hold(self):
        self.send(m.make_hold_frame(), 0.05, 0.03)
        time.sleep(0.35)


def guard(degs, tag, JI=None, tol_other=1.0, tol_self=0.01):
    """拦截超出实测机械行程的目标。返回 True 表示放行。

    ★ 非被试关节要放宽容差：它们的"目标"就是把【当前读数】原样回填
      （差值 0 < eps ⇒ 固件根本不会动它们），而复位基准本身可能带 ±0.1° 噪声。
      实测踩到：J2 复位后读 +0.09°，恰好落在机械行程端点 0 的外侧，
      严格判界会把整个测试帧否掉（③④⑤⑥ 全 FAIL）。
      被试关节仍按 0.01° 严格判 —— 那一路才是真会被驱动的。
    """
    for i, d in enumerate(degs):
        lo, hi = MECH[i]
        tol = tol_self if (JI is None or i == JI) else tol_other
        if d < lo - tol or d > hi + tol:
            print('  ⛔ %s：J%d 目标 %+.2f° 超出实测机械行程 [%+.0f, %+.0f]（容差 %.2f°）—— 拒绝执行'
                  % (tag, i + 1, d, lo, hi, tol))
            return False
    return True


def pick_tighten(cur, JI, step_deg=15.0, req_deg=45.0, mech_margin=25.0):
    """选一个"可收紧的边界"：把软上限/下限收紧到离机械限位足够远处。

    返回 (side, expr, tightened_val, request_val) 或 None。
    side = 'max' → 收紧软上限；'min' → 收紧软下限。
    """
    lo, hi = MECH[JI]
    slo, shi = SOFT[JI]
    c = cur[JI]

    # 收紧上界：新上界必须比原软上界更紧、离机械上界足够远、且当前值在其内
    tmax = c + step_deg
    rmax = c + req_deg
    if tmax <= shi and tmax <= hi - mech_margin and c <= tmax and lo <= rmax <= hi:
        return ('max', 'usb_pc_soft_max_deg[%d]' % JI, tmax, rmax)

    # 收紧下界
    tmin = c - step_deg
    rmin = c - req_deg
    if tmin >= slo and tmin >= lo + mech_margin and c >= tmin and lo <= rmin <= hi:
        return ('min', 'usb_pc_soft_min_deg[%d]' % JI, tmin, rmin)

    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port')
    ap.add_argument('--joint', type=int, default=5, help='被试关节 1~6（默认 J5）')
    ap.add_argument('--rate', type=float, default=0.03, help='下发周期(秒)')
    ap.add_argument('--idle', type=float, default=1.5, help='③ 验收前先静置多久(秒)')
    ap.add_argument('--step', type=float, default=5.0, help='③ 固定目标位移(度)')
    ap.add_argument('--max-travel', type=float, default=30.0, help='单次允许的最大行程(度)')
    ap.add_argument('--overshoot', type=float, default=2.0,
                    help='无 OctoLink 时的退化超程量(<%.0f°，否则越出机械行程)' % SOFT_INSET)
    ap.add_argument('--no-hardening', action='store_true',
                    help='③ 不做"先发 0x63 污染 ref_ms"的严格化')
    ap.add_argument('--skip-softlimit', action='store_true')
    ap.add_argument('--skip-brake', action='store_true')
    ap.add_argument('--disable-at-end', action='store_true',
                    help='收尾时失能（默认保持使能，避免重力塌落）')
    ap.add_argument('--octo', action='store_true',
                    help='读固件计数器（③ 的 stall_cnt 判据与 ④ 的收紧都依赖它）')
    args = ap.parse_args()

    JI = args.joint - 1
    if not 1 <= args.joint <= 6:
        print('关节号必须 1~6')
        return 2

    print('=' * 88)
    print('主臂控制器 · 一键回归   端口=%s   被试关节=J%d' % (args.port, args.joint))
    print('=' * 88)

    lk = None
    oc = None
    base = None
    saved_bound = None      # (expr, 原值) 供 finally 恢复
    try:
        try:
            lk = Link(args.port)
        except Exception as e:
            print()
            print('❌ 打开串口 %s 失败：%s' % (args.port, e))
            try:
                from serial.tools import list_ports
                print('   当前可用串口：')
                for p in list_ports.comports():
                    print('     %-8s %-26s %s' % (p.device, p.description[:26], p.hwid[:44]))
            except Exception:
                pass
            return 1

        if args.octo:
            oc = Octo()
            if not oc.enabled:
                print('   ⚠️ OctoLink 不可用 ⇒ ③ 只判"是否动作"、④ 退化用 ±%.1f° 超程'
                      % args.overshoot)

        # ---------- 前置：读基准 ----------
        cur, en, nf, bad = lk.drain(0.6)
        if cur is None:
            print('❌ 0.6s 内没收到 0x60 帧 —— 检查串口/固件/是否已烧录本次固件')
            return 1
        base = list(cur)
        print('  基准姿态 : ' + '  '.join('J%d=%+.2f' % (i + 1, cur[i]) for i in range(6)))
        print('  使能位   : %d' % en)
        if en != 1:
            print('  → 发 0x62 使能（机械臂会后锁死）')
            lk.enable(1)
            cur, en, nf, bad = lk.drain(0.5)
            base = list(cur)
            print('  重读使能 : %d' % en)

        # ================= ① 上行链路 =================
        print()
        print('【① 上行链路 0x60】采集 3s…')
        cur3, en3, nf3, bad3 = lk.drain(3.0)
        fps = nf3 / 3.0
        ok1 = (fps >= 45.0) and (bad3 == 0) and (cur3 is not None)
        rec('①', ok1, '帧率 %.1f fps(≥45)  校验失败 %d(=0)  帧数 %d' % (fps, bad3, nf3))
        base = list(cur3)

        # ================= ② 下行 0x62 =================
        print()
        print('【② 下行 0x62 使能/失能】')
        lk.enable(0)
        _, en_off, _, _ = lk.drain(0.4)
        lk.enable(1)
        _, en_on, _, _ = lk.drain(0.4)
        ok2 = (en_off == 0) and (en_on == 1)
        rec('②', ok2, '发 0(失能)→使能位 %s ；发 1(使能)→使能位 %s' % (en_off, en_on))

        # ================= ③ episode 锚点（核心）=================
        print()
        print('【③ episode 锚点：0x63 污染基准 → 静置 %.1fs → 发【固定目标】】' % args.idle)
        print('    修复前症状：该场景被当帧判堵转 ⇒ 位移 0.000°、stall_cnt +1')
        cur, _, _, _ = lk.drain(0.4)
        base3 = list(cur)
        tgt = list(base3)
        tgt[JI] = base3[JI] + args.step
        tgt[JI] = max(SOFT[JI][0], min(SOFT[JI][1], tgt[JI]))   # 别越软限位
        clamped = abs(tgt[JI] - (base3[JI] + args.step)) > 0.01

        if not guard(tgt, '③', JI):
            rec('③', False, '目标越界，已跳过')
        else:
            if not args.no_hardening:
                lk.hold()          # ★ 关键：把六路 s_stall_ref_ms 写成当前时刻
            c0 = oc.read(COUNTER_NAMES) if (oc and oc.enabled) else {}
            time.sleep(args.idle)                  # ★ 空闲 >600ms（stall 窗口）
            lk.target(tgt, 1.8, args.rate)         # 固定目标、不做插值
            cur3b, _, _, _ = lk.drain_ok(0.6)
            moved = cur3b[JI] - base3[JI]
            ok3 = abs(moved) >= 0.5 * abs(args.step) - 0.3

            d_stall = d_unl = d_retry = None
            if c0:
                c1 = oc.read(COUNTER_NAMES)
                d_stall = fnum(c1.get('usb_pc_write_stall_cnt'))
                d_stall = None if (d_stall is None or fnum(c0.get('usb_pc_write_stall_cnt')) is None) \
                    else int(d_stall - fnum(c0.get('usb_pc_write_stall_cnt')))
                d_unl = fnum(c1.get('usb_pc_stall_unload_cnt'))
                d_unl = None if (d_unl is None or fnum(c0.get('usb_pc_stall_unload_cnt')) is None) \
                    else int(d_unl - fnum(c0.get('usb_pc_stall_unload_cnt')))
                d_retry = fnum(c1.get('usb_pc_stall_retry_cnt'))
                d_retry = None if (d_retry is None or fnum(c0.get('usb_pc_stall_retry_cnt')) is None) \
                    else int(d_retry - fnum(c0.get('usb_pc_stall_retry_cnt')))
                if d_stall not in (None, 0):
                    ok3 = False
                if d_unl not in (None, 0):
                    ok3 = False

            if not ok3 and abs(moved) < 0.5 and (oc and oc.enabled):
                # 位移≈0 的典型原因：该路被堵转锁存（s_stalled=1）。
                # 判据：s_stalled=1 且 s_stall_ep 仍为 1（PC 停发时没走到"清 ep"）⇒
                #       下一轮没有 0→1 沿 ⇒ 沿用陈旧基准当帧误判 ⇒ 锁存后固定目标
                #       又不满足"目标变化 ≥3°"⇒ 永不重试。需要 a37b029 之后的修复。
                st = oc.read(['s_stalled', 's_stall_ep', 's_stall_ref_ms', 's_stall_ms'])
                print('    ↳ 自诊断：s_stalled=%s  s_stall_ep=%s' %
                      (st.get('s_stalled'), st.get('s_stall_ep')))
                print('              s_stall_ref_ms=%s  s_stall_ms=%s'
                      % (st.get('s_stall_ref_ms'), st.get('s_stall_ms')))
                print('              若该路 s_stalled=1 且 s_stall_ep=1 ⇒ 是"ep 残留"'
                      '导致的堵转误锁存（不是机械问题）')

            extra = ''
            if d_stall is not None:
                extra = '；stall_cnt Δ=%d、unload Δ=%d、retry Δ=%d（都应为 0）' % (
                    d_stall, d_unl, d_retry)
            elif not (oc and oc.enabled):
                extra = '（未开 --octo，只判"是否动作"）'
            rec('③', ok3, '固定目标 %+.1f°%s → 实际位移 %+.3f°（期望 ≈%+.1f°）%s'
                % (tgt[JI], '（被软限位钳到 %.1f°）' % tgt[JI] if clamped else '',
                   moved, args.step, extra))

        # ================= ④ 软限位 =================
        if not args.skip_softlimit:
            print()
            print('【④ 软限位：在线收紧边界 → 发远超收紧值的目标 → 应停在边界】')
            cur, _, _, _ = lk.drain(0.4)
            tight = pick_tighten(cur, JI) if (oc and oc.enabled) else None

            if tight:
                side, expr, tval, req = tight
                if not oc.write(expr, tval):
                    rec('④', False, '写入收紧值失败（%s = %.1f）' % (expr, tval))
                else:
                    saved_bound = (expr, SOFT[JI][1] if side == 'max' else SOFT[JI][0])
                    got = fnum(oc.read([expr]).get(expr))
                    print('    收紧 %s：%.1f → %.1f（读回 %s）'
                          % ('软上限' if side == 'max' else '软下限',
                             saved_bound[1], tval, got))
                    if got is None or abs(got - tval) > 0.05:
                        oc.write(expr, saved_bound[1])
                        saved_bound = None
                        rec('④', False, '收紧值读回不一致（%s），已恢复原值' % got)
                    else:
                        travel = abs(tval - cur[JI])
                        tgt4 = list(cur)
                        tgt4[JI] = req
                        if not guard(tgt4, '④', JI):
                            rec('④', False, '请求值越出机械行程，已跳过')
                        else:
                            # 发足够久：travel 15° 按 ~5°/s 估，给足余量
                            lk.target(tgt4, 2.5 + travel / 4.0, args.rate)
                            fin, _, _, _ = lk.drain_ok(0.6)
                            err = abs(fin[JI] - tval)
                            others = max(abs(fin[i] - cur[i])
                                         for i in range(6) if i != JI)
                            # 恢复原边界（写回后电流位置可能已在新边界外，不影响静止）
                            oc.write(expr, saved_bound[1])
                            rb = fnum(oc.read([expr]).get(expr))
                            restored = (rb is not None and abs(rb - saved_bound[1]) < 0.05)
                            saved_bound = None
                            ok4 = (err <= BAND) and (others <= ARRIVE_EPS + 0.5) and restored
                            rec('④', ok4,
                                '请求 %+.1f°（超收紧值 %.0f°）→ 落点 %+.3f°，'
                                '收紧边界 %+.1f°，差 %.2f°；其余五路最大 %.2f°；'
                                '边界已恢复=%s'
                                % (req, abs(req - tval), fin[JI], tval, err, others,
                                   '✅' if restored else '❌'))
            else:
                # 退化路径：只发"软边界外 overshoot°"，判据弱（与 eps 同量级）
                lo, hi = SOFT[JI]
                use_lo = abs(cur[JI] - lo) <= abs(cur[JI] - hi)
                bound = lo if use_lo else hi
                req = (bound - args.overshoot) if use_lo else (bound + args.overshoot)
                travel = abs(bound - cur[JI])
                if travel > args.max_travel:
                    rec('④', False, '行程 %.1f° > --max-travel %.0f°，已跳过'
                        % (travel, args.max_travel))
                else:
                    tgt4 = list(cur)
                    tgt4[JI] = req
                    if not guard(tgt4, '④', JI):
                        rec('④', False, '请求值越出机械行程，已跳过'
                            '（软限位内缩仅 %.0f°，请开 --octo 用收紧法）' % SOFT_INSET)
                    else:
                        lk.target(tgt4, 1.0 + travel / 6.0 + 1.0, args.rate)
                        fin, _, _, _ = lk.drain_ok(0.6)
                        err = abs(fin[JI] - bound)
                        others = max(abs(fin[i] - cur[i]) for i in range(6) if i != JI)
                        ok4 = (err <= BAND) and (others <= ARRIVE_EPS + 0.5)
                        rec('④', ok4, '（弱判据：未收紧，仅超程 %.1f°）落点 %+.3f°，'
                                       '软边界 %+.1f°，差 %.2f°；其余五路最大 %.2f°'
                            % (args.overshoot, fin[JI], bound, err, others))

        # ================= ⑤ 0x63 刹车 =================
        if not args.skip_brake:
            print()
            print('【⑤ 0x63 刹车：运动中发 0x63，位移应当场归零】')
            cur, _, _, _ = lk.drain(0.4)
            lo, hi = SOFT[JI]
            direction = 1.0 if abs(hi - cur[JI]) > abs(cur[JI] - lo) else -1.0
            tgt5 = list(cur)
            tgt5[JI] = cur[JI] + direction * 20.0
            tgt5[JI] = max(lo, min(hi, tgt5[JI]))
            if not guard(tgt5, '⑤', JI):
                rec('⑤', False, '目标越界，已跳过')
            else:
                lk.target(tgt5, 0.8, args.rate)          # 先跑 0.8s，确保它确实"在飞"
                lk.hold()                                # ★ 立即发 0x63 刹车
                # ★ 先给 0.5s 让"刹车前已在途"的余量落定，再取锚点。
                #   原实现把锚点取在 hold() 【之前】，而 hold() 内部要先发帧再
                #   sleep 0.35s ⇒ 把刹车生效前 0.4s 的在途运动也算进"刹车后位移"，
                #   实测得 0.653° 被误判 FAIL（真值应看刹车生效之后的位移）。
                time.sleep(0.5)
                at, _, _, _ = lk.drain_ok(0.06)
                time.sleep(0.9)
                after, _, _, _ = lk.drain_ok(0.06)
                if at is None or after is None:
                    rec('⑤', False, '取不到上行帧')
                else:
                    d_after = abs(after[JI] - at[JI])
                    inflight = abs(at[JI] - cur[JI])     # 刹车生效时它已经走了多少
                    ok5 = (d_after <= 0.3) and (inflight >= 1.0)
                    rec('⑤', ok5,
                        '刹车生效时 J%d 已走 %+.2f°（目标 %+.1f°）→ 之后 0.9s 位移 %.3f°'
                        '（应 ≈0；<1.0° 说明"没在飞"、判据无意义）'
                        % (args.joint, inflight, tgt5[JI] - cur[JI], d_after))

        # ================= ⑦ 堵转保护三件套（判堵转 → 卸载 → 目标变化后重试）=================
        print()
        print('【⑦ 堵转保护：连续命令流里目标不可达 → 判堵转+卸载；目标再变 ≥3° → 放行重试】')
        print('    ⚠️ 两段之间【不能停发】—— 停发会走"新一轮命令重新锚定"的静默解锁路径，'
              '就测不到 retry 机制了')
        cur, _, _, _ = lk.drain(0.4)
        tight = pick_tighten(cur, JI, step_deg=8.0, req_deg=40.0) if cur else None
        t1 = None
        if cur is not None:
            t1 = list(cur)
            t1[JI] = tight[3] if tight else cur[JI]
        if not (oc and oc.enabled):
            rec('⑦', False, '需要 --octo 才能读到堵转计数')
        elif cur is None:
            rec('⑦', False, '取不到上行帧')
        elif tight is None:
            rec('⑦', False, '该姿态下找不到可收紧的边界，已跳过')
        elif not guard(t1, '⑦', JI):
            rec('⑦', False, '目标越界，已跳过')
        else:
            side, expr, tval, req = tight
            orig = SOFT[JI][1] if side == 'max' else SOFT[JI][0]
            got = None
            if oc.write(expr, tval):
                got = fnum(oc.read([expr]).get(expr))
            if got is None or abs(got - tval) > 0.05:
                oc.write(expr, orig)
                rec('⑦', False, '收紧值读回异常（%s），已恢复' % got)
            else:
                saved_bound = (expr, orig)
                # 第2段目标：变化 ≥3°（此处 ≈37°）且落在可达范围内
                t2 = list(t1)
                t2[JI] = (tval - 5.0) if side == 'max' else (tval + 5.0)
                c0 = oc.read(COUNTER_NAMES)
                trace = lk.target_two(t1, 3.0, t2, 2.5, args.rate)   # ★ 连续，不断流
                c1 = oc.read(COUNTER_NAMES)

                def dlt(a, b, k):
                    x, y = fnum(a.get(k)), fnum(b.get(k))
                    return None if (x is None or y is None) else int(y - x)

                d_stall = dlt(c0, c1, 'usb_pc_write_stall_cnt')
                d_unl = dlt(c0, c1, 'usb_pc_stall_unload_cnt')
                d_skip = dlt(c0, c1, 'usb_pc_write_stall_skip_cnt')
                d_retry = dlt(c0, c1, 'usb_pc_stall_retry_cnt')
                peak = max((a[JI] for _, a in trace), default=None)
                fin = trace[-1][1][JI] if trace else None
                ok_peak = (peak is not None) and (abs(peak - tval) <= 2.5)
                ok_fin = (fin is not None) and (abs(fin - t2[JI]) <= 2.0)
                ok7 = (d_stall == 1) and (d_unl == 1) and (d_skip or 0) > 0 \
                      and (d_retry or 0) >= 1 and ok_peak and ok_fin
                rec('⑦', ok7,
                    '①不可达段：stall Δ=%s(应1) unload Δ=%s(应1) skip Δ=%s(应>0)、'
                    '最高到 %s°（收紧值 %+.1f°）；②改目标段：retry Δ=%s(应≥1)、'
                    '末角 %s°（目标 %+.1f°）'
                    % (d_stall, d_unl, d_skip,
                       ('%+.2f' % peak) if peak is not None else '?', tval,
                       d_retry, ('%+.2f' % fin) if fin is not None else '?', t2[JI]))
                oc.write(expr, orig)
                rb = fnum(oc.read([expr]).get(expr))
                saved_bound = None
                print('    （边界已恢复 %s = %s）' % (expr, rb))

        # ================= ⑧ 200ms 超时门 =================
        print()
        print('【⑧ 200ms 超时门：PC 停发后不得再驱动（但舵机仍会走完最后写入的目标）】')
        cur, _, _, _ = lk.drain(0.4)
        if cur is None:
            rec('⑧', False, '取不到上行帧')
        else:
            lo, hi = SOFT[JI]
            direction = 1.0 if abs(hi - cur[JI]) > abs(cur[JI] - lo) else -1.0
            t8 = list(cur)
            t8[JI] = max(lo, min(hi, cur[JI] + direction * 20.0))
            if not guard(t8, '⑧', JI):
                rec('⑧', False, '目标越界，已跳过')
            else:
                lk.target(t8, 1.0, args.rate)              # 送 1s
                a0, _, _, _ = lk.drain_ok(0.05)            # 停发那一刻
                c1 = oc.read(COUNTER_NAMES) if (oc and oc.enabled) else {}
                time.sleep(0.5)                            # 让 200ms 超时生效
                c2 = oc.read(COUNTER_NAMES) if (oc and oc.enabled) else {}
                time.sleep(1.1)
                c3 = oc.read(COUNTER_NAMES) if (oc and oc.enabled) else {}
                a1, _, _, _ = lk.drain_ok(0.1)
                time.sleep(0.4)
                a2, _, _, _ = lk.drain_ok(0.1)
                d_tail = abs(a1[JI] - a0[JI]) if (a0 and a1) else float('nan')
                d_settle = abs(a2[JI] - a1[JI]) if (a1 and a2) else float('nan')
                d_write = None
                if c2 and c3:
                    x, y = fnum(c2.get('usb_pc_write_cnt')), fnum(c3.get('usb_pc_write_cnt'))
                    d_write = None if (x is None or y is None) else int(y - x)
                lk.hold()                                   # 刹车收尾
                ok8 = (d_settle <= 0.3) and (d_write in (None, 0))
                rec('⑧', ok8,
                    '停发后走完"最后写入的目标"追加 %.3f°（受单帧 10° 上限约束）、'
                    '末段 0.4s 位移 %.3f°（应 ≈0）；超时【之后】的新增写入 Δ=%s（应为 0）'
                    % (d_tail, d_settle, d_write))

        # ================= ⑨ 收尾 =================
        print()
        print('【⑨ 收尾】恢复到基准姿态附近 + 刹车')
        cur, _, _, _ = lk.drain(0.4)
        back = list(base)
        if cur is None or not guard(back, '⑥', JI):
            back = list(cur or base)
        for _ in range(15):                        # 分步回，避免一步太大
            now, _, _, _ = lk.drain_ok(0.3)
            if now is None:
                print('  ⚠️ 连续取不到 0x60 帧，停止回位（先刹车）')
                break
            d = back[JI] - now[JI]
            if abs(d) < ARRIVE_EPS:
                break
            step = max(-10.0, min(10.0, d))
            t = list(now)
            t[JI] = now[JI] + step
            lk.target(t, 0.4, args.rate)           # ★ 送足 0.4s，让舵机真走一段
        lk.hold()
        fin, en_f, _, _ = lk.drain_ok(0.6)
        if fin is None:
            print('  ⚠️ 收尾取不到上行帧')
            fin = cur or back
        print('  最终姿态 : ' + '  '.join('J%d=%+.2f' % (i + 1, fin[i]) for i in range(6)))
        print('  回位残差 : J%d 距基准 %+.3f°' % (args.joint, fin[JI] - back[JI]))
        print('  使能位   : %s' % en_f)
        if args.disable_at_end:
            lk.enable(0)
            print('  已发 0x62 失能（机械臂会因重力塌落）')
        else:
            print('  保持使能（如需放开：python tools/pc_arm_monitor.py %s --enable 0）' % args.port)

        if oc and oc.enabled:
            cF = oc.read(COUNTER_NAMES)
            if cF:
                print()
                print('  固件计数（收尾时）：')
                for k in COUNTER_NAMES:
                    print('    %-32s = %s' % (k, cF.get(k)))

    except KeyboardInterrupt:
        print('\n  用户中断')
    except Exception:
        print('\n  异常：')
        traceback.print_exc()
    finally:
        try:
            if saved_bound is not None:      # ★ 软限位一定要还原
                oc.write(saved_bound[0], saved_bound[1])
                print('  （已恢复 %s = %s）' % saved_bound)
        except Exception:
            pass
        try:
            lk.hold()          # 无论如何先刹车
        except Exception:
            pass
        try:
            lk.close()
        except Exception:
            pass
        try:
            if oc:
                oc.close()
        except Exception:
            pass

    print()
    print('=' * 88)
    print('汇总')
    print('=' * 88)
    nfail = 0
    for name, ok, detail in RESULTS:
        print('  %-4s %s  %s' % (name, 'PASS' if ok else 'FAIL', detail))
        if not ok:
            nfail += 1
    print()
    print('  共 %d 项，失败 %d 项' % (len(RESULTS), nfail))
    return 1 if nfail else 0


if __name__ == '__main__':
    sys.exit(main())
