#!/usr/bin/env python3
"""Compares two roomba_driver runs of tools/ab_driver.sh (e.g. the Python and the Rust driver on the
same fake_roomba.py scenario): commands written to the serial port (byte for byte, in order, and
their timing), the shutdown bytes, stream layout requests and their frame sizes, roomba/mode,
service responses, odometry / joint states at stationary checkpoints, /tf, battery_state,
roomba/sensors value sequences and the log lines. Also reports how long each light-bumper stop took
(scripted hit -> stop command on the serial port) and the drive_direct steps of each bump reflex.

  tools/ab_driver_compare.py RUN_A RUN_B       (directories with serial.jsonl, harness.json, driver.log)
"""
import difflib
import json
import math
import os
import re
import statistics
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_roomba import FMT, LIGHT_HITS, command_length, ids_of  # noqa: E402

LAYOUTS = {}   # stream request bytes -> 'primary' / 'secondary' (the driver's first two layouts)
STOP = bytes([145, 0, 0, 0, 0])
CHECKPOINTS = [1.9, 6.0, 9.0, 13.5, 16.0, 29.9, 54.9, 58.4, 70.0]   # robot standing still
NAMES = {7: 'reset', 128: 'start', 131: 'safe', 132: 'full', 134: 'spot', 137: 'drive', 138: 'motors', 139: 'leds',
         140: 'song', 141: 'play', 144: 'pwm_motors', 145: 'drive_direct', 146: 'drive_pwm', 148: 'stream',
         150: 'pause/resume', 162: 'sched_leds', 163: 'digits_raw', 164: 'digits_ascii', 165: 'buttons',
         167: 'schedule', 168: 'day_time', 173: 'stop'}


def load(d):
    run = {'dir': d}
    run['t0'] = json.load(open(f'{d}/t0.json'))
    run['h'] = json.load(open(f'{d}/harness.json'))
    t0 = run['t0']['t0']
    buf, times, cmds = bytearray(), [], []
    run['scenario'] = 'ab'
    for line in open(f'{d}/serial.jsonl'):
        e = json.loads(line)
        if e['ev'] == 'start':
            run['scenario'] = e.get('scenario', 'ab')
        if e['ev'] != 'rx':
            continue
        for b in bytes.fromhex(e['data']):
            buf.append(b)
            times.append(e['t'] - t0)
        while buf:
            n = command_length(buf)
            if n is None:
                break
            n = n or 1
            cmds.append((times[0], bytes(buf[:n])))
            del buf[:n]
            del times[:n]
    run['cmds'] = cmds
    logs = []
    pat = re.compile(r'^\[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$')
    for line in open(f'{d}/driver.log', errors='replace'):
        m = pat.match(line.rstrip('\n'))
        if m:
            msg = m.group(4).replace(d, '<run>')
            logs.append((float(m.group(2)) - run['t0']['t0_wall'], m.group(1), msg))
        elif line.strip():
            logs.append((None, 'RAW', line.rstrip('\n')))
    run['logs'] = logs
    return run


def show(c):
    op = c[0]
    if c in LAYOUTS:
        return 'stream ' + LAYOUTS[c]
    return f"{NAMES.get(op, op)} {c[1:].hex(' ')}".strip()


def layouts(cmds):
    """The stream layouts the driver requested, in order of their first request (primary first)."""
    seen = []
    for _, c in cmds:
        if c[0] == 148 and c not in seen:
            seen.append(c)
    return seen


def frame_bytes(request):
    """Size of the stream frames for a stream request: header, count, ids, data, checksum."""
    return 3 + sum(1 + sum(struct.calcsize('>' + FMT[i]) for i in ids_of(pid)) for pid in request[2:])


def light_stops(run, kill):
    """(hit onset, seconds until the first stop command, speed) for each scripted light-bumper hit."""
    out = []
    for onset, _, speed in LIGHT_HITS.get(run['scenario'], []):
        stop = next((t for t, c in run['cmds'] if onset <= t < kill and c == STOP), None)
        out.append((onset, stop - onset if stop is not None and stop - onset < 1.0 else None, speed))
    return out


def reflexes(run, kill):
    """(start, reverse commands, turn commands, stopped after) of each bump reflex: a run of equal
    drive_direct reverse commands followed by turn-in-place ones."""
    dd = [(t, *struct.unpack('>hh', c[1:])[::-1]) for t, c in run['cmds'] if c[0] == 145 and t < kill]
    out, i = [], 0
    while i < len(dd):
        t, left, right = dd[i]
        if not left == right < 0:
            i += 1
            continue
        j = i
        while j < len(dd) and dd[j][1:] == (left, right):
            j += 1
        k = j
        while k < len(dd) and dd[k][1] == -dd[k][2] != 0 and dd[k][1:] == dd[j][1:]:
            k += 1
        stop = dd[k][1:] == (0, 0) if k < len(dd) else False
        out.append((t, j - i, k - j, stop))
        i = max(k, i + 1)
    return out


def mask_cmd(t, c):
    # set_day_time with use_system_time (sent at 47.4 s) carries the wall clock.
    if c[0] == 168 and 47.35 < t < 47.9 and c[1:] != bytes([3, 14, 5]):
        return bytes([168]) + b'\xff\xff\xff'
    return c


def compare_seq(name, a, b, key, fmt, limit=40):
    """Prints the differences of two sequences; returns the matched pairs."""
    ka, kb = [key(x) for x in a], [key(x) for x in b]
    sm = difflib.SequenceMatcher(None, ka, kb, autojunk=False)
    pairs, diffs = [], 0
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == 'equal':
            pairs += list(zip(a[i1:i2], b[j1:j2]))
            continue
        diffs += max(i2 - i1, j2 - j1)
        if limit > 0:
            for x in a[i1:i2]:
                print(f'    A only: {fmt(x)}')
            for x in b[j1:j2]:
                print(f'    B only: {fmt(x)}')
            limit -= 1
    status = 'IDENTICAL' if diffs == 0 and len(a) == len(b) else f'{diffs} differing'
    print(f'  {name}: A {len(a)}, B {len(b)}, matched {len(pairs)} -> {status}')
    return pairs


def timing(pairs, t=lambda x: x[0]):
    d = [abs(t(x) - t(y)) for x, y in pairs if t(x) is not None and t(y) is not None]
    if not d:
        return 'n/a'
    return f'|dt| median {statistics.median(d) * 1000:.1f} ms, 95% {sorted(d)[int(0.95 * (len(d) - 1))] * 1000:.1f} ms, max {max(d) * 1000:.1f} ms'


def last_before(msgs, t):
    best = None
    for m in msgs:
        if m['t'] is not None and m['t'] <= t:
            best = m
    return best


def same(x, y, tol=0.0):
    if isinstance(x, float) and isinstance(y, float):
        if math.isnan(x) and math.isnan(y):
            return True
        return abs(x - y) <= tol
    if isinstance(x, list) and isinstance(y, list):
        return len(x) == len(y) and all(same(p, q, tol) for p, q in zip(x, y))
    return x == y


def runs_of(values):
    out = []
    for v in values:
        if not out or out[-1] != v:
            out.append(v)
    return out


def main():
    a, b = load(sys.argv[1]), load(sys.argv[2])
    print(f'A = {a["dir"]}\nB = {b["dir"]}  (scenario {a["scenario"]})')
    ha, hb = a['h'], b['h']
    ok = True
    for c, name in zip(layouts(a['cmds']), ('primary', 'secondary')):
        LAYOUTS[c] = name

    print('\n== serial commands (excluding stream layout requests) ==')
    kill_a, kill_b = ha['sigterm'], hb['sigterm']
    body_a = [c for c in a['cmds'] if c[1][0] != 148 and (kill_a is None or c[0] < kill_a)]
    body_b = [c for c in b['cmds'] if c[1][0] != 148 and (kill_b is None or c[0] < kill_b)]
    pairs = compare_seq('commands', body_a, body_b, lambda c: mask_cmd(*c), lambda c: f'{c[0]:7.3f} {show(c[1])}')
    ok &= len(pairs) == len(body_a) == len(body_b)
    print(f'  timing of matched commands: {timing(pairs)}')
    worst = sorted(pairs, key=lambda p: -abs(p[0][0] - p[1][0]))[:5]
    for x, y in worst:
        print(f'    {x[0]:7.3f} vs {y[0]:7.3f}  {show(x[1])}')
    by_op = {}
    for _, c in body_a:
        by_op[show(c).split(' ')[0]] = by_op.get(show(c).split(' ')[0], 0) + 1
    print('  A command counts:', ', '.join(f'{k} {v}' for k, v in sorted(by_op.items())))

    print('\n== shutdown (SIGTERM) bytes ==')
    for run, kill in ((a, kill_a), (b, kill_b)):
        tail = [show(c) for t, c in run['cmds'] if kill is not None and t >= kill]
        exited = run['h'].get('exited')
        print(f'  {os.path.basename(run["dir"])}: SIGTERM at {kill:.3f} s, exited after '
              f'{(exited - kill) * 1000 if exited else float("nan"):.0f} ms: {tail}')
    # The reader thread keeps rotating the stream layout until the shutdown pauses it.
    tail_a = [c for t, c in a['cmds'] if t >= kill_a and c[0] != 148]
    tail_b = [c for t, c in b['cmds'] if t >= kill_b and c[0] != 148]
    print(f'  shutdown bytes (without stream rotation) {"IDENTICAL" if tail_a == tail_b else "DIFFER"}: '
          f'{b"".join(tail_a).hex(" ")}')
    ok &= tail_a == tail_b

    print('\n== stream layout requests ==')
    for run in (a, b):
        desc = ', '.join(f'{LAYOUTS.get(c, "other")} {list(c[2:])} ({frame_bytes(c)}-byte frames)'
                         for c in layouts(run['cmds']))
        print(f'  {os.path.basename(run["dir"])}: {desc}')
    same_layouts = layouts(a['cmds']) == layouts(b['cmds'])
    print(f'  layouts {"IDENTICAL" if same_layouts else "DIFFER"}')
    ok &= same_layouts
    for run, kill in ((a, kill_a), (b, kill_b)):
        st = [(t, c) for t, c in run['cmds'] if c[0] == 148 and t < kill]
        prim = sum(LAYOUTS.get(c) == 'primary' for _, c in st)
        sec = sum(LAYOUTS.get(c) == 'secondary' for _, c in st)
        span = kill - 0.0
        print(f'  {os.path.basename(run["dir"])}: primary {prim}, secondary {sec} ({sec / span:.2f}/s), '
              f'other {len(st) - prim - sec}')

    if LIGHT_HITS.get(a['scenario']):
        print('\n== light-bumper stops (scripted hit -> stop command received) ==')
        for run, kill in ((a, kill_a), (b, kill_b)):
            stops = light_stops(run, kill)
            lat = [s for _, s, _ in stops if s is not None]
            dist = [s * v * 1000 for _, s, v in stops if s is not None]
            name = os.path.basename(run['dir'])
            if not lat:
                print(f'  {name}: no stop within 1 s of a hit (light-bumper stop off?)')
                continue
            print(f'  {name}: {len(lat)}/{len(stops)} hits stopped; latency median {statistics.median(lat) * 1000:.0f} ms, '
                  f'mean {statistics.mean(lat) * 1000:.0f} ms, max {max(lat) * 1000:.0f} ms; driven meanwhile mean '
                  f'{statistics.mean(dist):.1f} mm, max {max(dist):.1f} mm')
            print('    ' + ' '.join(f'{s * 1000:.0f}' if s is not None else '-' for _, s, _ in stops) + ' ms')

    print('\n== bump reflexes (drive_direct steps) ==')
    for run, kill in ((a, kill_a), (b, kill_b)):
        print(f'  {os.path.basename(run["dir"])}: ' + ('; '.join(
            f'{t:.2f} s reverse x{nr}, turn x{nt}{", stop" if stop else ""}' for t, nr, nt, stop in reflexes(run, kill))
            or 'none'))

    print('\n== roomba/mode ==')
    ma, mb = [m['data'] for m in ha['mode']], [m['data'] for m in hb['mode']]
    # roomba/mode is latched with depth 1: if the first value ('passive') is published before the
    # harness has discovered the driver, only the next one arrives. Not a driver difference.
    if ma[:1] == ['safe'] and mb[:2] == ['passive', 'safe']:
        print("  (A's harness missed the first latched 'passive', published before discovery)")
        mb = mb[1:]
    elif mb[:1] == ['safe'] and ma[:2] == ['passive', 'safe']:
        print("  (B's harness missed the first latched 'passive', published before discovery)")
        ma = ma[1:]
    print(f'  A: {ma}')
    if ma == mb:
        print('  IDENTICAL')
    else:
        print(f'  B: {mb}\n  DIFFER')
        ok = False

    print('\n== services ==')
    sa = [(s['service'], json.dumps(s['request']), json.dumps(s.get('response'))) for s in ha['services']]
    sb = [(s['service'], json.dumps(s['request']), json.dumps(s.get('response'))) for s in hb['services']]
    msk = re.compile(r'clock set to day \d \d\d:\d\d sent')
    def norm(x):
        if x[0] == 'set_day_time' and '"use_system_time": true' in x[1]:
            return (x[0], x[1], msk.sub('clock set to day D HH:MM sent', x[2]))
        return x
    n_same = sum(norm(x) == norm(y) for x, y in zip(sa, sb))
    for s, x, y in zip(ha['services'], sa, sb):
        if norm(x) != norm(y):
            print(f'    {s["at"]:6.1f} {x[0]} {x[1]}\n      A {x[2]}\n      B {y[2]}')
    print(f'  {len(sa)} calls, {n_same} identical responses -> {"IDENTICAL" if n_same == len(sa) == len(sb) else "DIFFER"}')
    ok &= n_same == len(sa) == len(sb)
    for s in ha['services']:
        print(f'    {s["at"]:5.1f} {s["service"]:13s} {json.dumps(s["request"])[:60]:60s} -> {json.dumps(s.get("response"))}')
    rt = [(s['t_done'] - s['t']) for s in ha['services'] if 't_done' in s]
    rtb = [(s['t_done'] - s['t']) for s in hb['services'] if 't_done' in s]
    if rt and rtb:
        print(f'  response time median A {statistics.median(rt) * 1000:.1f} ms, B {statistics.median(rtb) * 1000:.1f} ms')

    print('\n== wheel/odom and joint_states ==')
    for key in ('odom', 'joints', 'tf', 'battery', 'sensors'):
        first = [f'{h[key][0]["t"] or 0:.2f}' if h[key] else '-' for h in (ha, hb)]
        print(f'  {key}: A {len(ha[key])}, B {len(hb[key])} messages (first received at {first[0]} / {first[1]} s)')
    # (lightbump: the scripted wheels never stand still)
    for t in [c for c in CHECKPOINTS if c < min(kill_a, kill_b) and a['scenario'] != 'lightbump']:
        oa, ob = last_before(ha['odom'], t), last_before(hb['odom'], t)
        ja, jb = last_before(ha['joints'], t), last_before(hb['joints'], t)
        if not (oa and ob and ja and jb):
            print(f'    t={t}: missing')
            ok = False
            continue
        keys = ('x', 'y', 'qz', 'qw', 'vx', 'wz', 'frame', 'child', 'pcov', 'tcov')
        eq = all(same(oa[k], ob[k]) for k in keys) and same(ja['position'], jb['position']) \
            and same(ja['velocity'], jb['velocity']) and ja['name'] == jb['name']
        ok &= eq
        print(f'    t={t:5.1f}: x {oa["x"]: .9f} y {oa["y"]: .9f} yaw {2 * math.atan2(oa["qz"], oa["qw"]): .6f} '
              f'wheels {ja["position"][0]: .5f} {ja["position"][1]: .5f}  {"IDENTICAL" if eq else "DIFFER"}')
        if not eq:
            print(f'           B: x {ob["x"]: .9f} y {ob["y"]: .9f} yaw {2 * math.atan2(ob["qz"], ob["qw"]): .6f}')
    va = [o['vx'] for o in ha['odom'] if o['t'] and 3.0 < o['t'] < 5.0]
    vb = [o['vx'] for o in hb['odom'] if o['t'] and 3.0 < o['t'] < 5.0]
    if va and vb:
        print(f'  mean speed 3-5 s: A {statistics.mean(va):.4f} m/s, B {statistics.mean(vb):.4f} m/s')
    if ha['tf'] or hb['tf']:
        ta, tb = ha['tf'][-1], hb['tf'][-1]
        eq = all(same(ta[k], tb[k]) for k in ('parent', 'child', 'x', 'y', 'z', 'qz', 'qw'))
        print(f'  last /tf {ta["parent"]} -> {ta["child"]}: {"IDENTICAL" if eq else "DIFFER"}')
        ok &= eq
    lag = [o['t'] - (o['stamp'] - a['t0']['t0_wall']) for o in ha['odom'] if o['t']]
    lagb = [o['t'] - (o['stamp'] - b['t0']['t0_wall']) for o in hb['odom'] if o['t']]
    print(f'  frame -> odom receive latency median: A {statistics.median(lag) * 1000:.1f} ms, '
          f'B {statistics.median(lagb) * 1000:.1f} ms')

    print('\n== battery_state ==')
    keys = ('voltage', 'current', 'temperature', 'charge', 'capacity', 'design_capacity', 'percentage',
            'power_supply_status', 'power_supply_health', 'power_supply_technology', 'present', 'frame')
    ba, bb = ha['battery'], hb['battery']
    # Paired by the second they arrived in (1 Hz), so a message the harness missed doesn't shift the rest.
    ka = {round(x['t']): x for x in ba if x['t'] is not None}
    kb = {round(x['t']): x for x in bb if x['t'] is not None}
    common = sorted(set(ka) & set(kb))
    bad = [s for s in common if not all(same(ka[s][k], kb[s][k], 1e-6) for k in keys)]
    print(f'  A {len(ba)}, B {len(bb)} messages, {len(common)} paired by stamp; {len(bad)} differ'
          + (f' (first at {bad[0]} s)' if bad else '') + f'; only in A: {sorted(set(ka) - set(kb))}, only in B: {sorted(set(kb) - set(ka))}')
    for s in bad[:5]:
        print(f'    A {ka[s]}\n    B {kb[s]}')
    statuses = runs_of([(x['power_supply_status'], x['power_supply_health']) for x in ba])
    print(f'  A status/health sequence: {statuses}; percentage {ba[0]["percentage"]:.4f} -> {ba[-1]["percentage"]:.4f}')
    ok &= not bad and len(common) >= min(len(ba), len(bb)) - 2

    print('\n== roomba/sensors ==')
    # From 0.5 s: the first samples depend on the timer phase (before the first secondary frame).
    sa = [m for m in ha['sensors'] if m['t'] is not None and m['t'] > 0.5]
    sb = [m for m in hb['sensors'] if m['t'] is not None and m['t'] > 0.5]
    skip = {'t', 'stamp', 'left_encoder_counts', 'right_encoder_counts', 'voltage_mv', 'current_ma',
            'left_motor_current_ma', 'right_motor_current_ma', 'stream_packet_count', 'requested_velocity_mm_s',
            'requested_radius_mm', 'requested_right_velocity_mm_s', 'requested_left_velocity_mm_s', 'stasis',
            'light_bump_front_left_signal', 'temperature_c', 'frame'}
    diff_fields = []
    for k in (sa[0] if sa else {}):
        if k in skip:
            continue
        if runs_of([m[k] for m in sa]) != runs_of([m[k] for m in sb]):
            diff_fields.append(k)
    print(f'  A {len(sa)}, B {len(sb)} messages; value sequences of {len(sa[0]) - len(skip & set(sa[0])) if sa else 0} fields '
          f'compared: {"IDENTICAL" if not diff_fields else "differ in " + ", ".join(diff_fields)}')
    for k in diff_fields[:5]:
        print(f'    {k}: A {runs_of([m[k] for m in sa])}\n    {" " * len(k)}  B {runs_of([m[k] for m in sb])}')
    ok &= not diff_fields
    for k in ('bump_left', 'bump_right', 'wheel_drop_left', 'cliff_front_right', 'home_base_available',
              'charging_state', 'oi_mode', 'button_clean', 'light_bump_front_left'):
        print(f'    {k}: {runs_of([m[k] for m in sa])}')

    print('\n== log lines ==')
    la = [x for x in a['logs'] if x[1] != 'RAW']
    lb = [x for x in b['logs'] if x[1] != 'RAW']
    pairs = compare_seq('log lines', la, lb, lambda x: (x[1], x[2]), lambda x: f'{x[0]:7.3f} [{x[1]}] {x[2]}')
    print(f'  timing: {timing(pairs)}')
    for run in (a, b):
        raw = [x[2] for x in run['logs'] if x[1] == 'RAW']
        if raw:
            print(f'  {os.path.basename(run["dir"])} other output: {raw[:10]}')
    print('  A log:')
    for t, lvl, m in la:
        print(f'    {t:7.3f} [{lvl}] {m}')

    print(f'\nRESULT: {"all compared outputs match" if ok else "differences found (see above)"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
