#!/usr/bin/env python3
"""Simulated Roomba 500 on a pseudo-terminal, for testing roomba_driver without the robot.

Opens a pty pair, links --link to the slave end (give that path to the driver as its port) and
behaves like the Open Interface: Start/Safe/Full/Stop/Reset and the cleaning behaviours change
the OI mode (Safe drops to Passive on a wheel drop, cliff or charger, as on the robot), opcode 148
streams frames [19][n][id][data]...[checksum] every 15 ms, 150 pauses/resumes, 142/149 answer
queries. Every byte the driver writes is logged with its CLOCK_MONOTONIC time (JSON lines).

The sensor values are a deterministic script of the time since the first stream request (t0),
frame by frame (15 ms ticks from t0), so two drivers run against it see the same data:
  --scenario ab      ~70 s: driving, light-bumper and bumper hits, buttons, wheel drop, cliff,
                     corrupted checksums, a split frame, garbage bytes, a 0.8 s stream dropout,
                     4 s asleep (OI off afterwards), docking (seen first in a secondary-layout
                     frame), charging states, a reset with boot text.
                     tools/ab_driver_harness.py sends the matching commands.
  --scenario lightbump  ~42 s driving forward at 0.25 m/s with 16 front light-bumper hits
                     (LIGHT_HITS) at different phases of the stream: tools/ab_driver_compare.py
                     reports how long each stop took and how far the robot drove meanwhile.
  --scenario steady  endless slow driving at the real stream rate (CPU measurements).
  --scenario replug  robot standing still; the port appears only 6 s after start (the driver must
                     retry opening it), and 10 s after t0 the adapter is "unplugged" for 2.5 s (the
                     pty is closed and a new one linked), so the driver has to reconnect. The stream
                     resumes with the driver's first command after that (on a real robot a frame
                     can land between the driver's open and its Start; which driver sees one is
                     down to timing, so the A/B leaves that race out).

  tools/fake_roomba.py --link /data/x/tty --log /data/x/serial.jsonl --t0-file /data/x/t0.json
"""
import argparse
import json
import math
import os
import select
import signal
import struct
import sys
import time
import tty

TICK = 0.015
M_PER_COUNT = math.pi * 0.072 / 508.8
# Front-left light-bumper hits (signal 250): (onset s, duration s, speed the robot drives at, m/s).
LIGHT_HITS = {
    'ab': [(10.0, 0.6, 0.15)],
    'lightbump': [(round(2.0 + 2.5137 * i, 4), 0.4, 0.25) for i in range(16)],
}
DOCK_AT = 36.5      # ab: docking from here on, in the first secondary-layout frame
UNDOCK_AT = 40.5

# ------------------------------------------------------------------ OI tables (as in oi.py)
FMT = {7: 'B', 8: 'B', 9: 'B', 10: 'B', 11: 'B', 12: 'B', 13: 'B', 14: 'B', 15: 'B', 16: 'B', 17: 'B',
       18: 'B', 19: 'h', 20: 'h', 21: 'B', 22: 'H', 23: 'h', 24: 'b', 25: 'H', 26: 'H', 27: 'H', 28: 'H',
       29: 'H', 30: 'H', 31: 'H', 32: 'B', 33: 'H', 34: 'B', 35: 'B', 36: 'B', 37: 'B', 38: 'B', 39: 'h',
       40: 'h', 41: 'h', 42: 'h', 43: 'H', 44: 'H', 45: 'B', 46: 'H', 47: 'H', 48: 'H', 49: 'H', 50: 'H',
       51: 'H', 52: 'B', 53: 'B', 54: 'h', 55: 'h', 56: 'h', 57: 'h', 58: 'B'}
GROUPS = {0: range(7, 27), 1: range(7, 17), 2: range(17, 21), 3: range(21, 27), 4: range(27, 35),
          5: range(35, 43), 6: range(7, 43), 100: range(7, 59), 101: range(43, 59), 106: range(46, 52),
          107: range(54, 59)}
# Command opcode -> number of data bytes (None: variable).
ARGS = {7: 0, 128: 0, 129: 1, 130: 0, 131: 0, 132: 0, 133: 0, 134: 0, 135: 0, 136: 0, 137: 4, 138: 1,
        139: 3, 140: None, 141: 1, 142: 1, 143: 0, 144: 3, 145: 4, 146: 4, 148: None, 149: None, 150: 1,
        162: 2, 163: 4, 164: 4, 165: 1, 167: 15, 168: 3, 173: 0}

BOOT_TEXT = (b'bl-start\r\nSTR730\r\nbootloader id: #x47186549 82ECCFFF\r\nbootloader info rev: #xF000\r\n'
             b'bootloader rev: #x0001\r\n2007-05-14-1715-L   \r\nRoomba by iRobot!\r\nstr730\r\n'
             b'2010-03-12-1430-L   \r\nbattery-current-zero 258\r\n')


def ids_of(pid):
    return list(GROUPS[pid]) if pid in GROUPS else [pid]


def command_length(buf):
    """Bytes in the command at the start of buf, or None if incomplete (0 if unknown opcode)."""
    op = buf[0]
    if op not in ARGS:
        return 0
    n = ARGS[op]
    if n is not None:
        return 1 + n if len(buf) >= 1 + n else None
    if len(buf) < 2:
        return None
    if op == 140:   # song: number, count, 2 * count
        if len(buf) < 3:
            return None
        total = 3 + 2 * buf[2]
    else:           # stream / query list: count, ids
        total = 2 + buf[1]
    return total if len(buf) >= total else None


# ------------------------------------------------------------------ the script
def window(t, a, b):
    return a <= t < b


class Script:
    """Sensor values as a function of the tick index (deterministic)."""

    def __init__(self, scenario):
        self.scenario = scenario
        self.left = 65000.0     # encoder counts; the left one rolls over when driving forward
        self.right = 100.0
        self.k = 0
        self.dock_k = None      # tick of the docking frame

    def docked(self, k):
        return self.dock_k is not None and self.dock_k <= k and k * TICK < UNDOCK_AT

    def wheel_speeds(self, t):
        """Scripted wheel speeds (m/s), roughly what the driver commands at that time."""
        if self.scenario == 'steady':
            return (0.1, 0.12) if int(t) % 20 < 10 else (-0.05, 0.05)
        if self.scenario == 'replug':
            return 0.0, 0.0
        if self.scenario == 'lightbump':
            return 0.25, 0.25
        for a, b, left, right in ((2.2, 5.2, 0.2, 0.2), (6.2, 8.2, -0.1175, 0.1175), (9.2, 10.0, 0.15, 0.15),
                                  (10.0, 10.8, -0.1, -0.1), (10.8, 11.27, 0.1, -0.1), (11.3, 12.5, 0.15, 0.15),
                                  (13.75, 14.55, -0.1, -0.1), (14.55, 15.02, 0.1, -0.1), (55.0, 55.5, 0.1, 0.13),
                                  (56.5, 57.0, 0.08, 0.12)):
            if window(t, a, b):
                return left, right
        return 0.0, 0.0

    def advance(self, k):
        """Move the encoders to tick k (integrating the scripted speeds tick by tick)."""
        while self.k < k:
            self.k += 1
            left, right = self.wheel_speeds(self.k * TICK)
            self.left += left * TICK / M_PER_COUNT
            self.right += right * TICK / M_PER_COUNT

    def values(self, k, mode, requested, layout_n):
        """Raw packet values (id -> int) at tick k."""
        self.advance(k)
        t = k * TICK
        ab = self.scenario == 'ab'
        moving = self.wheel_speeds(t) != (0.0, 0.0)
        sag = ab and any(window(t, a, b) for a, b in ((2.5, 5.5), (6.5, 8.5), (9.5, 12.5), (54.5, 57.5)))
        bump_left = ab and window(t, 13.5, 14.1)
        bump_right = ab and window(t, 15.0, 15.5)
        drop_left = ab and window(t, 17.0, 18.0)
        cliff_fr = ab and window(t, 21.0, 21.6)
        home_base = ab and self.docked(k)
        charging = 0
        if ab and window(t, DOCK_AT, 39.5):
            charging = 2
        elif ab and window(t, 42.5, 44.5):
            charging = 5
        hit = any(window(t, a, a + d) for a, d, _ in LIGHT_HITS.get(self.scenario, ()))
        light_fl = 250 if hit else (40 if moving else 12)
        buttons = 0b11 if ab and window(t, 16.0, 16.6) else 0
        # Battery: 20.2 V, 30 mV lower every 4 s (steps at 4k + 2.5 s), sagging under load.
        volts = 20200 - 30 * int(max(t - 2.5, 0) // 4 + (t >= 2.5)) if ab else 19800
        current = -300
        if sag or self.scenario == 'steady':
            current = -1800
            volts -= 180
        if charging == 2:
            current, volts = 1500, volts + 400
        v = {7: bump_right | bump_left << 1 | drop_left << 3, 8: 0, 9: 0, 10: 0, 11: int(cliff_fr), 12: 0, 13: 0,
             14: 0, 15: 0, 16: 0, 17: 161 if home_base else 0, 18: buttons, 19: 0, 20: 0, 21: charging,
             22: volts, 23: current, 24: 25 if not charging else 31, 25: 2000, 26: 2696, 27: 3,
             28: 2400, 29: 2500, 30: 400 if cliff_fr else 2550, 31: 2450, 32: 0, 33: 0, 34: home_base << 1,
             35: mode, 36: 0, 37: 0, 38: layout_n, 39: requested[0], 40: requested[1], 41: requested[2],
             42: requested[3], 43: int(round(self.left)) % 65536, 44: int(round(self.right)) % 65536,
             45: (1 << 1) if light_fl >= 200 else 0, 46: 5, 47: light_fl, 48: 9, 49: 7, 50: 11, 51: 4,
             52: 0, 53: 0, 54: 120 if moving else 0, 55: 118 if moving else 0, 56: 0, 57: 0,
             58: 1 if moving and self.wheel_speeds(t)[0] > 0 else 0}
        return v

    def safety(self, k):
        """Wheel drop / cliff / charger active at tick k (Safe mode drops to Passive).

        Docking happens in the first frame from DOCK_AT of the driver's secondary stream layout (the
        one without packet 34): the drop to Passive and the charger (packet 34) appear together, but
        the driver sees the drop first and the charger only in the next primary frame. It must not
        take that for a drop without a safety reason and ask for Safe again. (Charging, packet 21,
        starts at DOCK_AT itself so battery_state doesn't depend on the rotation phase.)
        """
        if self.scenario != 'ab':
            return False
        t = k * TICK
        return window(t, 17.0, 18.0) or window(t, 21.0, 21.6) or self.docked(k)

    def asleep(self, t):
        return self.scenario == 'ab' and window(t, 30.0, 34.0)

    def dropout(self, t):
        return self.scenario == 'ab' and window(t, 27.0, 27.8)


# ------------------------------------------------------------------ the robot
class FakeRoomba:

    def __init__(self, args):
        self.link = args.link
        self.master = self.slave = None
        self.open_pty()
        replug = args.scenario == 'replug'
        self.link_at = time.monotonic() + (6.0 if replug else 0.0)
        self.replug_at, self.replug_gap, self.relink_at = (10.0 if replug else None), 2.5, None
        self.hold_stream = False
        if not replug:
            self.make_link()
        self.log = open(args.log, 'w')
        self.t0_file = args.t0_file
        self.script = Script(args.scenario)
        self.duration = args.duration
        self.mode = 0             # OI off until Start
        self.layout = None        # requested stream ids
        self.streaming = False
        self.requested = (0, 0, 0, 0)
        self.buf = bytearray()
        self.t0 = None
        self.k = 0
        self.awake_after_sleep = False
        self.pending = b''        # second half of a split frame
        self.stats = {'frames': 0, 'corrupted': 0, 'rx_bytes': 0, 'commands': 0}
        self._event('start', link=args.link, pty=os.ttyname(self.slave), scenario=args.scenario)

    def open_pty(self):
        self.master, slave = os.openpty()
        tty.setraw(slave)
        self.slave = slave     # kept open: the pty survives the driver closing and reopening it
        os.set_blocking(self.master, False)

    def make_link(self):
        tmp = self.link + '.new'
        if os.path.lexists(tmp):
            os.unlink(tmp)
        os.symlink(os.ttyname(self.slave), tmp)
        os.replace(tmp, self.link)
        self.link_at = None

    def unplug(self, now):
        os.close(self.master)
        os.close(self.slave)
        self.master = self.slave = None
        self.relink_at = now + self.replug_gap
        self.replug_at = None
        self._event('unplugged')

    def _event(self, ev, **kw):
        kw.update(ev=ev, t=time.monotonic())
        self.log.write(json.dumps(kw) + '\n')

    # -------------------------------------------------------------- commands
    def on_bytes(self, data, now):
        self.stats['rx_bytes'] += len(data)
        self._event('rx', data=data.hex())
        if self.t0 is not None and self.script.asleep(now - self.t0):
            return          # asleep: the robot hears nothing
        self.buf += data
        while self.buf:
            n = command_length(self.buf)
            if n is None:
                break
            if n == 0:
                self._event('junk', data=self.buf[:1].hex())
                del self.buf[:1]
                continue
            cmd = bytes(self.buf[:n])
            del self.buf[:n]
            self.stats['commands'] += 1
            self.hold_stream = False
            self.execute(cmd, now)

    def set_mode(self, mode, now):
        if mode != self.mode:
            self.mode = mode
            self._event('mode', mode=mode)

    def execute(self, cmd, now):
        op = cmd[0]
        if op == 128:
            self.set_mode(1, now)
            return
        if self.mode == 0:
            return          # OI off: only Start is heard
        if op in (130, 131):
            if not self.script.safety(self.k):
                self.set_mode(2, now)
        elif op == 132:
            self.set_mode(3, now)
        elif op in (133, 134, 135, 136, 143):
            self.set_mode(1, now)
        elif op == 173:
            self.set_mode(0, now)
            self.streaming = False
        elif op == 7:
            self.set_mode(0, now)
            self.streaming = False
            self.layout = None
            if self.master is not None:
                os.write(self.master, BOOT_TEXT)
        elif op == 148:
            ids = list(cmd[2:])
            if self.t0 is None:
                self.t0 = now
                self.k = 0
                with open(self.t0_file + '.tmp', 'w') as f:
                    json.dump({'t0': now, 't0_wall': time.time() - (time.monotonic() - now)}, f)
                os.replace(self.t0_file + '.tmp', self.t0_file)
                self._event('t0')
            self.layout = ids
            self.streaming = True
            self._event('stream', ids=ids)
        elif op == 150:
            self.streaming = bool(cmd[1]) and self.layout is not None
        elif op in (142, 149):
            ids = [cmd[1]] if op == 142 else list(cmd[2:])
            if self.master is not None:
                os.write(self.master, b''.join(self.packet(i) for i in ids))
        elif op in (137, 145) and self.mode in (2, 3):
            a, b = struct.unpack('>hh', cmd[1:])
            self.requested = (a, b, 0, 0) if op == 137 else (0, 0, a, b)

    # -------------------------------------------------------------- frames
    def packet(self, pid, v=None):
        if v is None:
            v = self.script.values(self.k, self.mode, self.requested, len(self.layout or []))
        return b''.join(struct.pack('>' + FMT[i], v[i]) for i in ids_of(pid))

    def frame(self):
        v = self.script.values(self.k, self.mode, self.requested, len(self.layout))
        payload = b''.join(bytes([pid]) + self.packet(pid, v) for pid in self.layout)
        body = bytes([19, len(payload)]) + payload
        return body + bytes([-sum(body) & 0xFF])

    def tick(self, now):
        """Frame k is due at t0 + k * 15 ms."""
        self.k += 1
        t = self.k * TICK
        sc = self.script
        if sc.scenario == 'ab' and sc.dock_k is None and t >= DOCK_AT and self.streaming and self.layout \
                and 34 not in {i for pid in self.layout for i in ids_of(pid)}:
            sc.dock_k = self.k
            self._event('dock')
        if self.mode == 2 and sc.safety(self.k):
            self.set_mode(1, now)
        if sc.asleep(t):
            self.awake_after_sleep = True
            return
        if self.awake_after_sleep:
            # Woke up (e.g. Clean pressed): the OI is off and must be started again.
            self.awake_after_sleep = False
            self.set_mode(0, now)
            self.streaming = False
            self.layout = None
            self._event('woke')
        if not self.streaming or not self.layout or sc.dropout(t) or self.hold_stream:
            return
        frame = self.frame()
        out = self.pending
        self.pending = b''
        ab = sc.scenario == 'ab'
        if ab and window(t, 25.0, 26.0) and self.k % 7 == 0:
            frame = frame[:5] + bytes([frame[5] ^ 0x40]) + frame[6:]
            self.stats['corrupted'] += 1
        if ab and window(t, 26.2, 26.2 + TICK):
            out += frame[:20]
            self.pending = frame[20:]
            self._event('split')
        elif ab and window(t, 26.215, 26.245):
            return      # the rest of the split frame arrives 45 ms late, with the next frame
        else:
            if ab and window(t, 26.5, 26.5 + TICK):
                out += bytes([0x13, 0xFF, 0x13, 0x05, 0x00, 0x13, 0x43, 0x00, 0x00])
                self._event('garbage')
            out += frame
        if self.master is None:
            return      # unplugged
        try:
            os.write(self.master, out)
            self.stats['frames'] += 1
        except BlockingIOError:
            self._event('overflow')

    def run(self):
        stop = []
        signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
        signal.signal(signal.SIGINT, lambda *_: stop.append(1))
        start = time.monotonic()
        while not stop:
            now = time.monotonic()
            if self.duration and now - start > self.duration:
                break
            if self.link_at is not None and now >= self.link_at:
                self.make_link()
            # Unplug half-way between two frames, not right after writing one: a real unplug isn't
            # synchronised with the stream, so the driver is waiting for data, not processing a frame.
            if self.replug_at is not None and self.t0 is not None and now - self.t0 >= self.replug_at \
                    and 0.006 < (now - self.t0) % TICK < 0.009:
                self.unplug(now)
            if self.relink_at is not None and now >= self.relink_at:
                self.open_pty()
                self.make_link()
                self.relink_at = None
                self.hold_stream = True
                self._event('replugged', pty=os.ttyname(self.slave))
            due = self.t0 + (self.k + 1) * TICK if self.t0 is not None else now + 0.05
            timeout = min(max(0.0, due - now), 0.05)
            if self.replug_at is not None and self.t0 is not None and now - self.t0 > self.replug_at - 0.1:
                timeout = min(timeout, 0.001)
            try:
                ready, _, _ = select.select([self.master] if self.master is not None else [], [], [], timeout)
            except InterruptedError:
                continue
            now = time.monotonic()
            if ready:
                try:
                    data = os.read(self.master, 4096)
                except (BlockingIOError, InterruptedError):
                    data = b''
                except OSError:
                    data = b''
                if data:
                    self.on_bytes(data, now)
            if self.t0 is not None:
                while now >= self.t0 + (self.k + 1) * TICK:
                    self.tick(now)
        self._event('stop', **self.stats)
        self.log.close()
        if os.path.lexists(self.link):
            os.unlink(self.link)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--link', required=True, help='symlink to create for the pty slave (the driver port)')
    ap.add_argument('--log', required=True, help='JSON lines log of received bytes and events')
    ap.add_argument('--t0-file', required=True, help='written with the time of the first stream request')
    ap.add_argument('--scenario', default='ab', choices=('ab', 'lightbump', 'steady', 'replug'))
    ap.add_argument('--duration', type=float, default=0.0, help='exit after this many seconds (0 = on SIGTERM)')
    FakeRoomba(ap.parse_args()).run()


if __name__ == '__main__':
    sys.exit(main())
