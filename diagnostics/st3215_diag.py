#!/usr/bin/env python3
"""
ST3215 bench diagnostics & calibration tool (Waveshare/Feetech STS-series serial bus servos).

Talks the Feetech STS protocol directly over a Waveshare Serial Bus Servo Driver (or any
UART-TTL half-duplex adapter) plugged into USB. No ROS, no Teensy needed — this is the
pre-assembly bench tool: assign IDs, calibrate mid-points, read every diagnostic, and verify
each servo actually tracks commands before screwing anything in place.

Usage examples (default port auto-detect, default baud 1,000,000):
  ./st3215_diag.py scan                          # find every servo on the bus
  ./st3215_diag.py ping 1                        # is ID 1 alive?
  ./st3215_diag.py status 1                      # full diagnostic snapshot
  ./st3215_diag.py monitor 1 --hz 5              # live feedback (Ctrl-C to stop)
  ./st3215_diag.py set-id 1 7                    # factory ID 1 -> bus ID 7 (one servo at a time!)
  ./st3215_diag.py calibrate-mid 7               # set current physical pose as center (2048)
  ./st3215_diag.py move 7 2048 --speed 800       # commanded move + tracking report
  ./st3215_diag.py sweep 7 --amp-deg 45          # sine sweep + tracking-error report
  ./st3215_diag.py torque 7 off                  # free the output shaft
  ./st3215_diag.py limits 7                      # read (or --min/--max to write) angle limits
  ./st3215_diag.py plan                          # BARQ wiring/ID plan (matches robot_params.yaml)

Safety: bench servos WITHOUT horns/linkages attached first. Power the driver board from the
rated supply (2S / 7.4 V class for ST3215), not USB. Assign IDs one servo at a time.
"""

import argparse
import glob
import math
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  pip3 install pyserial")

# ── Feetech STS register map (ST3215) ──────────────────────────────────────────
EPROM_ID = 0x05
EPROM_BAUD = 0x06
EPROM_MIN_ANGLE = 0x09        # 2B
EPROM_MAX_ANGLE = 0x0B        # 2B
EPROM_MAX_TEMP = 0x0D
EPROM_MAX_VOLT = 0x0E
EPROM_MIN_VOLT = 0x0F
EPROM_MAX_TORQUE = 0x10       # 2B
EPROM_OFS = 0x1F              # 2B position offset
EPROM_MODE = 0x21             # 0 position / 1 wheel / 2 PWM / 3 step
REG_TORQUE_ENABLE = 0x28      # 0/1; writing 128 calibrates current pose as 2048
REG_ACC = 0x29
REG_GOAL_POS = 0x2A           # 2B
REG_GOAL_SPEED = 0x2E         # 2B
REG_LOCK = 0x37               # EPROM lock: 0 unlock, 1 lock
REG_PRESENT_POS = 0x38        # 2B
REG_PRESENT_SPEED = 0x3A      # 2B signed(bit15)
REG_PRESENT_LOAD = 0x3C       # 2B signed(bit15), ~0.1% units
REG_PRESENT_VOLT = 0x3E       # 0.1 V
REG_PRESENT_TEMP = 0x3F       # degC
REG_MOVING = 0x42
REG_PRESENT_CURRENT = 0x45    # 2B, ~6.5 mA units

INSTR_PING, INSTR_READ, INSTR_WRITE = 0x01, 0x02, 0x03

COUNTS_PER_REV = 4096
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV
CENTER = 2048

# BARQ servo ID plan — keep in sync with barq_description/config/robot_params.yaml (servos:).
ID_PLAN = [
    ('FL_coxa', 0), ('FL_femur', 1), ('FL_tibia', 2),
    ('FR_coxa', 3), ('FR_femur', 4), ('FR_tibia', 5),
    ('RL_coxa', 6), ('RL_femur', 7), ('RL_tibia', 8),
    ('RR_coxa', 9), ('RR_femur', 10), ('RR_tibia', 11),
]


def s15(v):
    """Decode Feetech 15-bit signed (bit 15 = sign)."""
    return -(v & 0x7FFF) if (v & 0x8000) else v


def counts_to_deg(c):
    return (c - CENTER) * DEG_PER_COUNT


class Bus:
    """Minimal half-duplex Feetech STS bus master."""

    def __init__(self, port, baud, timeout=0.05):
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def _txrx(self, sid, instr, params=b'', expect_reply=True):
        body = bytes([sid, len(params) + 2, instr]) + bytes(params)
        chk = (~sum(body)) & 0xFF
        self.ser.reset_input_buffer()
        self.ser.write(b'\xff\xff' + body + bytes([chk]))
        self.ser.flush()
        if not expect_reply:
            return None
        hdr = self.ser.read(4)
        if len(hdr) < 4 or hdr[0] != 0xFF or hdr[1] != 0xFF:
            return None
        rid, ln = hdr[2], hdr[3]
        rest = self.ser.read(ln)
        if len(rest) < ln:
            return None
        err, payload = rest[0], rest[1:-1]
        if (~(rid + ln + sum(rest[:-1]))) & 0xFF != rest[-1]:
            return None
        return err, payload

    def ping(self, sid):
        return self._txrx(sid, INSTR_PING) is not None

    def read(self, sid, addr, n):
        r = self._txrx(sid, INSTR_READ, bytes([addr, n]))
        if r is None:
            raise IOError(f'no response from ID {sid} (addr 0x{addr:02X})')
        return r[1]

    def write(self, sid, addr, data):
        r = self._txrx(sid, INSTR_WRITE, bytes([addr]) + bytes(data))
        if r is None:
            raise IOError(f'no response from ID {sid} on write 0x{addr:02X}')
        if r[0] != 0:
            print(f'  [!] servo error flags: 0x{r[0]:02X}')

    def read8(self, sid, addr):
        return self.read(sid, addr, 1)[0]

    def read16(self, sid, addr):
        d = self.read(sid, addr, 2)
        return d[0] | (d[1] << 8)          # STS = little-endian (low byte first)

    def write16(self, sid, addr, val):
        self.write(sid, addr, [val & 0xFF, (val >> 8) & 0xFF])

    def unlock(self, sid):
        self.write(sid, REG_LOCK, [0])

    def lock(self, sid):
        self.write(sid, REG_LOCK, [1])


def find_port(explicit):
    if explicit:
        return explicit
    for pat in ('/dev/ttyACM*', '/dev/ttyUSB*', '/dev/tty.usbserial*', '/dev/tty.usbmodem*'):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    sys.exit('no serial adapter found - plug in the Waveshare driver board or pass --port')


# ── commands ────────────────────────────────────────────────────────────────────

def cmd_scan(bus, args):
    found = []
    bus.ser.timeout = 0.015
    print(f'scanning IDs 0..{args.max_id} ...')
    for sid in range(0, args.max_id + 1):
        if bus.ping(sid):
            found.append(sid)
            print(f'  ID {sid:3d}  ALIVE')
    bus.ser.timeout = 0.05
    names = {i: n for n, i in ID_PLAN}
    print(f'\n{len(found)} servo(s): {found}')
    for sid in found:
        if sid in names:
            print(f'  ID {sid} = {names[sid]} per BARQ plan')
    return 0 if found else 1


def cmd_ping(bus, args):
    ok = bus.ping(args.id)
    print(f'ID {args.id}: {"ALIVE" if ok else "no response"}')
    return 0 if ok else 1


def snapshot(bus, sid):
    pos = bus.read16(sid, REG_PRESENT_POS)
    spd = s15(bus.read16(sid, REG_PRESENT_SPEED))
    load = s15(bus.read16(sid, REG_PRESENT_LOAD))
    volt = bus.read8(sid, REG_PRESENT_VOLT) / 10.0
    temp = bus.read8(sid, REG_PRESENT_TEMP)
    cur = s15(bus.read16(sid, REG_PRESENT_CURRENT)) * 6.5
    moving = bus.read8(sid, REG_MOVING)
    return pos, spd, load, volt, temp, cur, moving


def fmt_snap(snap):
    pos, spd, load, volt, temp, cur, moving = snap
    return (f'pos {pos:4d} ({counts_to_deg(pos):+7.2f} deg)  spd {spd:5d}  '
            f'load {load / 10.0:+6.1f}%  {volt:4.1f} V  {temp:2d} C  {cur:6.1f} mA  '
            f'{"MOVING" if moving else "idle"}')


def cmd_status(bus, args):
    sid = args.id
    print(f'── ID {sid} ──')
    print('  ' + fmt_snap(snapshot(bus, sid)))
    mode = bus.read8(sid, EPROM_MODE)
    lo, hi = bus.read16(sid, EPROM_MIN_ANGLE), bus.read16(sid, EPROM_MAX_ANGLE)
    ofs = bus.read16(sid, EPROM_OFS)
    tq = bus.read8(sid, REG_TORQUE_ENABLE)
    mt = bus.read16(sid, EPROM_MAX_TORQUE)
    print(f'  mode {mode} (0=position)  torque {"ON" if tq else "OFF"}  max_torque {mt / 10.0:.0f}%')
    print(f'  angle limits [{lo}, {hi}] counts = [{counts_to_deg(lo):+.1f}, {counts_to_deg(hi):+.1f}] deg'
          f'  ofs {ofs}')
    print(f'  temp/volt limits: max {bus.read8(sid, EPROM_MAX_TEMP)} C, '
          f'[{bus.read8(sid, EPROM_MIN_VOLT) / 10.0:.1f}, {bus.read8(sid, EPROM_MAX_VOLT) / 10.0:.1f}] V')
    return 0


def cmd_monitor(bus, args):
    period = 1.0 / args.hz
    print(f'monitoring ID {args.id} at {args.hz} Hz - Ctrl-C to stop')
    try:
        while True:
            print('\r' + fmt_snap(snapshot(bus, args.id)) + '   ', end='', flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print()
    return 0


def cmd_torque(bus, args):
    bus.write(args.id, REG_TORQUE_ENABLE, [1 if args.state == 'on' else 0])
    print(f'ID {args.id}: torque {args.state.upper()}')
    return 0


def cmd_set_id(bus, args):
    if not bus.ping(args.old):
        sys.exit(f'ID {args.old} not responding - connect exactly one servo')
    if bus.ping(args.new):
        sys.exit(f'ID {args.new} already on the bus - aborting (IDs must be unique)')
    if input(f'change servo ID {args.old} -> {args.new}? [y/N] ').lower() != 'y':
        return 1
    bus.unlock(args.old)
    bus.write(args.old, EPROM_ID, [args.new])
    time.sleep(0.05)
    bus.lock(args.new)
    ok = bus.ping(args.new)
    print(f'ID {args.new}: {"ALIVE - done" if ok else "NO RESPONSE - check bus!"}')
    return 0 if ok else 1


def cmd_calibrate_mid(bus, args):
    sid = args.id
    bus.write(sid, REG_TORQUE_ENABLE, [0])
    print(f'ID {sid}: torque OFF. Move the joint to its MECHANICAL MIDDLE by hand.')
    input('press ENTER when in position ... ')
    bus.write(sid, REG_TORQUE_ENABLE, [128])   # Feetech: current pose becomes 2048
    time.sleep(0.1)
    pos = bus.read16(sid, REG_PRESENT_POS)
    print(f'calibrated. present position now {pos} (want ~{CENTER})')
    return 0 if abs(pos - CENTER) < 10 else 1


def _track(bus, sid, goal):
    snap = snapshot(bus, sid)
    err = snap[0] - goal
    return snap, err


def cmd_move(bus, args):
    sid = args.id
    goal = int(CENTER + args.deg / DEG_PER_COUNT) if args.deg is not None else args.pos
    if goal is None:
        sys.exit('give a position in counts, or --deg')
    bus.write(sid, REG_TORQUE_ENABLE, [1])
    bus.write(sid, REG_ACC, [args.acc])
    bus.write16(sid, REG_GOAL_SPEED, args.speed)
    bus.write16(sid, REG_GOAL_POS, goal)
    t0 = time.time()
    while time.time() - t0 < args.timeout:
        snap, err = _track(bus, sid, goal)
        print('\r  ' + fmt_snap(snap) + f'  err {err:+4d}', end='', flush=True)
        if not snap[6] and abs(err) < 8:
            break
        time.sleep(0.05)
    snap, err = _track(bus, sid, goal)
    print(f'\nfinal: goal {goal}  pos {snap[0]}  err {err:+d} counts ({err * DEG_PER_COUNT:+.2f} deg)')
    return 0 if abs(err) < 15 else 1


def cmd_sweep(bus, args):
    sid = args.id
    amp = int(args.amp_deg / DEG_PER_COUNT)
    bus.write(sid, REG_TORQUE_ENABLE, [1])
    bus.write(sid, REG_ACC, [args.acc])
    bus.write16(sid, REG_GOAL_SPEED, args.speed)
    print(f'sine sweep +/-{args.amp_deg} deg x{args.cycles} cycles on ID {sid} - Ctrl-C aborts')
    worst, t0 = 0, time.time()
    try:
        while time.time() - t0 < args.cycles / args.freq:
            t = time.time() - t0
            goal = int(CENTER + amp * math.sin(2 * math.pi * args.freq * t))
            bus.write16(sid, REG_GOAL_POS, goal)
            snap, err = _track(bus, sid, goal)
            worst = max(worst, abs(err))
            print('\r  ' + fmt_snap(snap) + f'  err {err:+4d}', end='', flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    bus.write16(sid, REG_GOAL_POS, CENTER)
    print(f'\nsweep done. worst tracking error {worst} counts ({worst * DEG_PER_COUNT:.2f} deg) '
          f'(lag while moving is normal; watch for stalls, grinding, hot temps)')
    return 0


def cmd_limits(bus, args):
    sid = args.id
    if args.min is not None or args.max is not None:
        bus.unlock(sid)
        if args.min is not None:
            bus.write16(sid, EPROM_MIN_ANGLE, args.min)
        if args.max is not None:
            bus.write16(sid, EPROM_MAX_ANGLE, args.max)
        bus.lock(sid)
    lo, hi = bus.read16(sid, EPROM_MIN_ANGLE), bus.read16(sid, EPROM_MAX_ANGLE)
    print(f'ID {sid} angle limits: [{lo}, {hi}] counts = '
          f'[{counts_to_deg(lo):+.1f}, {counts_to_deg(hi):+.1f}] deg')
    return 0


def cmd_plan(_bus, _args):
    print('BARQ wiring/ID plan (one Waveshare driver board per leg, 3 servos chained):')
    for i, (name, sid) in enumerate(ID_PLAN):
        if i % 3 == 0:
            print(f'  ── {name[:2]} leg bus ──')
        print(f'    ID {sid:2d}  {name}')
    print('\nkeep in sync with barq_description/config/robot_params.yaml (servos:)')
    print('bench flow per servo: scan -> set-id -> calibrate-mid -> sweep -> label it.')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='serial device (default: auto-detect)')
    ap.add_argument('--baud', type=int, default=1_000_000, help='bus baud (default 1M)')
    sub = ap.add_subparsers(dest='cmd', required=True)

    s = sub.add_parser('scan');            s.add_argument('--max-id', type=int, default=30)
    s = sub.add_parser('ping');            s.add_argument('id', type=int)
    s = sub.add_parser('status');          s.add_argument('id', type=int)
    s = sub.add_parser('monitor');         s.add_argument('id', type=int)
    s.add_argument('--hz', type=float, default=5.0)
    s = sub.add_parser('torque');          s.add_argument('id', type=int)
    s.add_argument('state', choices=['on', 'off'])
    s = sub.add_parser('set-id');          s.add_argument('old', type=int); s.add_argument('new', type=int)
    s = sub.add_parser('calibrate-mid');   s.add_argument('id', type=int)
    s = sub.add_parser('move');            s.add_argument('id', type=int)
    s.add_argument('pos', type=int, nargs='?')
    s.add_argument('--deg', type=float, help='target in degrees from center (overrides pos)')
    s.add_argument('--speed', type=int, default=800)
    s.add_argument('--acc', type=int, default=50)
    s.add_argument('--timeout', type=float, default=5.0)
    s = sub.add_parser('sweep');           s.add_argument('id', type=int)
    s.add_argument('--amp-deg', type=float, default=30.0)
    s.add_argument('--cycles', type=int, default=3)
    s.add_argument('--freq', type=float, default=0.25)
    s.add_argument('--speed', type=int, default=0)
    s.add_argument('--acc', type=int, default=0)
    s = sub.add_parser('limits');          s.add_argument('id', type=int)
    s.add_argument('--min', type=int); s.add_argument('--max', type=int)
    sub.add_parser('plan')

    args = ap.parse_args()
    if args.cmd == 'plan':
        return cmd_plan(None, args)
    bus = Bus(find_port(args.port), args.baud)
    return {
        'scan': cmd_scan, 'ping': cmd_ping, 'status': cmd_status, 'monitor': cmd_monitor,
        'torque': cmd_torque, 'set-id': cmd_set_id, 'calibrate-mid': cmd_calibrate_mid,
        'move': cmd_move, 'sweep': cmd_sweep, 'limits': cmd_limits,
    }[args.cmd](bus, args)


if __name__ == '__main__':
    sys.exit(main())
