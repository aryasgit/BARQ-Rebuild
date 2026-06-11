#!/usr/bin/env python3
"""
Stage-4 integration test: full control stack against the REAL firmware logic, zero hardware.

  emulator (barq_firmware LoopCore on a PTY)
      <- phase A: Python codec direct  (PING/PONG, CMD->STATE echo, 200 ms deadman)
      <- phase B: ros2_control_node + barq_hw/BarqSystem + controllers (the robot launch line)

Run inside the dev container:  python3 integration_pty.py
Isolated on ROS_DOMAIN_ID=42 so a running sim container does not cross-talk.
Exit 0 = drop-in ready: on hardware day only `device:=` changes.
"""

import os

os.environ['ROS_DOMAIN_ID'] = '42'             # before any rclpy import

import math                                    # noqa: E402
import signal                                  # noqa: E402
import subprocess                              # noqa: E402
import sys                                     # noqa: E402
import time                                    # noqa: E402
import tty                                     # noqa: E402

from barq_control.barq_protocol import (Decoder, decode_state, encode_cmd, frame,  # noqa: E402
                                        TYPE_PONG, TYPE_STATE, TYPE_PING)

STANCE = [0.0, 1.047531, -1.928768, 0.0, 1.047531, -1.928768,
          0.0, 0.911998, -1.652637, 0.0, 0.911998, -1.652637]
CHECKS = []


def check(name, ok, detail=''):
    CHECKS.append((name, ok))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
    return ok


def drain(fd, dec, t_sec):
    """Collect (type, seq, payload) frames from fd for t_sec wall seconds."""
    out = []
    end = time.monotonic() + t_sec
    while time.monotonic() < end:
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            data = b''
        if data:
            out += dec.feed(data)
        else:
            time.sleep(0.002)
    return out


def phase_a(pty_path):
    print(f'— phase A: Python codec <-> firmware LoopCore on {pty_path}')
    fd = os.open(pty_path, os.O_RDWR | os.O_NONBLOCK)
    tty.setraw(fd)
    dec = Decoder()
    try:
        os.write(fd, frame(TYPE_PING, b'\x01\x02\x03\x04', seq=7))
        frames = drain(fd, dec, 0.3)
        pongs = [f for f in frames if f[0] == TYPE_PONG]
        check('PING -> PONG', bool(pongs) and pongs[0][1] == 7
              and pongs[0][2] == b'\x01\x02\x03\x04')

        # Stream CMD at 100 Hz; the loopback servo model must converge onto the targets.
        end = time.monotonic() + 0.5
        seq = 0
        while time.monotonic() < end:
            os.write(fd, encode_cmd(STANCE, seq=seq & 0xFF))
            seq += 1
            frames += drain(fd, dec, 0.01)
        states = [decode_state(p) for t, _, p in frames if t == TYPE_STATE]
        check('STATE telemetry flowing', len(states) > 30, f'{len(states)} frames')
        last = states[-1]
        err = max(abs(p - t) for p, t in zip(last['pos'], STANCE))
        check('loopback converged onto CMD targets', err < 0.005, f'max err {err*1000:.1f} mrad')
        check('no fault while driven', last['fault'] == 0, f"fault=0x{last['fault']:02x}")

        # Deadman: stop commanding; fault bit3 must latch within ~200 ms. The drain also
        # returns frames BUFFERED from silence-age <200 ms (correctly fault-free), so the
        # assertion is on the freshest tail, not the whole window.
        time.sleep(0.35)
        states = [decode_state(p) for t, _, p in drain(fd, dec, 0.25) if t == TYPE_STATE]
        check('deadman trips after CMD silence (bit3)',
              len(states) >= 5 and all(s['fault'] & 0x08 for s in states[-5:]),
              f"last fault=0x{states[-1]['fault']:02x}" if states else 'no STATE')
    finally:
        os.close(fd)


def phase_b(pty_path):
    print('— phase B: ros2_control stack on the same PTY (the robot launch line)')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'barq_bringup', 'real.launch.py',
         f'device:={pty_path}', 'gait:=false'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    try:
        import rclpy
        from rclpy.node import Node as RclpyNode
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        rclpy.init()
        node = RclpyNode('stage4_checker')
        got = {'msgs': [], 't0': None}

        def on_js(m):
            got['msgs'].append((time.monotonic(), dict(zip(m.name, m.position))))

        node.create_subscription(JointState, '/joint_states', on_js, 50)
        pub = node.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)

        end = time.monotonic() + 30.0
        while not got['msgs'] and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        check('stack up: /joint_states publishing', bool(got['msgs']))
        if not got['msgs']:
            return
        check('12 joints present', len(got['msgs'][-1][1]) == 12)

        # Command a small offset pose; the chain CM->BarqSystem->PTY->LoopCore->back
        # must land the measured positions on it.
        target = [v + 0.1 * math.copysign(1.0, v or 1.0) for v in STANCE]
        names = [f'{leg}_{p}_joint' for leg in ('FL', 'FR', 'RL', 'RR')
                 for p in ('hip', 'knee', 'ankle')]
        got['msgs'].clear()
        end = time.monotonic() + 4.0
        while time.monotonic() < end:
            pub.publish(Float64MultiArray(data=target))
            rclpy.spin_once(node, timeout_sec=0.02)
        n_msgs = len(got['msgs'])
        check('JSB streaming during drive', n_msgs > 100, f'{n_msgs} msgs in 4 s')
        last = got['msgs'][-1][1]
        err = max(abs(last[n] - t) for n, t in zip(names, target))
        check('round-trip convergence (CM->serial->firmware->CM)',
              err < 0.005, f'max err {err*1000:.1f} mrad')
        node.destroy_node()
        rclpy.shutdown()
    finally:
        os.killpg(os.getpgid(launch.pid), signal.SIGINT)
        try:
            launch.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(launch.pid), signal.SIGKILL)


def main():
    emu = subprocess.Popen(['ros2', 'run', 'barq_hw', 'teensy_emulator'],
                           stdout=subprocess.PIPE, text=True, preexec_fn=os.setsid)
    try:
        line = emu.stdout.readline().strip()
        assert line.startswith('PTY '), f'unexpected emulator banner: {line!r}'
        pty_path = line.split(' ', 1)[1]
        print(f'emulator up on {pty_path}')
        phase_a(pty_path)
        phase_b(pty_path)
    finally:
        os.killpg(os.getpgid(emu.pid), signal.SIGTERM)

    failed = [n for n, ok in CHECKS if not ok]
    print(f'\n{len(CHECKS) - len(failed)}/{len(CHECKS)} checks passed')
    if failed:
        print('FAILED:', ', '.join(failed))
        sys.exit(1)
    print('Stage 4 drop-in contract holds: swap device:= for /dev/ttyACM0 on hardware day.')


if __name__ == '__main__':
    main()
