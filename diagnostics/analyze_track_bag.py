#!/usr/bin/env python3
"""
Offline trot-tracking analysis from a ros2 bag (the metric of record for D-018).

Reads /joint_states and /joint_group_position_controller/commands out of a bag's
sqlite3 file directly (no executor — immune to the subscriber starvation that
invalidates live sampling on a loaded Jetson), pairs each state with the latest
command (zero-order hold, exactly what the position controller does), and prints
per-joint RMS / peak tracking error.

  ros2 bag record -o /tmp/track /joint_states /joint_group_position_controller/commands
  python3 analyze_track_bag.py /tmp/track/track_0.db3
"""

import math
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = [f'{leg}_{part}_joint' for leg in ('FL', 'FR', 'RL', 'RR')
          for part in ('hip', 'knee', 'ankle')]


def load(db_path):
    con = sqlite3.connect(db_path)
    topics = {name: (tid, mtype) for tid, name, mtype in
              con.execute('SELECT id, name, type FROM topics')}
    out = {}
    for name, (tid, _) in topics.items():
        rows = con.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (tid,)).fetchall()
        out[name] = rows
    con.close()
    return out


def main():
    db = sys.argv[1]
    raw = load(db)
    js_rows = raw.get('/joint_states', [])
    cmd_rows = raw.get('/joint_group_position_controller/commands', [])
    if not js_rows or not cmd_rows:
        sys.exit(f'bag has {len(js_rows)} states / {len(cmd_rows)} commands - record both')

    cmds = [(t, list(deserialize_message(d, Float64MultiArray).data))
            for t, d in cmd_rows]
    idx = None
    states = []
    for t, d in js_rows:
        m = deserialize_message(d, JointState)
        if idx is None:
            if not all(j in m.name for j in JOINTS):
                continue
            idx = [m.name.index(j) for j in JOINTS]
        states.append((t, [m.position[i] for i in idx]))

    # use only states inside the command window (walking), with ZOH pairing
    t_lo, t_hi = cmds[0][0], cmds[-1][0]
    states = [s for s in states if t_lo <= s[0] <= t_hi]
    span = (t_hi - t_lo) * 1e-9
    print(f'TRACK(bag) {span:.1f}s of commands, {len(states)} paired states, '
          f'{len(cmds)} commands ({len(cmds) / span:.0f} Hz)')

    print(f'  {"joint":<16} {"RMS err":>10} {"peak err":>10}')
    worst = (None, 0.0)
    rms_all = []
    ci = 0
    errs_by_joint = [[] for _ in JOINTS]
    for t, pos in states:
        while ci + 1 < len(cmds) and cmds[ci + 1][0] <= t:
            ci += 1
        for i in range(12):
            errs_by_joint[i].append(pos[i] - cmds[ci][1][i])
    for i, j in enumerate(JOINTS):
        e = errs_by_joint[i]
        rms = math.sqrt(sum(x * x for x in e) / len(e))
        peak = max(abs(x) for x in e)
        rms_all.append(rms)
        if peak > worst[1]:
            worst = (j, peak)
        print(f'  {j:<16} {rms * 1000:7.1f}mrad {peak * 1000:7.1f}mrad')
    print(f'  mean RMS {sum(rms_all) / 12 * 1000:.1f} mrad; '
          f'worst {worst[0]} peak {worst[1] * 1000:.1f} mrad')


if __name__ == '__main__':
    main()
