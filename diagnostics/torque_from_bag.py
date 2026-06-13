#!/usr/bin/env python3
"""
Per-servo torque analysis from a Gazebo walk bag (the torque-budget metric of record).

Reads /joint_states (which carries `effort` = the joint's TRANSMITTED-wrench torque about
its axis — ground reaction INCLUDED — only in mode:=gazebo, where the URDF declares the
effort state interface) out of a ros2 bag sqlite file. Reorders to canonical servo-ID order
(FL,FR,RL,RR x hip,knee,ankle), phase-bins every sample across all gait cycles (period
param), and reports per-joint peak |torque|, RMS, mean|torque|, and % of the 2.94 N.m
(30 kg.cm) ST3215 cap.

Outputs (to artifacts/):
  torque_timeseries.csv   t, then 12 effort columns in canonical order
  torque_phase.csv        phase (0..1), then per-joint mean and peak across cycles
  prints the headroom summary table.

  python3 torque_from_bag.py /root/barq_ws/artifacts/torquebag/torquebag_0.db3 [--period 0.5]
"""

import argparse
import math
import os
import sqlite3

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

JOINTS = [f'{leg}_{part}_joint' for leg in ('FL', 'FR', 'RL', 'RR')
          for part in ('hip', 'knee', 'ankle')]
TORQUE_CAP = 2.94          # N.m, ST3215 @12 V (30 kg.cm) — URDF effort limit (D-018)
ART = os.path.expanduser('~/barq_ws/artifacts')


def load_joint_states(db_path):
    con = sqlite3.connect(db_path)
    tid = {n: i for i, n, t in con.execute('SELECT id, name, type FROM topics')}
    if '/joint_states' not in tid:
        raise SystemExit('bag has no /joint_states')
    rows = con.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                       (tid['/joint_states'],)).fetchall()
    con.close()
    idx = None
    out = []                       # (t_sec, effort[12] in canonical order)
    for ts, data in rows:
        m = deserialize_message(data, JointState)
        if not m.effort:
            continue
        if idx is None:
            if not all(j in m.name for j in JOINTS):
                continue
            idx = [m.name.index(j) for j in JOINTS]
        out.append((ts * 1e-9, [m.effort[i] for i in idx]))
    if not out:
        raise SystemExit('no /joint_states samples carried an effort field — '
                         'was the sim launched in mode:=gazebo with the effort state interface?')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('db')
    ap.add_argument('--period', type=float, default=0.5)
    ap.add_argument('--bins', type=int, default=25)
    args = ap.parse_args()

    samples = load_joint_states(args.db)
    t0 = samples[0][0]
    span = samples[-1][0] - t0
    os.makedirs(ART, exist_ok=True)

    # 1) raw time series
    with open(os.path.join(ART, 'torque_timeseries.csv'), 'w') as f:
        f.write('t,' + ','.join(JOINTS) + '\n')
        for t, eff in samples:
            f.write(f'{t - t0:.4f},' + ','.join(f'{e:.4f}' for e in eff) + '\n')

    # 2) phase-bin across all cycles
    nb = args.bins
    bin_vals = [[[] for _ in range(12)] for _ in range(nb)]
    for t, eff in samples:
        ph = ((t - t0) % args.period) / args.period
        b = min(nb - 1, int(ph * nb))
        for j in range(12):
            bin_vals[b][j].append(eff[j])
    with open(os.path.join(ART, 'torque_phase.csv'), 'w') as f:
        f.write('phase,' + ','.join(f'{j}_mean' for j in JOINTS) +
                ',' + ','.join(f'{j}_peak' for j in JOINTS) + '\n')
        for b in range(nb):
            ph = (b + 0.5) / nb
            means = [sum(v) / len(v) if v else 0.0 for v in bin_vals[b]]
            peaks = [max(v, key=abs) if v else 0.0 for v in bin_vals[b]]
            f.write(f'{ph:.3f},' + ','.join(f'{m:.4f}' for m in means) +
                    ',' + ','.join(f'{p:.4f}' for p in peaks) + '\n')

    # 3) headroom summary
    print(f'TORQUE BUDGET  ({len(samples)} samples, {span:.1f}s, '
          f'~{span / args.period:.0f} cycles, cap {TORQUE_CAP} N.m)')
    print(f'  {"servo":<16} {"peak|t|":>8} {"RMS":>7} {"mean|t|":>8} {"%cap":>6} {"headroom":>9}')
    worst = (None, 0.0)
    for j, name in enumerate(JOINTS):
        col = [eff[j] for _, eff in samples]
        peak = max(abs(x) for x in col)
        rms = math.sqrt(sum(x * x for x in col) / len(col))
        meanabs = sum(abs(x) for x in col) / len(col)
        pct = peak / TORQUE_CAP * 100.0
        if peak > worst[1]:
            worst = (name, peak)
        flag = '  <-- WATCH' if pct > 80 else ''
        print(f'  {name:<16} {peak:8.3f} {rms:7.3f} {meanabs:8.3f} {pct:5.0f}% '
              f'{TORQUE_CAP - peak:8.3f}{flag}')
    print(f'  worst servo: {worst[0]} at {worst[1]:.3f} N.m '
          f'({worst[1] / TORQUE_CAP * 100:.0f}% of cap, '
          f'{TORQUE_CAP / worst[1]:.1f}x safety factor)')


if __name__ == '__main__':
    main()
