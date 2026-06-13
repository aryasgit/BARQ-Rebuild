#!/usr/bin/env python3
"""
Render the servo torque-budget figure from torque_from_bag.py's CSVs.

Top row: sustained cyclic torque vs gait phase (phase-MEAN across all recorded cycles,
which averages out single-sample contact-impact spikes), one panel per joint class
(hip/knee/ankle), 4 legs each — shows the diagonal-trot structure and the load split.
Bottom: peak |torque| per servo vs the 2.94 N.m (30 kg.cm) ST3215 cap.

  python3 plot_torque.py            # reads artifacts/torque_phase.csv + _timeseries.csv
Outputs artifacts/torque_graph.png
"""

import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt          # noqa: E402

ART = os.path.expanduser('~/barq_ws/artifacts')
CAP = 2.94
LEGS = ['FL', 'FR', 'RL', 'RR']
PARTS = ['hip', 'knee', 'ankle']
COLORS = {'FL': '#1f77b4', 'FR': '#ff7f0e', 'RL': '#2ca02c', 'RR': '#d62728'}
J = [f'{l}_{p}_joint' for l in LEGS for p in PARTS]


def read_csv(name):
    with open(os.path.join(ART, name)) as f:
        return list(csv.DictReader(f))


def main():
    phase = read_csv('torque_phase.csv')
    series = read_csv('torque_timeseries.csv')
    ph = [float(r['phase']) for r in phase]

    fig = plt.figure(figsize=(13, 8))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.3, 1.0], hspace=0.32, wspace=0.22)

    # ── top: sustained torque vs gait phase, per joint class ──
    for c, part in enumerate(PARTS):
        ax = fig.add_subplot(gs[0, c])
        for leg in LEGS:
            y = [float(r[f'{leg}_{part}_joint_mean']) for r in phase]
            ax.plot(ph, y, color=COLORS[leg], lw=2, label=leg)
        ax.axhline(CAP, ls='--', color='grey', lw=1)
        ax.axhline(-CAP, ls='--', color='grey', lw=1)
        ax.axhline(0, color='k', lw=0.5, alpha=0.4)
        ax.set_title(f'{part.capitalize()} — sustained torque vs gait phase')
        ax.set_xlabel('gait phase (0–1, one trot cycle)')
        if c == 0:
            ax.set_ylabel('joint torque  (N·m)')
        ax.set_ylim(-3.2, 3.2)
        ax.legend(fontsize=8, ncol=4, loc='upper right')
        ax.grid(alpha=0.25)

    # ── bottom: peak |torque| per servo vs cap ──
    ax = fig.add_subplot(gs[1, :])
    peak = []
    for j in J:
        col = [abs(float(r[j])) for r in series]
        peak.append(max(col))
    xs = range(12)
    bars = ax.bar(xs, peak, color=[COLORS[j.split('_')[0]] for j in J], alpha=0.85)
    ax.axhline(CAP, ls='--', color='red', lw=1.5, label=f'ST3215 cap {CAP} N·m')
    for i, (b, p) in enumerate(zip(bars, peak)):
        ax.text(i, p + 0.04, f'{p:.2f}', ha='center', va='bottom', fontsize=7)
    ax.set_xticks(list(xs))
    ax.set_xticklabels([j.replace('_joint', '').replace('_', '\n') for j in J], fontsize=8)
    ax.set_ylabel('peak |torque| (N·m)')
    ax.set_title('Peak per-servo torque vs cap  (transmitted wrench — includes foot-strike '
                 'impact transients, partly a rigid-contact sim artifact)')
    ax.set_ylim(0, 3.4)
    ax.legend(loc='lower right', fontsize=9)
    ax.grid(axis='y', alpha=0.25)

    fig.suptitle('BARQ servo torque budget — normal trot, vx 0.15 m/s, duty 0.6 (sim, '
                 'Gazebo transmitted-wrench)', fontsize=13, y=0.98)
    out = os.path.join(ART, 'torque_graph.png')
    fig.savefig(out, dpi=130, bbox_inches='tight')
    print('wrote', out)


if __name__ == '__main__':
    main()
