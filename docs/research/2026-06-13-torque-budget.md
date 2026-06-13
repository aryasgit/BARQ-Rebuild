# Servo torque budget — normal trot (sim)

**Date:** 2026-06-13 · **Author:** Aryaman Gupta · **Stage:** 2E sim (pre-hardware)
**Artifacts:** `2026-06-13-torque-budget.png`, `-torque-timeseries.csv`, `-torque-phase.csv`
**Tools:** `diagnostics/torque_from_bag.py`, `diagnostics/plot_torque.py`

## Question
How much torque does each of the 12 ST3215 servos bear at each point during a normal gait,
and how does that compare to the 2.94 N·m (30 kg·cm) actuator cap? (Servo-sizing confidence +
input to P4 torque staging and the P6 RL torque penalty.)

## Method — how the sim yields true joint torque
The Gazebo plugin (`external/gz_ros2_control`) computes each joint's effort as
`effort = JointTransmittedWrench.torque · joint_axis` (`gz_system.cpp` ~L706–732) — the scalar
constraint torque actually transmitted through the joint, **ground reaction included**. This is
the physically correct "torque the servo bears," and it is why a free-leg inverse-dynamics calc
would be wrong (it misses the stance ground-reaction load, which is the dominant term).

That effort only reaches `/joint_states` if the URDF declares an effort **state interface**. We
added one, **guarded to `mode:=gazebo`** (mock GenericSystem and real `barq_hw/BarqSystem` don't
export effort, so the guard keeps them error-free):
```xml
<xacro:if value="${'$(arg mode)' == 'gazebo'}"><state_interface name="effort"/></xacro:if>
```
Then: walk at vx 0.15 m/s, duty 0.6 (the normal gait), record `/joint_states` for ~7 s (~14 trot
cycles) at steady state, phase-bin every sample over the 0.5 s period, and reduce. Reproduce:
```bash
ros2 launch barq_bringup sim.launch.py gait:=true gui:=false        # then drive cmd_vel 0.15
ros2 bag record -o artifacts/torquebag /joint_states                 # ~8 s while walking
python3 diagnostics/torque_from_bag.py artifacts/torquebag/torquebag_0.db3
python3 diagnostics/plot_torque.py
```

## Results
| Servo | RMS (N·m) | mean\|τ\| | sustained cyclic peak¹ | transient peak² | % cap (peak) |
|---|---|---|---|---|---|
| FL hip | 0.70 | 0.41 | 0.54 | 3.00 | 102% |
| FL knee | 1.00 | 0.59 | 0.54 | 3.02 | 103% |
| FL ankle | 0.72 | 0.37 | 0.66 | 3.02 | 103% |
| FR hip | 0.53 | 0.31 | 0.35 | 2.96 | 101% |
| FR knee | 0.86 | 0.51 | 0.54 | 3.02 | 103% |
| FR ankle | 0.62 | 0.31 | 0.51 | 3.00 | 102% |
| RL hip | 0.58 | 0.34 | 0.34 | 2.96 | 101% |
| RL knee | 1.17 | 0.75 | 1.43 | 3.08 | 105% |
| **RL ankle** | **1.31** | 0.76 | **1.86** | 2.97 | 101% |
| RR hip | 0.74 | 0.42 | 0.52 | 2.97 | 101% |
| RR knee | 1.19 | 0.76 | 0.98 | 3.04 | 103% |
| RR ankle | 1.30 | 0.79 | 1.21 | 2.95 | 100% |

¹ peak of the phase-MEAN curve (averaged over 14 cycles — the trustworthy sustained signal).
² max single-sample \|τ\| (foot-strike transient; see caveats).

**Two distinct numbers, both true:**
- **Continuous load is comfortable.** Worst-case RMS is 1.31 N·m (RL ankle) = 45% of cap; the
  sustained cyclic peak reaches 1.86 N·m (63%). Continuous safety factor ≈ **2.2×** worst-case.
  The servos will not torque- or thermally-saturate in steady trot.
- **Foot-strike transients touch the cap.** Every joint shows brief spikes at ~2.95–3.08 N·m,
  concentrated at touchdown and worst on the rear legs (RL/RR ankle exceed 2.5 N·m on 13–14% of
  samples vs <2% on the front).

## Findings
1. **Load sits on the REAR legs, not the front** — counter to first intuition about the D-016
   nose-down (load-forward) trim. Rear ankles bear ~2× the front ankles' RMS. Mechanism: D-016
   shifts the vertical *force* forward, but it does so by *extending* the rear legs (`rear_raise`),
   which lengthens their moment arms — and torque = force × moment arm. The crouched front legs
   keep the foot nearly under the hip (short arm). Net: the stability trim **buys forward balance
   at the cost of rear-ankle torque headroom.** → Q-017.
2. **The trot's diagonal structure is visible** in the ankle phase curves: the stance hump of each
   leg, with the diagonal pairs (FL+RR / FR+RL) offset by half a cycle.

## Caveats (do not over-read the peaks)
- `JointTransmittedWrench` is the **total** structural torque through the joint (actuator + contact
  impulse + inertial), **not** pure motor demand. The motor itself is clamped at the 2.94 N·m
  effort limit; transmitted-wrench values *above* the cap (3.0–3.08) are impact impulses reacted by
  the joint **structure** — relevant to horn/gearbox stress at touchdown, not motor saturation.
- The transient peaks are partly a **rigid-contact sim artifact**: hard foot sphere, no modeled
  servo/gear compliance. The real robot's compliant TPU foot pad + ST3215 internal compliance will
  blunt them. **Sustained numbers transfer well; treat the transient peaks as a conservative upper
  bound**, to be re-measured on the bench/robot (P3-03, P4).

## Implications for the build
- **Servo sizing confirmed adequate** for continuous trot (2.2× margin), **marginal at worst-case
  impacts** — exactly the regime the soft-touchdown swing (D-019) was designed to reduce.
- **P4 torque staging:** bring the rear legs up conservatively; watch RL/RR ankle+knee current at
  touchdown first.
- **P6 RL:** weight the torque penalty to discourage the rear-ankle saturation; this graph is the
  pre-RL baseline the policy must beat (smoother torque, lower peaks, ideally flatter L/R + F/R).
- **Hardware check:** the bench step-response (P3-03) and first-walk current logs (P4) should
  reproduce the *continuous* band; if real peaks at touchdown are far below 2.94, the rigid-contact
  artifact is confirmed and the cap concern relaxes.
