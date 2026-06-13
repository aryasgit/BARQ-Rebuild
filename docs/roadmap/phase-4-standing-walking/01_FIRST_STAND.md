# P4-01 — First Stand: cradle → spotter → free-standing

> Phase P4 · verified against repo @ 0e5ddaf

## Objective
Take the robot from P3's air-walk (cradle, feet free) to a free stand on the floor:
staged torque ramp, stance hold with measured per-leg load symmetry, oscillation and
temperature triage, then lower to the floor and verify settle height + the D-016
nose-down trim against the model. Exit gates: **G4.1** (60 s free stand) and **G4.2**
(settle height + pitch within bars).

## Prerequisites (do not start without ALL of these)
1. **P3 exit gate passed**: `integration_pty.py` 9/9 on `/dev/ttyACM0` AND air-walk on
   the cradle with `ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=true`
   (README phase map row P3).
2. **P4-CODE-1 built and emulator-verified**: the `barq_hw` telemetry publishers
   (`/barq/diag` 1 Hz + `/imu/data` bridge) — full spec in
   `03_ESTIMATOR_AND_SAFETY.md` §5a. This file's load/temp/vbus steps read `/barq/diag`;
   without it you are standing blind. Verify against `teensy_emulator` first:
   `ros2 topic echo /barq/diag` must show 17 values.
3. **P4-CODE-2 added to the bench tool** (spec in §2 below): `st3215_diag.py max-torque`.
4. Battery charged (≥ 16.0 V resting), P1 power tree live, **master switch within arm's
   reach** (00 §3). Firmware deadman 200 ms NEVER disabled.
5. Cradle; foam pad / folded towel on the floor area; spotter present for §6+;
   steel rule + calipers; phone (slow-mo + photos); `docs/05_RESEARCH_LOG.md` open.
6. Servo zero-offsets calibrated per P2 (`calibrate-mid` done on all 12, IDs match
   `robot_params.yaml`).

Safety absolutes apply throughout (00_DOOMSDAY_PROTOCOL §3): cradle → spotter → free,
fingers out of the leg workspace whenever torque is on, torque-off one action away.

## Procedure

### 1. Pre-pose the legs (torque OFF)
With everything unpowered, hand-fold the legs to roughly the stance pose
(hip 0, knee ≈ +1.05 rad, ankle ≈ −1.93 rad — the D-014 stance initial values, photo in
P2). Reason: `BarqSystem` activates holding the *measured* pose (anti-lurch), but the
first `/foot_targets` from `ik_node` yanks the legs to stance at servo speed; starting
near stance turns that yank into a millimetre settle.

### 2. Staged torque ramp — 30 % → 60 % → 100 %
Mechanism: the ST3215 EPROM torque-limit register (`EPROM_MAX_TORQUE`, addr `0x10`,
units 0.1 %, persists across power cycles — `st3215_diag.py status` already displays it).

**P4-CODE-2 (small tooling addition, ~12 lines)** — `diagnostics/st3215_diag.py`:
a `max-torque` subcommand mirroring `cmd_limits`:
`cmd_max_torque(bus, args)` → `bus.unlock(id)`, `bus.write16(id, EPROM_MAX_TORQUE,
args.pct * 10)`, `bus.lock(id)`, read back and print; parser entry
`max-torque <id> <pct>` (pct 0–100). No other behaviour changes.

Bus access route: the Teensy owns the four leg buses in normal operation — to write
EPROM, temporarily plug each leg's Waveshare driver chain into the USB bench adapter
exactly as in P2 calibration (one leg at a time, robot on cradle, Teensy side
disconnected). If P3 ended with a different handover mechanism (e.g. firmware
passthrough), use that instead — the register and value are the same.

```
cd /home/barq/barq_ws/src/diagnostics
./st3215_diag.py max-torque 0 30     # repeat for IDs 0..11
./st3215_diag.py status 0            # confirm: max_torque 30%
```

| Stage | Limit | What it must do | Switch to next when |
|---|---|---|---|
| A | 30 % | cradle stance hold; legs movable by firm hand pressure (yields, no grinding) | §3–§5 clean on cradle |
| B | 60 % | cradle hold + first floor lower (§6); mild buckling under full weight is acceptable here | floor lower attempted, no oscillation |
| C | 100 % | full stand, G4.1/G4.2 | gates |

Verify the limit is honored at stage A: with torque on, push one knee gently with a
rod (not fingers) — it must yield well below full stall force. If it does not yield,
the register is not being honored → TBD row 1, stop and resolve before stage B.
Restore `max-torque <id> 100` on all 12 at the end of this file (and in Rollback).

### 3. Stance hold on the cradle (each torque stage)
```
ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=true
# second terminal — record everything, every run (C++ recorder, not live rclpy):
ros2 bag record -o stand_$(date +%H%M) /barq/diag /joint_states /imu/data
# third terminal:
ros2 topic echo /barq/diag
```
Do NOT publish `/cmd_vel`: with no command, the planner streams the neutral stance —
this IS the stand configuration. Hold 60 s. Watch: smooth motion to stance, no buzz,
no fault bits other than nothing (bit3 only appears pre-activation).

### 4. Per-leg load symmetry (the imbalance metric)
`/barq/diag` values [0..11] are per-servo load in %, ID order (FL,FR,RL,RR ×
coxa,femur,tibia). Over ≥ 10 s of samples define per leg:

```
L_leg = mean over samples of ( |load_femur| + |load_tibia| ) / 2
I_front = |L_FL − L_FR| / max(L_FL, L_FR) × 100 %      (same for I_rear)
```

**Bar: I_front < 30 % AND I_rear < 30 %.** Compare LEFT vs RIGHT only —
front-vs-rear asymmetry is BY DESIGN (D-016 deliberately loads the front feet;
record the front share `(L_FL+L_FR)/ΣL` as data, expected > 50 %, exact band TBD row 4).
On the cradle (feet free) all loads should be small and roughly equal; the metric
matters on the floor (§6 onward). If a lateral pair exceeds 30 % on the FLOOR:
check zero offsets of that pair first (ladder B below), then re-measure corner
heights (§7) — a height skew and a load skew together point at one leg's zero.

### 5. Oscillation triage + temp watch (cradle, before any floor work)
Visible buzz / limit-cycle around the hold pose. **Do NOT start by reducing the 100 Hz
command rate** — at hold the command stream is constant-valued, so the stream rate
cannot excite a limit cycle; the loop that oscillates is inside the servo or in the
mechanics. Ladder, in order, one change at a time:

1. **zero_offsets**: torque off one leg, set it to a known pose against a protractor,
   compare `/joint_states`. A wrong zero makes IK fight the clamp (`ANKLE_MIN −2.2`)
   or load a joint statically → re-run P2 `calibrate-mid` for that servo.
2. **Servo internal gains**: the STS series has P/D gain registers in EPROM —
   addresses NOT yet mapped in our tooling (TBD row 2). Procedure to fill the TBD:
   dump `0x00–0x45` with the `bus.read` primitive on the bench, diff against the
   Waveshare/Feetech STS3215 memory table, add the addresses as constants to
   `st3215_diag.py`, then reduce P in steps of 10 % on the buzzing servo only.
3. **Mechanical backlash**: horn screws, bracket flex, foot mount — re-torque,
   re-check. (Backlash + high internal gain is the classic buzz pair.)

Switch criteria: 2 failed attempts per rung or 1 hour → next rung. Oscillation
unresolved after the ladder → do not raise the torque stage; park per Escape hatch.

**Temp watch**: 10 min stance hold on the cradle at stage B. Log `/barq/diag`[14]
(temp_max, °C — note: max across all 12, the protocol carries only the max) once
per minute. Bar: plateau — rise < 2 °C over the final 3 min, absolute < 50 °C.
To find WHICH servo is hot: post-run `st3215_diag.py status` sweep on the bench
adapter, or IR thermometer; log it.

### 6. Lower to the floor (stage B, then C)
Spotter kneels with one hand flat under the belly (not fingers under feet). Lift the
robot off the cradle in stance (torque on), lower until all four feet touch the pad
area, keep the hand bearing ~half the weight for 2 s, then withdraw slowly over ~3 s.
Master switch operator (second person, or the spotter's free hand) stays on the switch.
- Buckling at 60 % → expected; go to 100 % and repeat.
- Buckling at 100 % → NOT a torque problem (sim proves 5–10× stance margin) →
  fallback ladder rung C below (vbus sag / zero offsets), do not push.

### 7. Settle height + attitude vs model (G4.2 measurement)
Datum: **floor → centre of the coxa (hip) axle**, all four corners, steel rule or
caliper depth rod, robot standing free and still. Model: hip height = stand_height
0.130 + foot sphere r 0.012 (hip z-offsets ≈ 0.0002, ignore) →

| Corner pair | Expected | Bar |
|---|---|---|
| FL, FR | **0.142 m** | each within ± 0.010 m |
| RL, RR | **0.162 m** | each within ± 0.010 m |
| rear − front mean delta | 17 mm | 11–23 mm (= pitch 4.5° ± 1.5°, see below) |
| left − right mean delta (roll) | 0 | < 5 mm (≈ 1.3°) |

Pitch = atan(Δ_rear−front / 0.2169) (hip x-spacing 0.108484 + 0.108371). Geometric
trim predicts 5.3° (gait_planner comment); the sim *settled* at 4.5° (D-016, and the
lidar counter-wedge confirmed 4.5° in physics) — the bar 4.5° ± 1.5° covers both;
record the measured value, it is the hardware twin of the sim's 0.2 mm settle-z metric.
Cross-check pitch/roll against `/imu/data` (P4-CODE-1) — tape vs IMU disagreement
> 1.5° means the BNO085 mount-axes mapping is wrong (fix before P4-03 estimator work).

### 8. Backward-tip check (the D-016 trim is FOR this)
Observe 60 s of free stand: any rearward rocking, rear-foot load climbing in
`/barq/diag`, or a backward tip on small disturbances (light fingertip nudge at the
top-centre of the body, ~5 N, both directions). If it still tips/rocks backward:
- rear_raise ladder **+0.005 m per rung**, one rung per run, re-measure §7 each time:
  `gait:=false`, then
  `ros2 run barq_control ik_node --ros-args -p rear_raise:=0.025` (and the matching
  `-p rear_raise:=0.025` on `gait_planner` when walking later — keep the two equal).
- Cap at 0.030 (pitch ~6.8°) — beyond that, STOP: the trim is compensating for a CoM
  surprise; execute the Q-014 CoM measurement (balance the robot on a rod under the
  belly, both axes, battery installed; record coordinates) and bring the data to the
  URDF (`base_link` inertial origin) instead of trimming further.

### 9. Battery-sag observation (feeds the P1 brownout table)
From `/barq/diag`[12] (vbus, V), record: (a) idle on cradle torque ON, (b) free stand
on floor, (c) during the §6 lowering transient (bag playback). Compute Δsag = (a)−(b)
and worst transient dip. Floor policy: ABORT the session if vbus < 13.6 V resting
(00 §3); firmware warns at 13.8 (fault bit2), hard cutoff 13.2 (P3). Enter all three
numbers in the P1-02 brownout TBD table AND the research log.

## Acceptance gates
- **G4.1 — Free stand**: 60 s free standing (no cradle, no hands), zero visible
  oscillation, temp_max ≤ 50 °C with the §5 plateau already demonstrated,
  I_front < 30 % and I_rear < 30 %, no fault bits, vbus ≥ 13.6 V. 2 consecutive runs.
- **G4.2 — Settle height & attitude**: all §7 bars met (corner heights ± 10 mm,
  pitch 4.5° ± 1.5° nose-down, roll < 1.3°).

## Fallback ladder
- **A — won't reach stance pose cleanly (lurch/grind on cradle)**: re-check §1
  pre-pose; drop to 30 % torque and watch which joint fights → zero offset of that
  servo (P2 re-calib). Switch: 2 attempts.
- **B — load imbalance ≥ 30 % or height skew on floor**: zero-offset audit of the
  worst pair (§5 rung 1); then swap-test the servo with a spare (00 §4 rule 4).
  Switch: 2 attempts each.
- **C — buckles at 100 % torque**: check vbus during the buckle in the bag (brownout
  → P1-01/P1-02 buck sizing, not a P4 problem); check temp_max (thermal derating);
  then re-run P3 air-walk to confirm tracking still 9/9. Switch: 2 attempts or
  2 hours, whichever first — then Escape hatch.
- **Oscillation**: dedicated ladder in §5 (zero offsets → internal gains TBD →
  mechanical). Never raise torque stage while oscillating.

## Rollback
Restore `max-torque <id> 100` × 12 (EPROM persists!); robot back on the cradle;
params back to defaults (no rear_raise override left running); re-run the P3 air-walk
check to prove nothing regressed; log the session even if rolled back.

## TBD table
| # | Unknown | Procedure that produces it |
|---|---|---|
| 1 | Does EPROM_MAX_TORQUE actually limit output dynamically on ST3215 FW in hand? | §2 stage-A yield test with a rod; if not honored, find the RAM torque register via the §5-rung-2 EPROM dump and re-spec P4-CODE-2 |
| 2 | STS3215 internal P/D gain register addresses | §5 rung 2 dump + vendor memory table diff |
| 3 | Exact CoM coords, battery installed (Q-014) | §8 rod-balance measurement → URDF inertial origin |
| 4 | Normal front-load share band at stance | §4 measurement at G4.1, 3 runs → research log |
| 5 | vbus idle/stand/transient sag | §9 → P1-02 brownout table |

## Artifacts → docs/05_RESEARCH_LOG.md
Bag files per run; §4 load numbers + imbalance %; §7 four corner heights + computed
pitch/roll vs IMU; §8 rear_raise rung results (if any); §9 sag numbers; torque-stage
at which each behaviour appeared. One research-log entry per session (standing
practice): believed → measured → changed.

## Escape hatch
Stuck ≥ 2 sessions on G4.1/G4.2: re-prove the highest green seam (P3 air-walk), write
the blocker into `docs/04_OPEN_QUESTIONS.md` with every measured number, park P4, and
advance a parallel phase (P6-01/02 RL env spec or P5-01 lidar decision — 00 §5). Do
NOT improvise a plan D the same day (00 §1.4). Next file after G4.1+G4.2: run the
safety drills **first** (`03_ESTIMATOR_AND_SAFETY.md` §6, gates G4.6/G4.7) — they are
cradle-level and MUST pass before the floor walking in `02_FIRST_STEPS.md`.
