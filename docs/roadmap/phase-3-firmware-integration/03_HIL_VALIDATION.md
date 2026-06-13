# P3-03 — HIL Validation: graduating from emulator to metal

> Phase P3 · verified against repo @ 4ea53a0

## Objective
Prove, in five locked steps, that the P3-01/P3-02 firmware on a real Teensy 4.1 + servos is a
drop-in for the emulator: same 6/6 native tests, same 9/9 integration checks on
`/dev/ttyACM0`, bench step-response identified and the **sim re-tuned to match the hardware**
(not the other way around), then a 60 s cradle air-walk inside quantitative gates. This is the
P3 exit (roadmap README: "integration_pty.py passes on /dev/ttyACM0; air-walk on cradle").

## Prerequisites
- G3.1–G3.4 all green. Robot assembled per P2 (or bench rig for steps 1–2), cradle ready
  (robot suspended, feet free), master switch in reach, spotter for any torque-on work.
- Archived last-good `firmware.hex` in `artifacts/fw/` (rollback is re-flash).
- Two small specced patches from this file applied BEFORE step 2: §patch-A
  (`integration_pty.py --device`) and §patch-B (`sim_actuation_probe.py` sim-time flag);
  §patch-C (barq_hw stats log line) before step 4.

## Specced patches (small, do them as written)
- **patch-A — `barq_hw/test/integration_pty.py`**: add argparse: `--device PATH` (skip the
  emulator `Popen`, use PATH for phase A and pass it to the phase-B launch) and
  `--tol-mrad N` (phase-A/B convergence threshold; default 5 = today's emulator bar).
  Rationale for the relaxed hardware tolerance: the bench truth is `st3215_diag.py cmd_move`,
  whose own pass bar is < 15 counts ≈ 23 mrad; real no-load servos settle within a few counts
  but not the emulator's 3 mrad quantization floor. Nothing else changes — **same 9 checks**.
- **patch-B — `diagnostics/sim_actuation_probe.py`**: line ~47 hardcodes
  `Parameter('use_sim_time', value=True)` — add `--real` to set it False (wall clock; the real
  stack publishes no `/clock`, so without this the probe's clock never advances and it hangs).
- **patch-C — `barq_hw/src/barq_system.cpp`**: in `read()`, a throttled (10 s) INFO line from
  the freshest STATE: `hw: temp_max=__C vbus=__mV cur=__mA fault=0x__`. Grep-able telemetry
  while ros2_control owns the port (state_peek cannot attach concurrently). Real sensor
  topics (IMU broadcaster etc.) are P4 scope.

## Step 1 — regression on the host (nothing may have rotted)  → gate G3.5
```bash
export PATH="$HOME/.local/bin:$PATH"
cd ~/barq_ws/src/barq_firmware && pio test -e native            # MUST: 6/6
cd ~/barq_ws && colcon build --packages-select barq_hw barq_firmware 2>/dev/null; \
  colcon build --packages-select barq_hw                        # host-compiles loop_core.cpp
python3 ~/barq_ws/src/barq_hw/test/integration_pty.py           # MUST: 9/9 (emulator)
```
**LoopCore purity check, precisely:** `pio test -e native` only compiles `protocol.cpp`
(see platformio.ini `build_src_filter`) — the thing that actually enforces "no Arduino types
in loop_core" is the **colcon/CMake host build of `barq_hw` (teensy_emulator compiles
loop_core.cpp verbatim)** plus the 9/9 run. If either fails after the P3-01/02 changes, the
hardware code leaked across the seam — fix the seam, do not `#ifdef` around it.
**G3.5 PASS = 6/6 AND barq_hw builds AND 9/9, all after the full firmware diff.**

## Step 2 — flash and run the SAME 9 checks against metal  → gate G3.6
Bench rig: servos powered from the P1-01 12 V rail, `--bench-power` config build (P3-02 trap),
**start with 3 servos on one bus** (`gen_firmware_config.py --fitted 0,1,2 --bench-power`,
rebuild), then the full 12 loose/partial set.
```bash
cd ~/barq_ws/src/barq_firmware
python3 ~/barq_ws/src/diagnostics/gen_firmware_config.py --fitted 0,1,2 --bench-power
pio run -e teensy41 -t upload                                   # flash over USB
python3 ~/barq_ws/src/barq_hw/test/integration_pty.py --device /dev/ttyACM0 --tol-mrad 25
```
Notes that will save you an afternoon:
- With `-DUSB_DUAL_SERIAL` the Teensy enumerates TWO CDC ports; the protocol is normally the
  first, but VERIFY: `state_peek.py --port /dev/ttyACMx` — the one emitting 100 Hz STATE is
  the protocol port; the other prints the 1 Hz stats text.
- Unfitted servos are firmware-loopback (P3-01 §4), so all 9 checks remain meaningful with 3
  real servos: real ones must track within tolerance, loopback ones are exact.
- **The deadman check now physically drops torque** — when phase A goes silent, the 3 powered
  servos audibly unlatch and go limp ≈ 0.2 s later. Watch it happen; that observation is part
  of the gate evidence (photo/video for the log).
- Phase B launches the real stack (`real.launch.py device:=/dev/ttyACM0 gait:=false` via the
  test) — controllers must activate (requires live STATE; if activation fails, STATE isn't
  flowing: check the port choice and fault byte first).

**G3.6 PASS = the same 9 checks pass on `/dev/ttyACM0` with 3 servos, then again with all 12
fitted (`--fitted` omitted), fault byte 0x00 while driven, plus the observed physical torque
drop on deadman.**

## Step 3 — bench step response through the FULL chain, then re-tune the sim  → gate G3.7
Through ros2_control + real servo (not the diag tool — that was P2's bench-direct baseline):
```bash
ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=false   # terminal 1
python3 ~/barq_ws/src/diagnostics/sim_actuation_probe.py step \
        --joint FL_knee_joint --delta 0.3 --real                            # terminal 2
```
(The probe only needs `/joint_states` + the command topic — both exist on the real stack.)
Capture rise (10–90 %), peak |velocity|, overshoot, settle; repeat ×3 (fresh between runs) and
on one joint of another bus. Compare three columns: sim CSVs (`~/barq_ws/artifacts`, D-018
runs: 50 ms rise on 0.3 rad), P2 bench-direct (`st3215_diag` move/sweep records), and this
full-chain measurement (expect bench-direct + ~10–20 ms of bus/protocol/controller latency).

**Re-tune the sim to the hardware** (`position_proportional_gain`,
`barq_bringup/config/ros2_controllers.yaml`, currently 0.6 = k=60/s, explicitly provisional
per D-018): run the SAME 0.3 rad step in sim, adjust the gain until sim 10–90 % rise matches
the measured hardware rise within ±10 %, re-run the sim trot tracking metric for the record.
Record old/new gain + both rise times in `docs/05_RESEARCH_LOG.md` and mint a decision entry
(next free D-number — D-020 was the last at the time of writing) "sim actuation re-calibrated
to bench" superseding D-018's provisional value.
**G3.7 PASS = 3 consistent hardware step captures (spread < 15 %), CSVs archived, sim gain
re-tuned and matching within ±10 %, D-number recorded.**

## Step 4 — cradle air-walk, 60 s  → gate G3.8
Robot on the cradle (feet free), robot-profile power config if on battery (NOT --bench-power),
spotter ready, master switch in reach:
```bash
ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=true    # terminal 1
ros2 bag record -o /tmp/airwalk /joint_states \
    /joint_group_position_controller/commands                              # terminal 2
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.10}}'                                                  # terminal 3, 60 s
python3 ~/barq_ws/src/diagnostics/analyze_track_bag.py /tmp/airwalk/airwalk_0.db3
```
(Publish `/cmd_vel` continuously at 10 Hz — the gait layer has its own 1 s cmd_vel deadman.)
**G3.8 PASS = all of:**
- Joint-tracking mean RMS from the bag ≤ **2× the sim's 17.8 mrad ⇒ ≤ 35.6 mrad** (per-joint
  peaks noted, not gated, v1).
- Temps: patch-C lines after ≥ 5 min of walking show `temp_max < 55 °C`.
- Current logged per bus: battery current from patch-C + per-servo currents (P3-01 read block,
  ≈6.5 mA units) summed per bus from the debug-CDC stats — paste the 4 numbers into the log.
- **Deadman pull-the-USB test: yank the USB mid-walk → all servos limp in < 300 ms**
  (200 ms deadman + margin). Measure with a 240 fps phone video: frames from cable-out to leg
  free-fall, ÷240. The robot sits/collapses onto the cradle — that is the designed outcome.
- No fault bits except the expected bit3 after the pull; vbus never below alert during the
  minute (else investigate before continuing — TBD-9 debounce data).

## Step 5 — thermal soak, 10 min  → gate G3.9
Immediately repeat step 4's walk for 10 continuous minutes (no cool-down), teeing terminal 1:
`... real.launch.py ... 2>&1 | tee /tmp/soak.log`, then
`grep -o 'hw: temp_max=[0-9]*' /tmp/soak.log` → curve into `artifacts/thermal_soak.csv`
(timestamped by the log lines; debug-CDC per-servo temps identify WHICH servo is hottest).
**G3.9 PASS = temp curve recorded; plateau reached (slope < 1 °C/min over the last 3 min);
absolute `temp_max < 60 °C`; no servo within 10 °C of its EPROM_MAX_TEMP; zero bus errors
accumulated over the soak (counters).** Fail → log the curve anyway (that's the point), then
the fallback ladder.

## Fallback ladders
**Tracking way off in step 3/4 (RMS ≫ gate or any joint visibly wrong):** in order —
1. **zero_offset / direction first (the most likely culprit):** torque off, hand-hold each
   joint at its semantic zero, read STATE pos via `state_peek` — must be ≈ 0 ± 30 mrad with
   the right sign moving positive. Any offender: fix `servo_calib.yaml` / `robot_params.yaml`,
   re-run `gen_firmware_config.py`, re-flash, re-test. (G3.2(b) was the early warning.)
2. **Bus latency / read strategy:** re-run G3.2(c) latency probe; check rx_timeout/retries
   counters during gait; if strategy C or 500 k is active, the +N frames of staleness shows up
   as uniform phase lag across all joints — revisit the P3-01 ladder rung before blaming gains.
3. **Servo internal gains:** the ST3215 position loop has internal P/D registers — **NOT
   exposed by `st3215_diag.py` ⇒ TBD-11: pull addresses from the Feetech datasheet, extend the
   diag script (`status` printout + a `gain` subcommand) on one bench servo first.** Only then
   consider touching them, one servo, one change, measured before/after.
**CRC storms on the harness (counters non-zero during gait but clean on the bench):**
mechanical/electrical ladder — re-seat connectors → shorten/thicken the ground return path
between buck, driver boards and Teensy (single-point ground) → separate signal runs from servo
power leads / add shielding → THEN drop baud (P3-01 ladder A2, with its timing consequence).
Switch criterion per rung: 2 failed re-tests or 2 h.
**Controllers won't activate on hardware:** STATE absent or stale — wrong ACM port, fault
floor trip (bench power profile?), or 100 Hz STATE broken: bisect at the seams
(`st3215_diag.py` single servo → `state_peek` → controllers), per the doomsday meta-fallback.

## Rollback
Any step: re-flash `artifacts/fw/<last-good>.hex`, fall back to the emulator
(`ros2 run barq_hw teensy_emulator` + `device:=/dev/pts/N`) — development continues unblocked.
Step 3's sim gain change is one line in `ros2_controllers.yaml`; if the new k regresses sim
walking (re-run `diagnostics/sim_walk_metric.py`), revert it and record the discrepancy as an
open question instead of forcing it.

## TBD table
| # | Value | Producing procedure |
|---|---|---|
| TBD-11 | ST3215 internal P/D gain registers | datasheet + extend st3215_diag.py, bench-verify on one servo |
| TBD-12 | full-chain added latency (vs bench-direct) | step-3 comparison table |
| TBD-13 | per-bus walking current + thermal plateau | steps 4–5 logs |
| (carried) | TBD-9 alert debounce under trot sag | step-4 vbus trace |

## Artifacts → docs/05_RESEARCH_LOG.md
The 9/9 hardware transcript, deadman video (file name in the log), step-response CSVs ×3 +
comparison table (sim / bench-direct / full-chain), old→new `position_proportional_gain` +
D-number, air-walk bag + `analyze_track_bag.py` output, per-bus currents, thermal CSV + the
hottest-servo identity, all counter snapshots. Update `docs/01_STATUS.md` one-liner and
HANDOFF frontier; commit + push (an unpushed session didn't happen).

## Escape hatch
If metal HIL cannot be stabilized after every ladder here and in P3-01/02: development is NOT
blocked (emulator path stays green), but the **P3 exit gate cannot be waived** — no P4
stand/walk work on hardware until G3.6 + G3.8 pass. Park the blocker with full measurements in
`docs/04_OPEN_QUESTIONS.md`, advance P6-01/02 or P5-01 in parallel (phase graph), and evaluate
the P3-01 escape hatch (Jetson-direct buses) as a formal D-number decision — it changes the
deadman story and must not be adopted casually.
