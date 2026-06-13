# P7-02 — Operations Runbook: the daily driver

> Phase P7 · verified against repo @ 4ea53a0

## Objective
Make every field session boringly repeatable: one PRE-RUN checklist, one RUN block, one POST-RUN
close-out, a CRASH tree you execute instead of improvising, and a maintenance clock that catches
wear before it becomes a fall. The doomsday criterion lives here: **G7.6 = two consecutive
sessions run start-to-finish from these checklists alone.** Print this file or keep it open on
the phone; the checklists are written to be ticked standing next to the robot.

## Prerequisites
- [ ] P4 closed (stands and walks untethered), P5 closed for autonomy sessions.
- [ ] Master switch installed within arm's reach (P1-01); firmware deadman active (never
      disabled — Protocol §3).
- [ ] Reference stance photo from P2 printed/taped inside the transport box AND saved on phone.
- [ ] Witness marks painted once at P2 assembly (paint-pen stripe across each horn screw head
      onto the horn). If never done: do it now at a bench session, robot torqued off.
- [ ] Mac side: `~/.barq-channel.sh` sourced (barq-get works from a LOCAL Mac terminal).
- [ ] A charged 4S pack (resting ≥ 15.4 V), LiPo bag, voltage checker, tape measure, phone.

### Session constants (memorize these five numbers)
| What | Value | Source |
|---|---|---|
| Battery floor (abort) | **13.6 V** (3.4 V/cell) | P1 |
| Battery storage | 15.2–15.4 V (3.80–3.85 V/cell) | P1 |
| Servo temp warn / hard-stop | **60 °C / 65 °C** | P4 |
| Nav speed ceiling (until raised per P7-01) | 0.10 m/s | P5 |
| Firmware deadman / gait deadman | 200 ms / 1 s | 06_PROTOCOL |

---

## 1. PRE-RUN checklist (★ = never skippable, even in a hurry)

**Battery & power**
- [ ] ★ Resting voltage ≥ 15.4 V (full session) — 14.8–15.4 V = short session only, < 14.8 V stop.
- [ ] ★ Pack inspect: no puff, no balance-lead nicks, no connector heat marks. Fail any → retire
      pack to the LiPo bag outdoors, use spare.
- [ ] ★ Master switch test: switch OFF → servo rail dead (no servo hold torque by hand); ON →
      rail live. One flip each way.

**Fasteners (the loosen-first list, in order — witness marks)**
- [ ] Horn center screws ×12: witness stripes aligned? (these back out first — reversing torque)
- [ ] Horn-to-bracket perimeter screws: stripes aligned on the 4 highest-load legs' brackets.
- [ ] Foot fasteners ×4: hand-wiggle, no click.
- [ ] Hip bracket-to-chassis: hand-wiggle each hip, no play.
- [ ] Any stripe broken → re-torque by feel (snug + 1/8 turn, small driver, no white knuckles),
      repaint stripe, note joint in the session log line.

**Cables**
- [ ] Visual pass: 4 servo bus daisy-chains seated; JST/bus leads no copper showing; no cable
      inside a leg's fold path (D-012 pinch zone); lidar lead strain-relieved.

**Bench boot (robot on cradle or stood on bench, spotter rule applies)**
- [ ] Power on → Jetson boots (≈ 1 min) → from Mac: `ssh barq@barq.local`.
- [ ] Container up: `~/run_barq.sh` then `source ~/barq_ws/install/setup.bash`.
- [ ] Stack up: `ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=true`
- [ ] ★ Integration ping (one command, second terminal in the container):
      `timeout 12 ros2 topic hz --window 50 /joint_states`
      PASS = steady rate, no gaps. FAIL → ladder F1 below. (Full 9-check fallback:
      `ros2 run barq_hw integration_pty.py` proves the stack minus the Teensy device.)
- [ ] ★ Estimator sane at stance: robot standing, untouched 30 s →
      `ros2 topic echo --once /odom_est` at 0 s and 30 s: position delta < 0.02 m, fault-free.
- [ ] ★ Fault byte clean (bit0 bus / bit1 IMU / bit2 power / bit3 deadman all 0) — check the
      telemetry topic (TBD below) or the barq_hw log banner.
- [ ] Ctrl-C the launch, master switch OFF, carry robot to the course start (carry by chassis).

| TBD | How to fill |
|---|---|
| Telemetry topic exposing vbus/current/temp_max/fault | named by the P3/P4 exporter (expected `/barq/telemetry`); write it here + in the bag set when known |
| Real integration ping for /dev/ttyACM0 (`integration_pty.py --device` variant) | P3-04 delivers it; until then the /joint_states hz ping above is the bench gate |

### Fallback ladder F1 — bench boot fails
- **A.** No /joint_states → check device exists: `ls -l /dev/ttyACM0`. Missing → reseat USB,
  power-cycle Teensy. 2 attempts max.
- **B.** Device present, still silent → seam-bisect (Protocol §4): emulator first —
  `ros2 run barq_hw teensy_emulator` + `real.launch.py device:=<PTY>`. Emulator green →
  fault is Teensy/firmware/cable → P3 failure tree. Emulator red → container/build problem →
  rebuild: `colcon build --symlink-install && source install/setup.bash`.
- **C.** Estimator drifts at stance → joints disagree with reality → run the re-zero quick check
  (CRASH step 6) before anything else. Still bad → P4 estimator tree. **No mission today.**

---

## 2. RUN procedure (start order is the order — do not resequence)

1. [ ] Robot at start mark, area clear, spotter briefed (hands off unless a fall is underway).
2. [ ] ★ Master switch ON → Jetson boot → `ssh barq@barq.local` → `~/run_barq.sh`.
3. [ ] The whole stack, one block (terminal 1 = stack, terminal 2 = bag + mission;
       `tmux` keeps both alive if Wi-Fi drops — recommended):

```
# T1 — control stack (+ autonomy when the mission needs it, per P5 bringup):
source ~/barq_ws/install/setup.bash
ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=true
#   autonomy sessions: ALSO start lidar + SLAM + nav2 per the P5 bringup file. NO RViz
#   on the robot — ever (2d compute lesson). RViz lives on the Mac if needed.

# T2 — logging ON BY DEFAULT (size-capped splits), then the mission:
source ~/barq_ws/install/setup.bash
ros2 bag record -o ~/barq_ws/bags/$(date +%Y%m%d_%H%M%S)_run \
  --max-bag-size 1073741824 \
  /joint_states /imu/data /odom_est /cmd_vel /foot_targets /tf /tf_static /scan /map /plan
```

4. [ ] ★ **Stand gate** (every boot, before any motion command): robot rises to neutral stance
       at zero /cmd_vel and holds 10 s — no oscillation, no joint buzzing, no lean vs the
       reference photo, fault byte 0. Fail → torque off (master switch), ladder F1-C.
5. [ ] Walk-off check (manual sessions): 2 m straight at 0.08 m/s, stop, stance steady.
6. [ ] Mission (autonomy sessions) — send, then verify by TELEMETRY, not the client exit code:

```
# T2 — example 2-waypoint mission (Course-3 pattern). P5's robust mission client is
# preferred when present; raw send_goal is the fallback and its exit code is NOT proof:
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [ {header: {frame_id: map}, pose: {position: {x: 4.5, y: 0.0}}},
             {header: {frame_id: map}, pose: {position: {x: -3.6, y: 2.4}}} ]}"
# PROOF the goal is live = /plan refreshing + /cmd_vel nonzero + robot walking.
# Goal "accepted" with no /cmd_vel within 20 s = silently lost (2d) -> resend ONCE,
# then treat as failure and bag-stop normally.
```

7. [ ] Hands off during missions. Count recoveries; spot for genuine falls only.
8. [ ] End: Ctrl-C bag FIRST (clean close), then the stack; master switch OFF.
9. [ ] Mid-session abort triggers (any → end the run now): vbus < 13.6 V · temp_max ≥ 60 °C ·
       fault bit set · smell/heat/swelling (→ pack outdoors 30 min, Protocol §3).

---

## 3. POST-RUN checklist
- [ ] ★ Battery out → measure resting V, write it down → storage-charge to 15.2–15.4 V
      (supervised, LiPo bag) unless next session < 3 days AND pack ≥ 15.4 V.
- [ ] Temp-trend note: session max temp_max + which conditions (course, minutes walked).
      One line per session in the log — this table is the early-failure radar (see §5).
- [ ] Horn screw witness marks: quick scan; any broken stripe → re-torque + repaint + note.
- [ ] Bags: `ros2 bag info ~/barq_ws/bags/<latest>` closes clean; then offload **from a LOCAL
      Mac terminal** (not inside ssh — else it copies Jetson→Jetson):
      `barq-get barq_ws/bags/<session_dir> -d ~/barq_data/`
      Offload verified → prune Jetson bags oldest-first whenever `df -h /` shows < 10 GB free
      (keep at least the last 3 sessions on the Jetson regardless).
- [ ] ★ Research-log line in `docs/05_RESEARCH_LOG.md` (believed / measured / changed), plus
      walking-minutes tally for the maintenance clock (§5). Commit + push (Protocol §1.5 —
      an unpushed session didn't happen).

---

## 4. CRASH RESPONSE tree (execute top-down; do not skip; do not reorder)

```
ROBOT FELL / CRASHED
 ├─1 Is it still powered & twitching?
 │    fault bit3 (deadman) usually beat you to torque-off.
 │    EITHER WAY: ★ MASTER SWITCH OFF NOW. Do not grab a powered leg.
 ├─2 PHOTO in place (pose + surroundings) BEFORE touching. 10 seconds, always worth it.
 ├─3 Battery out. Sniff + palm test. Hot/puffed/smell -> outdoors 30 min, retire pack.
 ├─4 Damage checklist (torque off, by hand, in this order):
 │    [ ] horns: hairline cracks, wiggle each horn on its spline (loosen-first parts)
 │    [ ] brackets: cracks at screw bosses, bent flanges
 │    [ ] wires: pinched in a fold joint? copper showing? connector half-out?
 │    [ ] lidar: lens scratch, mast bent, connector seated
 │    [ ] chassis: lid/standoffs, Jetson + Teensy seated, buck wires intact
 ├─5 Anything broken -> swap from SPARES (§6), witness-mark new screws, note in log.
 ├─6 ★ RE-ZERO CHECK before the next run (P2-03 quick variant):
 │    robot on cradle (or held), power on, stack up gait:=true, zero cmd ->
 │    neutral stance commanded. Compare EVERY joint visually against the reference
 │    stance photo. Any joint visibly off (> ~5 deg) -> STOP -> full P2-03 re-zero
 │    on the bench. All match -> proceed.
 └─7 Next run starts at the stand gate (RUN step 4), then a 2 m walk-off BEFORE any
      mission. Log the crash: cause if known, photo filename, parts touched.
```
A crash with no known cause after the bag review = open question, not a shrug: file it in
`docs/04_OPEN_QUESTIONS.md` before the next session.

---

## 5. MAINTENANCE schedule (clock = cumulative walking-minutes from §3 log lines)

**Every 5 run-hours (or after any crash):**
- [ ] Full fastener pass: ALL witness marks inspected + renewed; loosen-first list (§1) torqued.
- [ ] Spline/backlash hand check, per joint, torque off: hold the proximal link, wiggle the
      distal link. Compare against the P2 back-drive feel baseline (new = barely perceptible
      play, smooth resistance). A clunk or visible free angle → horn screw, then spline wear →
      swap horn; re-check; persists → swap servo (spare) and bench-test the suspect
      (`diagnostics/st3215_diag.py` step metrics vs its P2 record).
- [ ] Cable chafe inspection: hip crossings, fold-joint paths, lidar mast. Re-dress + sleeve
      anything shiny.

**Every 20 run-hours:**
- [ ] Servo temp-trend review: plot/eyeball the §3 temp notes. Any servo family trending
      ≥ 5 °C above its own earlier baseline at similar load = early-failure signal → move that
      servo to a low-load joint or swap to spare; bench-test it.
- [ ] Battery capacity spot-check: runtime-to-floor trend from the session log (minutes walked
      vs start voltage). A pack delivering < ~70% of its early-life minutes → retire from field
      duty to bench-supply duty.
- [ ] Re-run the P7-01 Course 1 regression row (3 runs) — drift in those numbers is the
      whole-robot health metric.

---

## 6. SPARES list (keep stocked; ₹ are estimates — fill the TBD with real purchase prices)

| Item | Qty | ~₹ |
|---|---|---|
| Servo horns + horn screw sets | 4 horns | 400 |
| M2/M3 screws + nylocks assortment | 1 box | 400 |
| Servo-bus / JST leads (pre-crimped) | 6 | 300 |
| **Spare ST3215 servo (minimum one, bench-calibrated per P2 in advance)** | 1 | 2,800 |
| Spare 12 V high-current buck (same model as P1-01) | 1 | 1,200 |
| Fuses for the 4S main + Jetson branch | assorted | 200 |
| Paint pen (witness marks), tape, zip ties, foam | — | 200 |
| **Total** | | **~5,500** |

| TBD | How to fill |
|---|---|
| Actual spare prices + suppliers | first restock order; record in this table |
| Spare-servo pre-calibration record | run P2 bench calib on the spare NOW, store its zero/ID; a spare you can't drop in is not a spare |

---

## 7. TRANSPORT (any move beyond carrying to the next room)

- [ ] ★ Torque off (master switch), THEN fold by hand:
- [ ] Fold pose: tibias to full fold q3 → −2.2 rad (inside the D-012 verified range — never
      force past it), femurs tucked so feet sit under the belly, body lies flat on its belly pad.
- [ ] ★ Battery OUT, in the LiPo bag, transported separately from the robot box.
- [ ] Lidar lens covered (cap or cloth + rubber band).
- [ ] Box with foam; robot belly-down; no load on the legs; reference stance photo in the lid.
- [ ] On arrival: PRE-RUN checklist from the top (transport counts as "unknown state").

---

## Acceptance gates

### Gate G7.6 — the doomsday criterion
- [ ] Two **consecutive** field sessions executed start-to-finish from §1–§3 alone: every ★ box
      ticked, no step improvised, no step skipped, no external help (human or LLM).
- [ ] Evidence per session: ticked checklist (photo or paper), bag offloaded to the Mac,
      research-log line pushed.
- [ ] Audit: the OTHER team member (or you, next day) replays the evidence against this file
      and finds no gaps. Any improvised step → the gate resets to zero sessions AND the step
      that forced improvisation gets fixed in this file (that edit is the deliverable).

### Gate G7.7 — crash recovery, zero improvisation
- [ ] One crash recovery executed exactly per the §4 tree: photo exists, damage checklist
      ticked, re-zero check done, next run reached the stand gate.
- [ ] No real crash by the time G7.6 passes? **Stage a drill**: from stance on a mat, master
      switch off, tip the robot over by hand, then walk the tree end-to-end as if real.
      A drill pass counts — the tree must be exercised before it's needed at 9 PM with a
      half-dead battery.

### Fallback ladder (G7.6/G7.7)
- **A.** A session needed improvisation → fix the checklist the same evening (add/clarify the
  step), reset the streak, go again. The document converges; that's the mechanism.
- **B.** Checklist fatigue (sessions skipped because "too heavy") → trim to the ★ items only as
  the legal minimum, record the trim as an override in the research log. ★ items themselves are
  never trimmed — they are the Protocol §3 safety absolutes wearing field clothes.
- **C.** Same pre-run check fails 3 sessions running → it's not ops anymore, it's a phase
  regression: reopen the owning phase (F1 ladder tells you which) and pause field work.

## Rollback
This file changes no code. Operational rollback = the CRASH tree (§4) + re-zero check; document
rollback = `git checkout -- docs/roadmap/phase-7-field/02_OPERATIONS_RUNBOOK.md`. If a checklist
edit made sessions worse, revert the edit and log why — overridden decisions are publication
material (research-log standing practice).

## Artifacts → docs/05_RESEARCH_LOG.md
- One line per session (date, course/work, minutes walked, start/end V, temp_max, anomalies).
- Crash entries: cause, photo filename, parts replaced, re-zero verdict.
- Maintenance entries at each 5 h / 20 h service: what was found loose/worn — this trend data
  is the reliability section of the paper.
- G7.6/G7.7 verdict lines + tick `appendices/B_ACCEPTANCE_GATES.md`.

## Escape hatch
If the robot is needed for a demo TODAY and a non-★ item is failing: run the ★-only minimum,
keep speed ≤ 0.08 m/s, missions ≤ 5 min, spotter mandatory, and write the skipped items in the
log before the demo, not after. If a ★ item is failing, there is no demo — a ★ failure is the
robot telling you it will fall, and it is always cheaper to believe it on the bench than to
film it on the floor.
