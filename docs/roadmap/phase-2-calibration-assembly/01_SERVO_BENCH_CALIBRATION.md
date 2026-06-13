# P2-01 — Servo bench calibration (all 12 × ST3215, pre-assembly)

> Phase P2 · verified against repo @ 0e5ddaf

## Objective
Every one of the 12 ST3215 servos is individually ID'd, mid-calibrated, proven mechanically and
electrically healthy, and characterized (step response + sweep) **before** anything is screwed to
a leg. Output: one committed calibration YAML per servo + a batch roll-up table. Budget ~10 min
per servo (≈ 2.5–3 h for the batch including records). Nothing in this file requires the Teensy,
ROS, or the printed frame.

**Why characterize now:** the bench step/sweep numbers are the SAME metrics the sim emits
(`diagnostics/sim_actuation_probe.py` — rise time 10–90 %, peak velocity, overshoot, tracking
error). D-018's sim servo stiffness (k=60/s) is *provisional until bench ID*; P3-03 re-tunes k
against the data you record here. Sloppy records now = a sim that lies later.

## Prerequisites
- P0/P1 bench: Waveshare Serial Bus Servo Driver board on USB (`/dev/ttyACM*` / `/dev/ttyUSB*`,
  Mac `/dev/tty.usbserial-*`), servo power from the bench supply at **12.0 V** — never from USB.
  - ⚠ **Contradiction on record:** `diagnostics/README.md` says "own 7.4 V class supply
    (ST3215 rated range)". That line is stale. The project's spec basis is **12 V** (4.71 rad/s
    @ 12 V, D-018) and the roadmap power decision states the ST3215/driver max is 12.6 V
    (hence the 12 V buck in P1). Bench at 12.0 V so bench metrics share the sim's voltage
    basis. Before first power-up, VERIFY the voltage range printed on your servo's label /
    datasheet covers 12 V; the servo's own EPROM volt window is read back in step 5.
- `pip3 install pyserial`. Tool: `/home/barq/barq_ws/src/diagnostics/st3215_diag.py`
  (run from `~/barq_ws/src/diagnostics`; add `--port <dev>` if auto-detect grabs the wrong port).
- 12 servos unboxed but unmounted. **No horns, no linkages** during bench tests.
- Label maker (or tape + marker), multimeter, note of ambient temperature.
- `mkdir -p ~/barq_ws/artifacts/servo_calib`

## The ID map (authoritative — `ID_PLAN` in st3215_diag.py ≡ `robot_params.yaml servos:`)
Print it any time with `./st3215_diag.py plan`. Bus = leg (one driver board / one Teensy UART
per leg later); chain order on the bus is hip → knee → ankle.

| Bus (leg) | Servo name | ID | URDF joint     | yaml `direction` (hypothesis, verified in P2-03) |
|-----------|------------|----|----------------|---|
| FL        | FL_coxa    | 0  | FL_hip_joint   | +1 |
| FL        | FL_femur   | 1  | FL_knee_joint  | +1 |
| FL        | FL_tibia   | 2  | FL_ankle_joint | +1 |
| FR        | FR_coxa    | 3  | FR_hip_joint   | −1 |
| FR        | FR_femur   | 4  | FR_knee_joint  | +1 |
| FR        | FR_tibia   | 5  | FR_ankle_joint | +1 |
| RL        | RL_coxa    | 6  | RL_hip_joint   | +1 |
| RL        | RL_femur   | 7  | RL_knee_joint  | +1 |
| RL        | RL_tibia   | 8  | RL_ankle_joint | +1 |
| RR        | RR_coxa    | 9  | RR_hip_joint   | −1 |
| RR        | RR_femur   | 10 | RR_knee_joint  | +1 |
| RR        | RR_tibia   | 11 | RR_ankle_joint | +1 |

Bench work is always in the **raw servo frame** (counts: 4096/rev, center 2048, 0.0879 °/count —
constants from the script). `zero_offset`/`direction` are P2-03's business.

## Procedure (repeat per servo, in ID order 0 → 11)

1. **Unbox-inspect.** Case cracks, bent connector pins, spline damage; rotate the output shaft
   by hand through a turn — note grit/notchiness; check axial end-play by feel. Record the
   box/case serial marking (or "none").
2. **Connect SINGLY** — exactly one servo on the bus (IDs are assigned blind; two servos with
   the same ID corrupt each other forever after). Power on; multimeter at the board: 12.0 V.
3. **Scan.**
   ```
   ./st3215_diag.py scan
   ```
   Expected: `ID   1  ALIVE` then `1 servo(s): [1]` (factory ID is 1). Nothing found → full-range
   sweep `./st3215_diag.py scan --max-id 253` (some units ship with other IDs). Still nothing →
   DOA path (reject criteria below).
4. **Assign the BARQ ID.**
   ```
   ./st3215_diag.py set-id 1 <ID>
   ```
   Answer `y`; expected `ID <ID>: ALIVE - done`. **Label the case NOW** (`<name> / ID<ID>`,
   e.g. `RL_femur / ID7`). An unlabeled calibrated servo is scrap time.
5. **Status snapshot (records the as-found EPROM).**
   ```
   ./st3215_diag.py status <ID>
   ```
   Record into the YAML: volts (expect 11.8–12.2 — cross-check the multimeter; >0.5 V apart is a
   reject flag), temp ≈ ambient, `mode 0 (0=position)`, the as-found `angle limits [lo, hi]`,
   `ofs`, `max_torque`, and the line `temp/volt limits: max X C, [lo, hi] V`. These are read from
   the servo's own EPROM — they are the restore reference (T2.4) and the only register values
   you may quote (never invent them; anything else: verify against the Feetech datasheet).
6. **Mid-calibration.**
   ```
   ./st3215_diag.py calibrate-mid <ID>
   ```
   Torque drops; hand-set the shaft anywhere comfortable and press ENTER. Expected:
   `calibrated. present position now 204x (want ~2048)`; the tool exits nonzero if
   |pos−2048| ≥ 10 → redo. *What this is actually for:* pre-assembly the bare shaft has no
   mechanical meaning — the point is that **2048 = this shaft pose** and the sensor wrap point
   (0/4095) now sits 180° away. P2-02 attaches the horn AT 2048 in the URDF-zero orientation, so
   the wrap lands diametrically opposite the leg workspace (largest design excursion is the
   tibia at −126°, comfortably clear of ±180°). Re-read `status` and record the new `ofs`.
7. **Hold-and-rock (backlash screen).**
   ```
   ./st3215_diag.py move <ID> 2048
   ./st3215_diag.py monitor <ID> --hz 10
   ```
   With torque holding, gently rock the output shaft between two fingers (alternating light
   torque, no tools). Record the peak-to-peak counts excursion from the live `pos` field.
   This is a *comparative* screen — threshold comes from the batch (TBD T2.1), not a datasheet.
8. **Step-response capture (the sim-k cross-match, D-018).** Two native-speed steps, +60° and
   back (speed 0 / acc 0 = uncapped, the same convention the script's sweep defaults use —
   physical units of those registers: verify against the Feetech datasheet, T2.3):
   ```
   stdbuf -o0 ./st3215_diag.py move <ID> --deg 60 --speed 0 --acc 0 --timeout 3 \
     | stdbuf -o0 tr '\r' '\n' \
     | python3 -u -c 'import sys,time; t0=time.time(); [print(f"{time.time()-t0:.3f},{l.rstrip()}") for l in sys.stdin]' \
     | tee ~/barq_ws/artifacts/servo_calib/step_id<ID>_up.log

   stdbuf -o0 ./st3215_diag.py move <ID> --deg 0 --speed 0 --acc 0 --timeout 3 \
     | stdbuf -o0 tr '\r' '\n' \
     | python3 -u -c 'import sys,time; t0=time.time(); [print(f"{time.time()-t0:.3f},{l.rstrip()}") for l in sys.stdin]' \
     | tee ~/barq_ws/artifacts/servo_calib/step_id<ID>_down.log
   ```
   Convert each log to CSV (`t,pos,spd` — the regex matches the tool's live-line format exactly):
   ```
   sed -n 's/^\([0-9.]*\),.*pos \+\([0-9]\+\).*spd \+\(-\?[0-9]\+\).*/\1,\2,\3/p' \
     ~/barq_ws/artifacts/servo_calib/step_id<ID>_up.log \
     > ~/barq_ws/artifacts/servo_calib/step_id<ID>_up.csv
   ```
   Extract by hand from the CSV: **rise time 10–90 %** (up-step: t between pos crossing 2116 and
   2663; down-step: between 2663 and 2116), **peak |spd|** (raw units until T2.3), **overshoot**
   (counts beyond 2731 / below 2048), final `err` from the tool's last line.
   **Known resolution limit:** the tool samples at ~20 Hz (0.05 s loop), so rise time carries
   ±50 ms uncertainty; a 60° native step is ≈ 178 ms theoretical 10–90 % at the 4.71 rad/s spec
   → expect 3–5 in-flight samples. That is good enough to (a) screen outlier servos and
   (b) bracket the sim k (sim @ k=60/s: 50 ms rise on a 0.3 rad step — `docs/02_DECISIONS.md`
   D-018). The *fine* system ID happens in P3-03 over the Teensy's 100 Hz STATE stream; here you
   only archive clean, comparable per-servo data.
9. **Sweep.**
   ```
   ./st3215_diag.py sweep <ID>
   ```
   Default ±30°, 3 cycles, 0.25 Hz (~12 s; peak commanded velocity ≈ 0.82 rad/s, far below the
   4.71 cap). Watch the live line and record: end-of-run `worst tracking error N counts`,
   max |load %|, max mA, temp before/after. Healthy: smooth motor tone, no grinding/clunks,
   load roughly symmetric in both directions, lag-while-moving is normal (the tool says so).
   Optional second pass `--amp-deg 60` if the first is clean (more travel exercised). Use the
   SAME flags for all 12 — batch comparability is the whole point.
10. **Limits behavior (demo only — protective limits are written in P2-03, after direction and
    zero_offset are known; writing asymmetric limits now, blind to direction, is how you brick a
    range).**
    ```
    ./st3215_diag.py limits <ID>                        # as-found (recorded in step 5)
    ./st3215_diag.py limits <ID> --min 1844 --max 2252  # ±18° test window
    ./st3215_diag.py move <ID> --deg 60 --timeout 2     # must STOP near 2252; large final err = the clamp working
    ./st3215_diag.py limits <ID> --min <as-found lo> --max <as-found hi>   # RESTORE
    ./st3215_diag.py limits <ID>                        # read-back: restored
    ```
11. **Power down handling.** `./st3215_diag.py torque <ID> off` → unplug → store in a labeled
    slot. Never store/handle with torque on.
12. **Record.** Fill the servo's YAML (schema below) and append its row to the batch roll-up.

## Calibration record (committed — gate G2.2)
One file per servo: `barq_description/config/servo_calib/servo_<ID>_<name>.yaml`
(e.g. `servo_07_RL_femur.yaml`). Schema (all fields required; raw servo frame):

```yaml
# P2-01 bench record. Raw servo frame (counts, 2048 = mid). URDF-frame zero_offset/direction
# live in robot_params.yaml (written by P2-03, not here).
id: 7
name: RL_femur
label: "RL_femur / ID7"          # exactly what is written on the case
serial_no: "<case/box marking, or 'none'>"
date: 2026-06-XX
supply_v: 12.0
ambient_c: 24
firmware_asfound:                 # from step 5 status — restore reference (T2.4)
  limits: [0, 4095]               # as printed, do not assume
  ofs: 0
  mode: 0
  max_temp_c: 0                   # the EPROM value status prints — never invented
  volt_window: [0.0, 0.0]
mid_offset: 0                     # EPROM ofs counts AFTER calibrate-mid (step 6 re-status)
calib_mid_pos: 2048               # present position right after calibrate-mid (want 2048±10)
rock_pp_counts: 0                 # step 7 peak-to-peak
step:                             # step 8 (±50 ms resolution; raw spd units until T2.3)
  rise_ms:   { up: 0, down: 0 }
  peak_spd:  { up: 0, down: 0 }
  overshoot_counts: { up: 0, down: 0 }
  final_err_counts: { up: 0, down: 0 }
sweep:                            # step 9
  amp_deg: 30
  worst_err_counts: 0
  max_load_pct: 0
  max_ma: 0
  temp_start_c: 0
  temp_end_c: 0
direction_observed: null          # filled by P2-03 (bench can't know — no link attached)
eprom_limits_written: null        # filled by P2-03 step 5
result: PASS                      # PASS | REJECT
notes: ""
```

**Roll-up table** (one row per servo: id · rise up/down · peak spd · sweep worst err · rock pp ·
temp rise · mA max · result) goes into `docs/05_RESEARCH_LOG.md` as the P2-01 entry. This table
*defines* the batch medians used by T2.1/T2.2 — the thresholds are comparative, not invented.

## TBD table (measure, never guess)
| TBD | Value | Measurement procedure |
|---|---|---|
| T2.1 | backlash reject threshold (counts p-p) | after all 12: median + spread of `rock_pp_counts`; investigate > 1.5× median; reject > 2× median **and** audible clunk/grind |
| T2.2 | temp-rise bar | provisional judgment: flag if ≥ 50 °C absolute or rise > 15 °C in one default sweep; confirm against batch median rise + the EPROM max-temp read in step 5 |
| T2.3 | physical units of speed/acc registers & spd feedback | Feetech STS datasheet; until then record raw units (peak velocity in rad/s only after this lands) |
| T2.4 | as-found EPROM limits/ofs per servo | step 5 — restore reference |
| T2.5 | no-load sweep current envelope (mA) | batch median/max from step 9; the datasheet stall figure: verify against datasheet, never from memory |

## Acceptance gates
- **G2.1 — all 12 pass bench.** Every servo: `result: PASS` — instant scan, `calibrate-mid`
  exit 0 (2048±10), smooth sweep with worst-err inside the batch family (no outlier > 2× median),
  no reject criterion hit.
- **G2.2 — calibration YAMLs committed.** 12 files under `barq_description/config/servo_calib/`
  + the roll-up table in `docs/05_RESEARCH_LOG.md`, committed and pushed (D-013 workflow).

## Reject criteria & the swap/spare path
- **DOA:** no reply on `scan --max-id 253` after the isolation ladder (swap cable → swap port →
  prove the setup with a known-good servo).
- **Mechanical:** grinding/clunk in sweep; `rock_pp_counts` > 2× batch median; gross shaft
  end-play vs the batch.
- **Thermal:** T2.2 bar tripped (temp runaway during a 12 s no-load sweep is never OK).
- **Electrical:** reported volts > 0.5 V off the multimeter; sweep current a clear batch outlier.

Path: pull a spare (P0-02 BOM line; if no spares were procured, STOP and order before
assembling anything — a leg torn down later costs ~10× a bench hour), run the spare through this
entire file under the rejected unit's ID. Keep the rejected unit's YAML with `result: REJECT`
(it is evidence) and log the swap in the research log.

## Fallback ladder
- **A** — procedure as written. *Switch when:* the same step fails twice on the same servo.
- **B** — isolate the layer: swap cable → explicit `--port` → known-good servo on the same
  setup → same servo on the other host (Mac, `/dev/tty.usbserial-*`). The fault is between the
  last green and first red swap. *Switch when:* 2 failed attempts or 1 h.
- **C** — drive the servo with the Waveshare vendor demo tool instead. Vendor tool works but
  `st3215_diag.py` doesn't → protocol/firmware-variant bug in our `Bus` class: debug against the
  register map in the script header (and a logic analyzer if available), fix, commit. Neither
  works → hardware (board or servo) → spare path.

## Rollback
Everything this file writes to a servo is reversible: `set-id <ID> 1` returns the factory ID,
`limits` restores from the as-found values (T2.4), `calibrate-mid` can simply be re-run.
Repo side: `git revert` of the YAML commit.

## Artifacts → docs/05_RESEARCH_LOG.md
- Raw step logs/CSVs in `~/barq_ws/artifacts/servo_calib/` (back up to the Mac with `barq-send`).
- 12 committed YAMLs + roll-up table; research-log entry in the standing format (believed /
  measured / changed), `docs/01_STATUS.md` one-liner, commit, push.

## If this entire phase approach fails
If NO servo responds on ANY host/board/cable, vendor tool included (ladder C exhausted): the
procurement is wrong — protocol variant or damaged shipment. Stop P2. Re-verify the exact SKU
against P0-02 BOM ("Waveshare ST3215 serial bus servo, 30 kg·cm class"), RMA the batch, and
advance P6-01/02 (RL env spec, sim-only) or P5-01 (lidar decision) per the dependency graph in
`00_DOOMSDAY_PROTOCOL.md` §5. Under no circumstances assemble unverified servos.
