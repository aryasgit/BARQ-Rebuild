# P6-03 — Training Recipes (PPO, per compute track)

> Phase P6 · verified against repo @ 4ea53a0

## Objective
Turn the P6-02 env into a policy that passes the in-sim gates: **G6.4** (eval suite) and **G6.5**
(beats the classical gait in the SAME env). This file is the per-track PPO recipe, the knob
discipline for when training fails, and the run-management rules that make any conclusion
citable. Sim-only — nothing here touches the robot.

## Prerequisites
- G6.1 (track + skeleton), G6.2 (parity), G6.3 (classical replay) passed. The G6.3
  realized-speed number is on record — it is the G6.5 baseline **in this env**.
- Budget caps, kill criteria and the run-card format are P6-01 §2/§5; this file does not re-cap.
- Trainer installed and its STOCK example verified before importing BARQ (separates "install
  broken" from "our env broken"): Track A/B = rsl_rl as shipped with Isaac Lab (or with the
  MuJoCo adapter if 01 §2 chose that path); Track C = sb3 **or** cleanrl — pick ONE, pin
  versions in `barq_rl/PINNED.md`, verify at execution time.

## 1. Ground rules (all tracks)
1. **One knob per run** (Protocol §2). Reward weights start at the 02 §5 table; every delta is
   recorded in the run card's resolved config (that's what `config_sha256` hashes).
2. Train with DR + obs noise ON (02 §8). Evaluate with DR pinned at the current
   `actuator.yaml`/nominal centers, obs noise still ON (§6) — deployment has noise.
3. Running/empirical obs normalization **OFF by default** — the 02 §3 scale column IS the
   normalization, and fixed scales deploy with zero artifacts. If you turn it ON (a legitimate
   knob for the NaN row in §4), it must be baked into the ONNX graph at export (P6-04 §2) and
   flagged in the run card.
4. Timeout = truncation with value bootstrap (02 §6). Verify the trainer flag on day one — this
   bug produces plausible-but-biased policies and no error message.
5. Smoke-test 5 minutes locally before anything runs at ₹/hr (01 §1: never debug rewards on a
   rented GPU).

## 2. Track A/B recipe (rsl_rl-class PPO)
Starting table (names per rsl_rl; map 1:1 if using another trainer — verify exact keys at
execution time):

| knob | start | small variant | notes |
|---|---|---|---|
| num_envs | **4096** | 2048 | fits a mid-RTX easily (01 §2) |
| rollout steps/env | **24** | 24 | batch = 98,304 |
| learning epochs | **5** | 5 | |
| minibatches | **4** | 4 | 24,576 samples each |
| lr | **1e-3, adaptive, KL target 0.01** | same | rsl_rl `schedule=adaptive, desired_kl=0.01` |
| gamma / lambda | **0.99 / 0.95** | same | |
| clip | **0.2** | same | |
| entropy coef | **0.01** | same | |
| value coef / clipped v-loss | 1.0 / on | same | rsl_rl defaults — verify |
| grad-norm clip | 1.0 | same | |
| net (actor & critic MLP) | **[512,256,128] ELU** | **[256,128]** | actor ≈ 190k params at full size |
| init action std | 1.0 | 1.0 | |
| obs normalization | OFF (§1.3) | OFF | |
| first run length | 1500 iters ≈ 147 M steps | — | first-walk band 100–200 M (01 §2) |

Healthy-run vitals while it trains: KL hovers near 0.01 (the adaptive lr is doing its job);
entropy decays slowly, no cliff in the first 10 %; value loss spikes at each curriculum advance
then settles; mean episode length climbs toward timeout by mid-stage-0.

**Which knob first when it fails** (after the §4 table names the signature — one per run):
1. Not a knob: the §4 "first check" column — termination histograms and reward-share plots are
   data, look before touching anything.
2. The matching §4 reward-weight row.
3. lr 1e-3 → 5e-4 (or KL target 0.01 → 0.008) if KL is spiky / updates look destructive.
4. entropy 0.01 → 0.005 if the policy never commits to a gait; → 0.02 if it collapsed to one
   behavior in the first 10 %.
5. rollout 24 → 48 if value targets look noisy (credit over longer horizons).
6. Net to the small variant LAST — capacity is almost never the problem at 12 DoF.

## 3. Track C recipe (sb3 or cleanrl, CPU MuJoCo on the Mac mini)

### 3.1 Throughput probe FIRST — gate G6.6
Do not start a training run before this number exists.
1. `barq_rl/tools/throughput_probe.py`: (i) random-policy batched stepping, 60 s → raw
   env-steps/s; (ii) a 5-minute real `learn()` with the §3.2 config → **effective** env-steps/s
   (collection + update; this is the number that matters).
2. Layout sweep, one variable at a time: single-process batched env loop (preferred — wrap the
   vectorized MuJoCo step as one VecEnv; 128 separate processes on a Mac mini drowns in IPC) vs
   SubprocVecEnv 8×16; env count 64 / 128 / 256. Record the table.
3. Arithmetic honesty: at 3k steps/s, 10 M steps ≈ 56 min, 50 M ≈ 4.6 h — that is 01 §7's
   "hours-to-a-day per run". At <1k, 50 M is 14 h+ and the iteration loop dies — rescope.

**G6.6** bar below. Rescope ladder on fail: 64 envs (cache locality) → rollout 1024 → 256
(faster update cadence; doesn't change steps/s but shrinks memory stalls) → net [64,64] →
**escalate to Track A** at the pilot cap (extends 01 §6's C→A criterion: "CPU can't afford it"
includes can't afford the throughput).

### 3.2 PPO starting table
| knob | start | notes |
|---|---|---|
| envs | **128** | per §3.1's winning layout |
| n_steps (rollout/env) | **1024** | batch 131,072 — long rollouts amortize update overhead on CPU |
| minibatch | 4096 | 32 minibatches/epoch |
| epochs | 5 | watch KL drift across epochs at this batch size |
| lr | 3e-4 constant | sb3/cleanrl have no KL-adaptive lr; set sb3 `target_kl=0.02` as an early-stop guard only — honest difference vs Track A/B |
| gamma / lambda / clip / entropy | 0.99 / 0.95 / 0.2 / 0.01 | |
| value coef | 0.5 | sb3 default |
| net | **[128,128]** | record the activation you used — export must match (P6-04 §2) |
| total steps | **10–50 M** per run | expect several runs |
| device | CPU | MPS is often slower for a [128,128] MLP (01 §4) — benchmark once, move on |
| normalization | VecNormalize OFF (§1.3) | |

### 3.3 Honest expectations
Curriculum capped at stage 1 + pushes (01 §4, 02 §7 "C-narrow"); target = a conservative blind
policy at vx 0.10–0.15. Days-to-weeks of wall-clock across iterations. G6.4 is then evaluated
**on the trained envelope** (run card states it); G6.5 is unchanged — 0.15 cmd is inside C's
envelope, so the baseline fight is still fair.

## 4. Failure-signature table (read top-down, fire ONE row per run)
| signature (curves, §5) | first check (data, not knobs) | knob order |
|---|---|---|
| reward flat from step 0, ep-len short and flat | **termination spam**: histogram termination causes; init jitter (02 §8) vs termination bands; height/orientation terms against the CAPTURED g* (02 §2.6) — a hand-typed pitch sign makes every episode die at reset | fix termination band / g* capture first — it's a bug, not a hyperparameter |
| ep-len = timeout, reward flat, robot stands and won't walk | reward-share plot: alive+height share of positive reward > ~60 %? curriculum log: did stage advance before its err bar was met? cmd sampler: too many near-deadband commands? | alive 0.25 → 0.15; air-time weight 1.0 → 1.5; re-pin curriculum at stage 0 (too-aggressive advance is "stands forever" in disguise) |
| tracking ↑ but torque penalty magnitude insane / actions ride the ctrlrange stops | per-joint |τ| histogram; which joints saturate | torque weight 2e-4 → 5e-4 (×2.5 steps); then action-rate 0.01 → 0.02 |
| walks in sim, then NaNs (loss or obs) | scaled-obs extrema (q̇ × 0.05 should live in ±1; spikes ≫ = contact/solver instability — timestep/solver BEFORE hyperparameters) | clip scaled obs to ±10; lr 1e-3 → 5e-4; only then consider running normalization (and accept the P6-04 §2 export consequence) |
| joints buzz/vibrate at stand | FFT of action stream; is the 02 §4 two-tick interpolation actually on? | action-rate 0.01 → 0.03; joint-accel ×4 |
| pronks/hops instead of trot | per-foot air-time histogram | air-time weight 1.0 → 0.7 (or target 0.2 → 0.25 s); foot-slip weight up |
| vx tracks, yaw drifts in eval | cmd sampler coverage of wz; ang tracking term value | ang-vel weight 0.5 → 0.8 |

Re-run on the SAME seed first (isolates the knob), then fresh seeds per §5.

## 5. Run management (what makes a result a result)
- **Run card** (P6-01 §5 format, verbatim) committed before a run is declared done; plus one
  line in `training_runs/INDEX.md`: `id | track | steps | wall-clock | result one-liner | gates`.
  Config hash, seed, machine, cost — all in the card already; don't fork the format.
- **3-seed rule**: any conclusion that changes a default (a reward weight, a knob, "Track C is
  enough") needs 3 seeds. Gate claims need the gate passed on the seed you ship PLUS same-shape
  curves from 2 more (shorter runs are fine as the supporting pair).
- **Checkpoints**: every ≤ 30 min wall-clock (A/B: rsl_rl `save_interval` ≈ 50 iters at this
  scale; C: every 1 M steps). Keep `latest` + `best-by-eval`. The resume drill happens BEFORE
  the first long run (01 §2 requires it for spot instances; do it on B/C too).
- **TensorBoard curves that matter** (and the §4 row they point at):

| curve | healthy | sick → |
|---|---|---|
| eval tracking error (m/s and %) | < 25 % by mid-stage-1 | rows 1–2 |
| mean episode length | climbs to timeout; sawtooth at curriculum advances is normal | row 1 |
| torque-penalty share of Σ\|terms\| | < ~25 % | row 3 |
| KL | pinned near target (A/B) / < 0.02 (C) | lr knob |
| entropy | slow decay | entropy knob |

- **Obs percentile snapshot**: `evaluate.py` (§6) writes `obs_stats.npz` (per-dim p1…p99 of the
  final policy's obs stream) — run-card artifact; P6-04 §8 consumes it for the hardware gap
  analysis. Not optional: without it the sim2real loop is blind.

## 6. Evaluation protocol — `barq_rl/tools/evaluate.py` (gates G6.4 / G6.5)
Deterministic action (distribution mean), DR pinned nominal, obs noise ON, fresh reset per item.
Envelope = the TRAINED envelope (Track C: stage-1 caps; the run card says which).
1. **Cmd grid**: vx ∈ {0, ±0.10, ±0.15, ±0.25*} × wz ∈ {0, ±0.3, ±0.8*} plus vy ∈ {±0.08} spot
   cells (* only if trained). 20 s per cell; error averaged over t = 2–20 s. Tracking error =
   ‖v − v_cmd‖/‖v_cmd‖ for ‖cmd‖ ≥ 0.05; the zero-cmd cell is judged absolute: |v| < 0.03 m/s.
2. **Endurance**: 25 episodes × 20 s, commands resampled each episode (= 500 s).
3. **Push battery**: 10 lateral impulses (0.5 m/s A/B, 0.3 C-narrow) at stand + 10 at vx 0.10.
   Recovery = no fall within 3 s AND lin-vel tracking term back above 0.5 within 2 s.

## Acceptance gates
- **G6.4 — sim eval suite.** Grid mean tracking error **< 20 %** with no cell > 30 %; **0 falls
  in the 500 s** endurance; push recovery **≥ 80 %** (≥ 16/20). Recorded per run card; this is
  the gate 01 §6's switch criteria and escape hatch count.
- **G6.5 — beats the classical gait in the SAME env.** vx 0.15 cmd, 10 s × 5 seeds (the exact
  G6.3 replay protocol): mean realized **> 0.09 m/s** AND > the recorded G6.3 classical-replay
  number (whichever is higher binds). 0.09 = the gait's ~60 % at duty 0.6 (Q-013/D-019) — the
  policy must beat the thing it replaces where that thing actually lives.
- **G6.6 — Track C throughput (run FIRST on Track C).** Effective (learner-in-loop) rate
  **≥ 3,000 env-steps/s** on the best §3.1 layout, else rescope/escalate per §3.1. (Gate id out
  of execution order because G6.4/G6.5 were already bound by references in 01/02.)

## Fallback ladder
A: §4 knob ladder, one row per run → B: simplify to a known-trainable core (small net +
temporarily drop foot-slip and joint-accel terms; get ANY stable walk, then re-add one term per
run) → C: reduce scope to the C-narrow envelope — a slow blind policy that passes G6.4 at
stage 1 is shippable to P6-04 with the envelope documented → D: switch compute tracks per
01 §6. Switch rungs after 2 failed runs or 2 sessions, whichever first.

## Rollback
Sim-only. Rollback = check out the configs of the best previous run card (cards are immutable;
checkpoints live outside git per 01 §5). Nothing deployed changes in this file.

## TBD table
| # | Unknown | Procedure |
|---|---|---|
| 1 | S (env-steps/s) per machine | 01 §2 10-min probe (A) / §3.1 probe (C) → run cards |
| 2 | N steps to first-walk / to G6.4 for OUR env | TB curves of the first passing runs |
| 3 | Final reward-weight deltas vs the 02 §5 starts | diff of resolved configs across cards |
| 4 | Track C env count where steps/s saturates | §3.1 sweep 64/128/256 |

## Artifacts → docs/05_RESEARCH_LOG.md
Per session: run ids + one-liners; which §4 row fired and the measured before/after; the three
G6.4 numbers (grid %, falls, push %); the G6.5 number against the G6.3 baseline; `obs_stats.npz`
path. Commit cards + INDEX, push (Protocol §1).

## Escape hatch
If 3 full-budget runs (01 §2 caps) or 3 weekends of Track C produce no G6.4 — 01 §6's bypass
trigger — stop tuning. Write the best curves and the dominant failure signature into the
research log, park RL, and make `05_NO_RL_BYPASS.md` the phase's main line. Keep the env, the
cards, and `obs_stats.npz`: a future GPU week resumes from the cards, not from memory. A policy
that passes G6.4 only on a reduced envelope is NOT this hatch — that one proceeds to P6-04 with
its envelope stated.
