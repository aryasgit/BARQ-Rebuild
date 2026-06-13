# 00 — Doomsday Protocol: how to execute this roadmap without an LLM

> verified against repo @ 4ea53a0 · applies to every phase folder

This file is the operating system for the rest of the roadmap. It exists because the plan must
survive the loss of the assistant that wrote it. Everything here is discipline, not cleverness.

## 1. The execution loop (per work session)
1. **Pick the next unchecked gate** in phase order (`appendices/B_ACCEPTANCE_GATES.md`).
2. Read that phase file fully BEFORE touching hardware. Stage the listed prerequisites.
3. Execute the numbered steps exactly. Where a step says MEASURE, write the number down
   immediately (phone note is fine, transcribe later).
4. Test against the gate. **Pass** → tick it, record artifacts (below), move on.
   **Fail** → run the file's fallback ladder. Never improvise past a failed gate on the same
   day you failed it — sleep on plan C before inventing plan D.
5. End of session: update `docs/05_RESEARCH_LOG.md` (what was believed / what was measured /
   what changed), `docs/01_STATUS.md` one-liner, commit, push. The git history is the team's
   shared memory now — treat an unpushed session as work that didn't happen.

## 2. Rules that already saved this project (learned in sim, they transfer)
- **One change at a time, measured before and after.** The μ-sweep "null result" solved Q-013;
  it only worked because friction changed while everything else was pinned.
- **Verify at the lowest layer that can lie.** We grepped the engine's SDF, not our URDF
  (D-018). Hardware twin: verify at the servo register / scope level, not the ROS topic level.
- **Exit codes lie under load; logs and telemetry don't.** Check the log tail, the STATE
  frame, the measured number (sim build #4 false success; nav goals silently lost).
- **A null result from a sweep is evidence.** Invariance to a parameter eliminates a whole
  hypothesis class — design tests so that "nothing changed" MEANS something.
- **Between-run state is part of the experiment.** Fresh spawn / fresh power-cycle per
  measured run, or you are comparing garbage (the mid-stride-teleport fiasco).
- **Record overridden decisions, not just final ones** — that trail is the publication.

## 3. Safety absolutes (hardware phases)
- **Torque-off must always be one action away**: master switch within arm's reach (P1-01),
  and the firmware deadman (200 ms, fault bit3) is never to be disabled, even "temporarily".
- New pose / new gait / new policy → **cradle first** (robot suspended, feet free), then
  floor with a spotter hand, then free. No exceptions, including "it worked in sim".
- LiPo: charge supervised, in a LiPo bag, charger profile VERIFIED against pack chemistry
  (4.20 vs 4.35 V/cell — P1-01 procedure); storage-charge if idle > 3 days; floor 13.6 V.
- Fingers out of the leg workspace whenever torque is on. The tibia fold (D-012) is a
  pinch hazard by design.
- Smell/heat/swelling → power off, outdoors, wait 30 min. No heroics.

## 4. When you are stuck (the meta-fallback)
1. Reproduce the failure twice. If it won't reproduce, it's state — power-cycle everything
   (battery, Teensy, container) and re-run from a known gate.
2. `appendices/A_FAILURE_TREES.md` — find the symptom, walk the ladder.
3. Bisect the chain at its seams. The architecture has clean test points by design:
   bench servo alone (`st3215_diag.py`) → Teensy loopback (`integration_pty.py` on a PTY) →
   Teensy real (`integration_pty.py` on /dev/ttyACM0) → controllers (`real.launch.py`) →
   gait → autonomy. The failure is between the last green seam and the first red one.
4. Swap-test components (spare servo, spare cable, second UART bus) before blaming code
   that has a passing test.
5. If truly blocked ≥ 2 sessions: write the blocker up in `docs/04_OPEN_QUESTIONS.md` with
   everything measured, park it, and advance a parallel phase (the phase graph below).

## 5. Phase dependency graph (what can proceed in parallel)
```
P0 ──► P1 ──► P2 ──► P3 ──► P4 ──► P5 ──► P7
              │                    ▲
              └── P6 (RL: env+training can start any time after P0;
                       sim2real needs P3's bench ID; deployment needs P4)
```
Waiting on parts? Advance P6-01/02 (RL env spec runs in sim) or P5-01 (lidar decision).

## 6. Definition of "fully functioning" (the finish line)
All of: (a) untethered walking under `/cmd_vel` with the classical stack, straight-line and
turns, repeatable across 10 consecutive runs; (b) autonomous indoor A→B with lidar SLAM +
nav2, self-recovering; (c) EITHER a deployed RL policy beating the classical gait on the
P6-04 regression table OR the no-RL bypass ladder (P6-05) adopted through at least rung 2
(body-velocity feedback); (d) the P7 runbook in actual use (two consecutive field sessions
executed from checklists alone).
