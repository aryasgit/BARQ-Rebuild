# BARQ Support Docs

Living documentation for the BARQ quadruped project. **These are maintained continuously** —
updated after every iteration, change, decision, and choice.

| File | Purpose | Update cadence |
|---|---|---|
| [00_OVERVIEW.md](00_OVERVIEW.md) | The durable reference: mission, hardware topology, control architecture, verified geometry, staged plan. | When the plan or architecture changes. |
| [01_STATUS.md](01_STATUS.md) | Where we are *right now* — current stage, done / in-progress / next. | Every working session. |
| [02_DECISIONS.md](02_DECISIONS.md) | Decision log (ADR-style). Every choice with its rationale. | Whenever a decision is made. |
| [03_CHANGELOG.md](03_CHANGELOG.md) | Dated log of concrete changes to the repo. | After every change. |
| [04_OPEN_QUESTIONS.md](04_OPEN_QUESTIONS.md) | Ambiguities & pending decisions awaiting input. | When raised / resolved. |

## Conventions
- Dates are absolute (YYYY-MM-DD).
- Decisions get an ID (`D-001`, …) and are referenced from the changelog and code comments.
- Open questions get an ID (`Q-001`, …); when resolved they move to `02_DECISIONS.md`.
- Geometry / conventions never live here as source of truth — that is `barq_description/config/`
  and `barq_description/urdf/`. These docs *point to* it.
