# AGENTS.md

## Repo Context

- This repository is Team 18650's FTC SDK fork for robot code used for the Decode season, and originally developed for Into the Deep.
- The codebase is primarily owned by Callam, lead student programmer.
- Eben, a recent team alumnus, sometimes contributes technical help.
- Peter, the head coach, occasional helps work through problems.
- Much of the code is organic, hand-written, and iteratively refactored over roughly the last 24 months. Expect uneven maturity across subsystems.

## Current Operating Mode

- During competition, optimize for reliability, debuggability, and match readiness.
- Prefer targeted fixes over broad refactors.
- Avoid style-only churn, speculative architecture cleanup, and risky rewrites unless explicitly requested.
- When a subsystem was recently rebuilt, assume regressions are more likely to come from integration gaps, state handling, lifecycle ordering, hardware assumptions, or missed edge cases than from isolated syntax mistakes.

## Project Layout

- `TeamCode/` is the main team-owned code and the default place for edits.
- `FtcRobotController/` mostly tracks the upstream FTC SDK app and should only be changed for a clear reason.
- `doc/` and `.github/` contain supporting documentation and workflow guidance.

## Tech Stack

- Android Studio Ladybug-era FTC SDK project
- Gradle multi-module Android build
- Main modules: `:TeamCode` and `:FtcRobotController`
- Primary language: Java

## Working Norms For Agents

- Preserve student ownership. Make the smallest change that materially improves robot behavior or code safety.
- Read surrounding code before editing; this repo contains ongoing refactors and partially migrated patterns.
- Do not assume abstractions are fully finished just because they look framework-like.
- Treat hardware wrappers, opmode lifecycle code, teleop task flow, and autonomous sequencing as high-risk areas that deserve extra review.
- If there are uncommitted changes, do not revert them unless explicitly asked.
- Keep comments brief and only where they clarify non-obvious behavior.

## Reliability Priorities

- Favor fixes that reduce the chance of deadlocks, null state, stale cached hardware state, unexpected control handoff, task cancellation bugs, and telemetry-silent failures.
- Prefer explicit guards, sane fallbacks, and clearer failure behavior over cleverness.
- For match-day changes, call out anything that still needs on-robot validation.

## Review Priorities

- Start reviews with the most recent commits and the files touched by active subsystem rewrites.
- Look first for behavioral regressions, incorrect assumptions about FTC lifecycle timing, hardware initialization mistakes, state persistence issues, and task sequencing bugs.
- Be skeptical of changes affecting:
  - `hardware/`
  - `core/`
  - `core/teleoptasks/`
  - `components/mechanisms/`
  - `components/subsystems/`

## Build And Validation

- Prefer narrow validation first, then broader checks if needed.
- Useful commands:
  - `.\gradlew.bat :TeamCode:assembleDebug`
  - `.\gradlew.bat lint`
- A green build is not enough for robot-facing changes; note what still requires driver station or on-robot testing.

## Communication

- Be direct and concrete.
- When recommending a fix, explain the expected robot behavior change and the risk if left unfixed.
- If time is short before matches, prioritize the few changes most likely to improve reliability tomorrow.
