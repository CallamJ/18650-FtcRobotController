---
name: review-recent-commits
description: Review recent commits in this FTC team-code repo to find likely regressions, quality issues, risky integration changes, and drift between live code and AGENTS.md. Use when Callam or mentors want a focused review of recent branch history, match-day changes, subsystem rewrites, opmode behavior changes, or documentation drift caused by ongoing refactors.
---

# Review Recent Commits

## Overview

Review recent git history as a probe into the live robot code, not just as a diff summary. Start from the changed commits, follow the affected execution paths into the active opmodes and subsystem coordinators, then report concrete findings and any places where `AGENTS.md` no longer matches reality.

## Workflow

1. Define the review window.
- Use the user-specified range when provided.
- Otherwise prefer a narrow recent window such as `HEAD~5..HEAD`, `HEAD~10..HEAD`, or the branch range since the last known stable base.
- Start with:
  - `git log --oneline --decorate -n 12`
  - `git diff --stat <base>..<head>`
  - `git diff --name-only <base>..<head>`

2. Identify high-risk files first.
- In this repo, prioritize changes under:
  - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/`
  - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/implementations/`
  - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/teleoptasks/`
  - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/components/subsystems/`
  - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/components/mechanisms/`
  - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/`
  - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/pedroPathing/Constants.java`
- Treat `MainTeleOp.java` and `AutoOpBase.java` as likely execution hubs even if they were not directly edited.

3. Use changed commits as entry points into live behavior.
- Do not stop at the changed lines.
- Follow each risky diff into:
  - the opmode that constructs or calls it
  - the subsystem or state machine it coordinates
  - the hardware wrapper or persistent state bridge it depends on
- In this repo, explicitly check for impact on:
  - opmode lifecycle ordering
  - gamepad edge handling
  - per-opmode hardware caching
  - `MatchStateStore` persistence
  - `PersistentStorage` keys and defaults
  - state-machine transitions in storage/firing code
  - Pedro follower vs non-follower drive behavior

4. Look for repo-specific failure patterns.
- Prefer findings in these categories:
  - shared or conflicting keybinds
  - stale static subsystem references across opmode restarts
  - framework hook misuse, especially if internal setup leaks back into `on...` overrides
  - null-tolerant initialization that later crashes in loop code
  - queue/state-machine fallthrough
  - persistence mismatches between teleop and auto
  - use of old abstractions after wiring moved elsewhere
  - tuning/config fields that no longer affect the live code path
  - telemetry or diagnostics that now lie or silently omit failures
- Be skeptical of refactors that rename or split classes but leave old mental models in place.

5. Compare code reality against `AGENTS.md`.
- Read `AGENTS.md` after identifying the changed execution paths, not before.
- Check whether `AGENTS.md` is wrong, incomplete, or misleading about:
  - primary entrypoints
  - active control surfaces
  - current state machines
  - persistent state bridges
  - subsystem ownership boundaries
  - which opmodes are the real source of truth
- Report drift only when you can point to the code that contradicts the doc.

6. Validate as far as the environment allows.
- If local build tooling is available, prefer a narrow compile or validation step after identifying likely issues.
- If compilation is blocked, say so explicitly and continue with source review.

## Repo-Specific Reading Order

Use this order when the recent diffs touch multiple systems:

1. `AGENTS.md`
2. changed commits and diff stats
3. `MainTeleOp.java`
4. `AutoOpBase.java`
5. changed subsystem/mechanism/hardware files
6. `MatchStateStore.java`
7. `PersistentStorage.java`
8. `TeleOpTaskContext.java`, `TeleOpTaskManager.java`, and active task classes when teleop automation is involved

## Findings Standard

Lead with findings, ordered by severity. For each finding include:

- file reference
- the concrete risky behavior
- why the recent commit likely introduced or exposed it
- the likely robot-facing effect or maintenance risk

After findings, add:

- `AGENTS.md drift`
  Only list real mismatches or missing documentation.
- `Open questions`
  Use when the code suggests multiple plausible intents.
- `Validation gaps`
  Note anything that still needs compile, driver-station, or on-robot confirmation.

If there are no findings, say so directly and still mention any drift or remaining blind spots.

## Command Hints

Prefer fast local commands:

- `rg --files`
- `rg -n "<pattern>" TeamCode/src/main/java`
- `git log --oneline --decorate -n 12`
- `git diff --stat <base>..<head>`
- `git diff <base>..<head> -- <path>`
- numbered file reads for precise references

Avoid broad theory. Use the commits to choose where to dig.

## Keep This Skill Current

- Update this skill when the repo's main control surface changes, especially if `MainTeleOp`, `AutoOpBase`, teleop tasks, storage managers, or persistence bridges are replaced or re-owned.
- Update the repo-specific failure patterns when new recurring bug classes appear.
- If `AGENTS.md` gains or loses major sections, keep the drift-check guidance aligned with that structure.
