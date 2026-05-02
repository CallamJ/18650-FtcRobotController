---
name: review-recent-commits
description: Review recent commits in this FTC team-code repo to find regressions, integration risks, and drift between live code and AGENTS.md. Use when Callam or mentors want a focused review of recent branch history, match-day changes, subsystem rewrites, or opmode behavior changes.
---

# Review Recent Commits

## Operating Model

This skill is review *method*. Repo-specific facts — directories, control surfaces, state machines, recurring pitfalls, persistence bridges, toolchain — live in `AGENTS.md`, which is authoritative and auto-loaded in Codex. Treat AGENTS.md as the index this skill reads against.

If you find yourself wanting to add a file path, mechanism name, or pitfall pattern *to this skill*, that fact belongs in `AGENTS.md`. Update AGENTS.md and reference it from here.

## Workflow

1. **Define the review window.**
   - Use the user-specified range when given.
   - Otherwise pick a narrow recent window: `HEAD~5..HEAD`, `HEAD~10..HEAD`, or the branch range since the last stable base.
   - Start with:
     - `git log --oneline --decorate -n 12`
     - `git diff --stat <base>..<head>`
     - `git diff --name-only <base>..<head>`

2. **Identify high-risk paths via AGENTS.md.**
   - Cross-reference the changed files against AGENTS.md's *Project Map*, *Primary Entry Points*, and *Active Control Surfaces*. Anything those sections name is high-signal.
   - Treat the entrypoints AGENTS.md identifies as likely execution hubs even if they were not directly edited.

3. **Follow each risky diff into live behavior.**
   - Don't stop at changed lines. Walk the diff into the opmode that constructs or calls it, the subsystem coordinator it touches, and the hardware wrapper or persistence bridge it depends on.
   - Use AGENTS.md's *Key Non-Obvious Contracts*, *State Machines And Coordinators*, and *Implicit Bridges Between Systems* as the map for what the diff might break.

4. **Check the diff against AGENTS.md's *Repo-Specific Failure Patterns*.**
   - That section enumerates failure modes tied to this codebase's conventions. Run it as a checklist against the changed paths.
   - Apply general heuristics on top of that — race conditions, off-by-one in loop counters, log-and-swallow exception handlers, dead branches — anything not pinned to a specific repo convention is your judgment, not the doc's.

5. **Drift check against AGENTS.md.**
   - Reread AGENTS.md *after* identifying changed execution paths. This pass is for drift, not initial orientation.
   - Drift is a required review deliverable. Either name a concrete mismatch or say "no drift observed."
   - Treat AGENTS.md updates as a longer-cycle documentation task. In review-only work, report drift clearly; only edit AGENTS.md when the user requested documentation updates or when the code change has settled enough that the new shape is no longer speculative.

6. **Validate as far as the environment allows.**
   - Prefer a narrow compile if local tooling is available; see AGENTS.md *Validation Guidance* for current commands.
   - If compilation is blocked, say so explicitly and continue with source review.

## Findings Standard

Lead with findings, ordered by severity. For each finding include:

- file reference
- the concrete risky behavior
- why the recent commit likely introduced or exposed it
- the likely robot-facing effect or maintenance risk

After findings, always include:

- **AGENTS.md drift** — concrete mismatches, or "none observed." Required.
- **Open questions** — when the code suggests multiple plausible intents.
- **Validation gaps** — anything that still needs compile, driver-station, or on-robot confirmation.

If there are no findings, say so directly and still report drift and remaining blind spots.

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

This skill describes review *method*, not codebase facts. It changes only when the way reviews are run changes — not when the code changes.

- If you want to add a file path, directory, mechanism, control surface, or recurring pitfall: put it in `AGENTS.md`, not here.
- Update this skill when the review process itself shifts: a new findings format, a new validation gate, a different drift-check protocol, a change in which AGENTS.md sections this skill references by name.
- If AGENTS.md renames or removes a section this skill references, update the references here in the same change.
