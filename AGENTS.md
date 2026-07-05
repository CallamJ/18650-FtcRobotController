# AGENTS.md

## Repo Context

- This is Team 18650's FTC SDK fork for Decode, built on top of work that started during Into the Deep.
- Callam is the primary owner and lead student programmer. Most agent help should assume his normal use case: iterating on real robot code, tuning behavior, and fixing integration problems without losing local context.
- Eben and Peter may contribute, but `TeamCode/` is fundamentally a student-owned codebase and changes should respect that style and intent.
- The code is hand-written and heavily evolved over roughly two seasons. Some abstractions are solid, some are mid-refactor, and some are only "real" because the current opmodes use them.

## Default Agent Stance

- Optimize first for helping Callam move the robot code forward safely and quickly.
- Prefer understanding the active opmode and its live wiring over making assumptions from class names alone.
- Treat `MainTeleOp` and `AutoOpBase` as the highest-signal entrypoints for what the robot currently does.
- Make small, behavior-aware edits unless a larger refactor is explicitly requested.
- Do not remove or "simplify" state bridges until you confirm what cross-opmode behavior they preserve.

## Project Map

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/`
  Lifecycle and opmode framework.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/implementations/`
  Real opmodes. This is where current robot behavior is easiest to understand.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/components/`
  Shared building blocks above raw devices: `ActuatorComponent` and the `*AxisComponent` family (`MotorAxisComponent`, `MotorPositionAxisComponent`, `MotorVelocityAxisComponent`, `PositionAxisComponent`, `VelocityAxisComponent`).
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/components/mechanisms/`
  Low-level robot mechanisms: `DriveBase`, `Turret`, `Launcher`, `Hood`, `Indexer`, `Collector`, `FeedRamp`, `FeedWheels`, `LimelightLocalizer`.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/components/subsystems/`
  Coordination layers that sit above mechanisms, especially storage and firing logic.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/`
  Smart FTC hardware wrappers, cache management, and sensor configuration. Includes `hardware/controllers/` (PID family: `PID`, `DirectionalPID`, `VelocityPID`, `GravityPID`, `BangBangController`, `HybridController`) and `hardware/filters/` (`DataFilter`, `RollingAverage`).
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/`
  Drive configuration and Pedro Pathing constants.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/`
  AprilTag readers (`AprilTagReader`, `MultiAprilTagReader`) and `Detection` types used by aiming/localization.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utilities/`
  Persistent storage, match-state carryover, telemetry helpers, and small support types.
- `FtcRobotController/`
  Mostly upstream FTC SDK app code. Avoid edits unless the reason is explicit.

## Primary Entry Points

- `core/implementations/MainTeleOp.java`
  The main match teleop. If a change affects driver controls, firing flow, intake/storage behavior, match-state persistence, or task-assisted teleop, start here.
- `core/implementations/AutoOpBase.java`
  The autonomous framework. Most autos inherit from this and build plans with `StepSpec`.
- `core/implementations/SimpleTeleOp.java`
  Reduced teleop used for simpler mechanism testing and fallback behavior.
- `core/implementations/*Diagnostics*`, `*Tuning*`, `ServoTester`, `MotorIdentifier`, `PotentiometerTest`
  Utility opmodes used for calibration, diagnostics, and hardware bring-up.
- `core/OpModeCore.java`
  The base lifecycle. It owns `Hardware`, `PersistentStorage`, telemetry setup, and the per-loop cache invalidation flow.
- `core/TeleOpCore.java`
  Adds stable edge-detected gamepad handling on top of `OpModeCore`.

## Key Non-Obvious Contracts

- `OpModeCore.frameworkTick()` invalidates hardware caches every loop through `hardware.invalidateCaches()`. Hardware wrappers depend on that refresh cycle.
- Methods beginning with `on` in the opmode framework are hooks. The intended API convention is that hooks do **not** require a `super.on...()` call — `OpModeCore` and `TeleOpCore` deliberately route core setup through framework pre-init plumbing so subclasses never have to chain. `AutoOpBase.onInitialize` currently *violates* that convention: it runs substantive setup (alliance color, drive-base motor config, Limelight init, obelisk-assist state, telemetry plumbing) directly in the hook, so today an auto subclass that overrides it without calling `super.onInitialize()` silently strips that setup. This is a wart, not a pattern to emulate: the durable fix is to move that setup into pre-init plumbing so the super-call requirement disappears, and **do not** add new hooks that require subclasses to chain `super`. Until then, auto subclasses still must call `super.onInitialize()`. When in doubt, read the parent's hook body first.
- `TeleOpCore` snapshots initial gamepad state through framework pre-init plumbing, not through subclass `super.onInitialize()` calls.
- `Hardware` is now per-opmode, not global static state. Do not reintroduce the old `Hardware.init(...)` mental model.
- Several opmodes keep subsystem references in static fields and explicitly reset them on init. That is intentional protection against stale state across FTC opmode reruns.
- `utilities/LiveMatchTuning.java` is the only intended `@Configurable` surface. Keep it scalar-only and match-facing; do not add `@Configurable` back to mechanisms, subsystems, autos, or classes with complex object fields unless the Sloth patch-application cost has been reconsidered.

## Active Control Surfaces

- `core/implementations/MainTeleOp.java`
  Source of truth for the current match control map, teleop task behavior, and how subsystems are actually wired together.
- `core/implementations/AutoOpBase.java`
  Source of truth for how autonomous initialization, firing readiness, obelisk acquisition assist, and path-step execution currently work.
- `drive/pedroPathing/Constants.java`
  Shared follower and drivetrain tuning that affects both teleop follower-assisted behavior and autonomous.
- `utilities/LiveMatchTuning.java`
  Runtime Control Hub tuning for current match-facing scalar values. Autonomous path/plan constants are code-patched rather than exposed as live configurables.

## State Machines And Coordinators

- `components/subsystems/FireControlSystem.java`
  Firing/aiming coordinator. Key fields: `runLauncher`, `fallbackVelocity`, `allianceColor`, `depotAutoAimEnabled`, `firing`, and static `bearingToDepot`.
  State enum: `SEEKING`, `READYING`, `READY`.
  Hidden behavior: it falls back to depot-pose aiming when tags are not visible if configured to do so.
- `components/subsystems/VolleyFireStorageManager.java`
  Queue-driven storage/fire coordinator used by current teleop and auto flow.
  State enum: `RESTING`, `BUMPING`, `PREPARING_TO_FIRE`, `ENGAGING_FEEDER`, `FIRING`, `ENDING_FIRING`.
  Task enum: `READY_FOR_COLLECTION`, bump tasks, and fire-pattern tasks.
  Hidden behavior: it owns jam-correction timing and coordinates `Indexer`, `FeedSystem`, `IndexerStorage`, and `FireControlSystem`.
- `components/subsystems/IndexerStorage.java`
  Logical model of the three storage slots.
  Important fields/concepts: slot content, front-slot freshness, front sensor distance gating, and color classification.
  Hidden behavior: slot identity is tied to the physical indexer position through `Indexer.getNormalizedCurrentIndex()`, so content appears to "move" as the indexer rotates.
- `core/teleoptasks/tasks/FarFiringTask.java`
  Teleop automation state machine for driving to base, waiting for storage fill, returning, and draining shots.
  It depends on `TeleOpTaskContext` suppliers rather than concrete subsystem ownership.
- `core/implementations/AutoOpBase.java`
  Autonomous plan runner. `StepSpec`, `TimeoutAction`, `PlanResult`, and `AutoContext` together form a mini state-machine framework for autos.

## Implicit Bridges Between Systems

- `utilities/MatchStateStore.java`
  Persists alliance, pose, indexer slot content, index targets, turret state, and the latest seen obelisk metadata across opmodes.
  This is a real bridge between runs, not just debugging support.
- `utilities/PersistentStorage.java`
  Backing store for `MatchStateStore` and persisted color tuning. Data survives opmode switches and app restarts through Android `SharedPreferences`.
- Legacy color-tuning paths around `hardware/ColorMatchConfig.java`
  Callam considers this file obsolete. It may still be referenced by wrappers or old tuning opmodes, but agents should not assume it is the current source of truth for live robot behavior unless the active opmode clearly depends on it.
- `core/teleoptasks/TeleOpTaskContext.java`
  Bridges `MainTeleOp` to task automation through suppliers for drive, storage, FCS, alliance, and runtime.
  Tasks may start successfully even when some suppliers later return `null`, so check both the context and the consumer task logic.
- `components/mechanisms/DriveBase.java`
  Has two operating modes: follower-backed when `startFollower` is `true`, and simpler mecanum/localizer mode when `false`.
  That changes which Pedro objects exist and which movement APIs are meaningful.

## Main TeleOp Wiring

- `MainTeleOp` is currently the place where these pieces meet:
  `DriveBase` + Pedro follower, `FeedSystem`, `Indexer`, `Collector`, `IndexerStorage`, `VolleyFireStorageManager`, `FireControlSystem`, `SmartLimelight3A`, `MatchStateStore`, and `TeleOpTaskManager`.
- `MainTeleOp` also owns:
  startup snapshot loading, periodic match-state saving, alliance toggling, maintenance-mode controls, and manual fallback controls when tasks are inactive.
- If behavior seems inconsistent with a subsystem's standalone implementation, trust the live wiring in `MainTeleOp` first.

## Auto Wiring

- `AutoOpBase` owns subsystem setup for autos, plan execution, autonomous alliance selection, and periodic state persistence.
- Child autos typically override `getAutonomousAllianceColor()` and `buildPlan()`.
- Obelisk acquisition assist is handled inside `AutoOpBase`, not inside individual auto plans, unless a child opmode deliberately overrides around it.

## Hardware And Sensor Notes

- `hardware/Hardware.java`
  Per-opmode device registry and cache owner.
- Smart wrappers in `hardware/`
  Often expose both cached state and raw-device access patterns. Read the wrapper before assuming raw FTC device semantics still apply.
- `hardware/ColorMatchConfig.java`
  Treat as legacy/obsolete unless the task is specifically about maintaining or removing old color-tuning paths.
- `frontColorSensor`
  Current storage logic expects distance sensing, not just color sensing.
- `SmartLimelight3A`
  AprilTag visibility affects both aiming and persisted match-state context.

## How To Read This Code Effectively

- Start from the active opmode, then follow concrete subsystem construction from there.
- When behavior is surprising, inspect:
  1. opmode wiring
  2. subsystem coordinator
  3. mechanism
  4. smart hardware wrapper
- Be careful with stale abstractions and renamed classes. Some files represent older patterns that have been partially replaced.
- Do not assume a file is current just because it is widely referenced. Some legacy files remain in support of older wrappers, tuning tools, or unfinished migrations.
- Prefer caller-driven truth over old helper classes that look authoritative but are no longer the main path.

## Repo-Specific Failure Patterns

Failure modes attested by this codebase's commit history. Listed for use as a pre-flight checklist on reviews and edits. Add a new pattern only when the git log supports it; otherwise it's general judgment, not a documented pattern.

- **State-machine cleanup omissions.** Multiple historical fixes have addressed transitions that didn't clear `activeTask`, didn't reset slot content after firing, missed a `break` in a switch arm, missed an `else` for a load-task with nothing to load, or chose a state without triggering its required side effect (e.g., `feeder.trigger()` skipped on a direct-load path). When editing `VolleyFireStorageManager`, `FireControlSystem`, or the auto plan runner, audit each transition for cleanup completeness and side-effect symmetry. Witnessed: `f1062cf`, `990855a`.
- **Hardware cache correctness.** Recent fixes around the per-loop cache flow have addressed: ordering between Lynx bulk cache clear and per-device cache invalidation, treating `null` as "uncached" (it's a valid cached value, not a sentinel for empty), HSV/distance recomputed per call so two same-loop reads disagreed, and caches not being invalidated when their inputs (filter config) changed. When editing `Hardware`, `HardwareCache`, or any `Smart*` sensor wrapper, verify all four. Witnessed: `1b568fe`, `1225776`.
- **Refactors that silently drop real-world calibration.** A hotfix was needed to re-add a potentiometer offset that a refactor had removed from the device-construction signature. The compiler does not catch this; only the robot does. When changing constructors, factory methods, or builders for hardware wrappers, audit every call site for parameters that carry calibration values. Witnessed: `a667b9a`.
- **Auto subclasses overriding `onInitialize` without `super`.** `AutoOpBase.onInitialize` runs substantive setup (alliance, drive config, limelight, telemetry, obelisk state) directly in the hook, so an auto subclass that overrides it without chaining `super.onInitialize()` silently strips that setup. Six subclasses had this bug; a fix added `super.onInitialize()` to all of them (`ff42101`). The lesson is **not** "remember to call super" — it's that this hook shouldn't require super at all (it's the lone deviation from the repo's hook convention; see *Key Non-Obvious Contracts*). Until `AutoOpBase` is refactored, audit auto subclasses for the missing super call, and don't add new super-requiring hooks.

A separate framework convention worth preserving even though no historical fix attests it directly:

- **Per-opmode reset of static subsystem fields.** Several opmodes hold subsystem references as `static` and explicitly reset them on init. The convention is the design — the absence of fix commits in this category suggests it's working. Removing the resets would reintroduce stale-state-across-opmode-reruns as a real failure mode.

## Editing Guidance

- Preserve Callam's control of architecture unless the requested task clearly requires structural change.
- Keep edits localized when working near teleop bindings, autonomous plans, or subsystem state transitions.
- If you change initialization order, cached hardware behavior, persisted keys, or supplier-based task wiring, explain the behavioral impact explicitly.
- Do not "clean up" implicit persistence or control-handoff behavior unless you can describe what user-visible behavior replaces it.

## Toolchain And Dependencies

- Gradle modules are `:FtcRobotController` (Android library, vendored upstream) and `:TeamCode` (Android application, depends on `:FtcRobotController`). Package root is `org.firstinspires.ftc.teamcode`; app id is `com.qualcomm.ftcrobotcontroller`.
- Android Gradle Plugin 8.7.0, `compileSdk` 35 (TeamCode) / 34 (FtcRobotController), `minSdk` 24, Java 8 source/target, NDK 21.3.6528147, ABIs `armeabi-v7a` + `arm64-v8a`.
- `targetSdk` is intentionally pinned to 28 with `//noinspection ExpiredTargetSdkVersion` in `build.common.gradle`. Do not "fix" it — it is a deliberate FTC SDK requirement.
- `versionCode` and `versionName` are scraped at configure time from `FtcRobotController/src/main/AndroidManifest.xml` by a regex in `build.common.gradle`. Bump version fields there, not in TeamCode.
- FTC SDK pinned to `org.firstinspires.ftc:* @ 11.1.0` across RobotCore, Hardware, FtcCommon, Vision, Inspection, Blocks, OnBotJava, RobotServer.
- Pedro Pathing pinned to `com.pedropathing:ftc:2.0.4` + `com.pedropathing:telemetry:1.0.0`. The Pedro 1.x → 2.x API shift matters when reading older code or external examples.
- `@Configurable` runtime tuning surface comes from Panels: `com.bylazar:fullpanels:1.0.12`.
- Non-default Maven sources: `https://maven.brott.dev/` (Road Runner) and `https://mymaven.bylazar.com/releases` (Pedro + Panels). Both are required for a clean build.
- TeamCode-only Java deps worth noticing while reading utilities/persistence code: `commons-math3:3.6.1`, `jackson-databind:2.13.4.2`.

## Validation Guidance

- Useful local commands:
  - `.\gradlew.bat :TeamCode:assembleDebug` — TeamCode debug APK only
  - `.\gradlew.bat assembleDebug` — both modules
  - `.\gradlew.bat lint`
  - `.\gradlew.bat clean`
- There is no JUnit or instrumentation test suite. `gradlew test` is effectively a no-op; do not invent test commands.
- A compile is necessary but not sufficient for robot-facing changes.
- For subsystem or opmode edits, call out what still needs driver-station or on-robot validation.

## Cross-Tool Notes

- A Codex-side skill at `.codex/skills/review-recent-commits/SKILL.md` encodes the same opmode-first reading order this file teaches. Useful as a cross-reference, but `AGENTS.md` remains the canonical agent guide.

## Keeping This File Current

- Update this file whenever any of the following change:
  - the primary match teleop opmode
  - the autonomous framework shape
  - subsystem ownership boundaries
  - persistent-storage keys or cross-opmode state behavior
  - teleop task bridges
  - major state machines
  - the expected source of truth for driver bindings
- When adding a new subsystem or coordinator, add:
  - where it is constructed
  - which opmodes actually use it
  - its main state enum or queue model, if any
  - any hidden dependencies on other systems
- When removing or replacing a state machine, delete or rewrite the corresponding section here in the same change.
- If a refactor changes which file is the real control surface, update the "Primary Entry Points" and "Active Control Surfaces" sections immediately.
- If you discover this file is stale during a task, fix `AGENTS.md` as part of that task rather than leaving a note for later.
