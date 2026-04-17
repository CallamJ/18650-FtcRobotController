package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.subsystems.FireControlSystem;
import org.firstinspires.ftc.teamcode.components.subsystems.StorageController;
import org.firstinspires.ftc.teamcode.components.mechanisms.*;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartCameraColorSensor;
import org.firstinspires.ftc.teamcode.hardware.SmartLEDIndicator;
import org.firstinspires.ftc.teamcode.hardware.SmartLimelight3A;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Configurable
public abstract class AutoOpBase extends OpModeCore {
    private static final Logger log = LoggerFactory.getLogger(AutoOpBase.class);
    protected static final SmartLimelight3A.AprilTag.Type DEFAULT_TAG_PATTERN = SmartLimelight3A.AprilTag.Type.OBELISK_GPP;

    protected static DriveBase driveBase;
    protected static Feeder feeder;
    protected static Collector collector;
    protected static Indexer indexer;
    protected static Launcher launcher;
    protected static StorageController storageController;
    protected static Hood hood;
    protected static Turret turret;
    protected static FireControlSystem fcs;
    protected static SmartLimelight3A limelight;
    protected static SmartCameraColorSensor frontCameraSensor;
    public static boolean runFCS = true;
    public static double targetVelocity = 2000;
    public static double matchStateSaveIntervalMs = 500;
    public static boolean enableObeliskAcquisitionAssist = true;
    public static double obeliskAcquisitionTimeoutSec = 3.0;
    public static double blueObeliskPoseX = 72;
    public static double blueObeliskPoseY = 0;
    public static double redObeliskPoseX = 72;
    public static double redObeliskPoseY = 0;
    public static double autoFollowerMaxPower = 1.0;
    public static double autoPathEndVelocityConstraint = 0.1;
    public static double autoPathEndTranslationalConstraint = 0.1;
    public static double autoPathEndHeadingConstraint = 0.007;
    public static double autoPathEndTValueConstraint = 0.995;
    public static double autoPathEndTimeoutConstraintMs = 100.0;
    public static double autoPathBrakingStrength = 1.0;
    public static double autoPathBrakingStart = 1.0;
    public static int autoPathBezierSearchLimit = 10;
    public static boolean autoPathUseGlobalDeceleration = false;
    public static boolean autoPathDisableDeceleration = false;

    protected SmartLimelight3A.AprilTag aprilTag = null;
    protected MatchStateStore.AllianceColor allianceColor = MatchStateStore.AllianceColor.BLUE;
    private long lastMatchStateSaveMs = 0;
    private boolean obeliskAssistComplete = false;
    private double obeliskAssistStartSec = 0;

    private String activeStepName = "None";
    private String activeStepStatus = "IDLE";
    private double activeStepElapsedSec = 0;
    private PlanResult lastPlanResult = null;

    public enum StepStatus {
        RUNNING,
        COMPLETE,
        FAILED,
        TIMED_OUT
    }

    public enum TimeoutAction {
        CONTINUE,
        JUMP_TO_STEP,
        CANCEL_PLAN
    }

    public interface AutoStep {
        String name();
        void start(AutoContext ctx);
        StepStatus tick(AutoContext ctx);
        default void stop(AutoContext ctx) {}
    }

    public static final class StepSpec {
        public final AutoStep step;
        public final double timeoutSec;
        public final boolean continueOnFailure;
        public final TimeoutAction timeoutAction;
        public final int timeoutJumpToStepIndex;

        public StepSpec(AutoStep step, double timeoutSec, boolean continueOnFailure) {
            this(step, timeoutSec, continueOnFailure, TimeoutAction.CONTINUE, -1);
        }

        public StepSpec(
                AutoStep step,
                double timeoutSec,
                boolean continueOnFailure,
                TimeoutAction timeoutAction,
                int timeoutJumpToStepIndex
        ) {
            this.step = step;
            this.timeoutSec = timeoutSec;
            this.continueOnFailure = continueOnFailure;
            this.timeoutAction = timeoutAction != null ? timeoutAction : TimeoutAction.CONTINUE;
            this.timeoutJumpToStepIndex = timeoutJumpToStepIndex;
        }

        public static StepSpec required(AutoStep step, double timeoutSec) {
            return new StepSpec(step, timeoutSec, false);
        }

        public static StepSpec optional(AutoStep step, double timeoutSec) {
            return new StepSpec(step, timeoutSec, true);
        }

        public StepSpec onTimeoutContinue() {
            return new StepSpec(step, timeoutSec, continueOnFailure, TimeoutAction.CONTINUE, -1);
        }

        public StepSpec onTimeoutCancelPlan() {
            return new StepSpec(step, timeoutSec, continueOnFailure, TimeoutAction.CANCEL_PLAN, -1);
        }

        public StepSpec onTimeoutJumpToStep(int stepIndex) {
            return new StepSpec(step, timeoutSec, continueOnFailure, TimeoutAction.JUMP_TO_STEP, stepIndex);
        }
    }

    public static final class PlanResult {
        public final boolean success;
        public final int completedSteps;
        public final String failureReason;

        public PlanResult(boolean success, int completedSteps, String failureReason) {
            this.success = success;
            this.completedSteps = completedSteps;
            this.failureReason = failureReason;
        }
    }

    protected final class AutoContext {
        public DriveBase driveBase() { return driveBase; }
        public Follower follower() { return driveBase != null ? driveBase.getFollower() : null; }
        public Collector collector() { return collector; }
        public Launcher launcher() { return launcher; }
        public StorageController storageController() { return storageController; }
        public SmartLimelight3A limelight() { return limelight; }
        public SmartLimelight3A.AprilTag latestObeliskTag() { return aprilTag; }
        public SmartLimelight3A.AprilTag.Type detectTagOrDefault(SmartLimelight3A.AprilTag.Type fallback) {
            return AutoOpBase.this.detectTagOrDefault(fallback);
        }
        public double runtimeSec() { return getRuntime(); }
    }

    private static void resetSubsystemReferences() {
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception e) {
                log.warn("Failed to stop previous limelight instance cleanly", e);
            }
        }

        driveBase = null;
        feeder = null;
        collector = null;
        indexer = null;
        launcher = null;
        storageController = null;
        if (frontCameraSensor != null) {
            try {
                frontCameraSensor.close();
            } catch (Exception e) {
                log.warn("Failed to close previous front camera color sensor cleanly", e);
            }
        }
        frontCameraSensor = null;
        hood = null;
        turret = null;
        fcs = null;
        limelight = null;
        runFCS = true;
    }

    @Override
    protected void onInitialize() {
        resetSubsystemReferences();
        allianceColor = getAutonomousAllianceColor();
        lastMatchStateSaveMs = 0;
        obeliskAssistComplete = !enableObeliskAcquisitionAssist;
        obeliskAssistStartSec = 0;

        DriveBaseMotorConfig.DriveBaseMotorConfigBuilder configBuilder = new DriveBaseMotorConfig.DriveBaseMotorConfigBuilder();
        configBuilder.leftFront("LFront", Direction.FORWARD);
        configBuilder.leftRear("LRear", Direction.FORWARD);
        configBuilder.rightFront("RFront", Direction.REVERSE);
        configBuilder.rightRear("RRear", Direction.FORWARD);

        telemetry.setMsTransmissionInterval(11);
        Constants.setMecanumMaxPower(autoFollowerMaxPower);

        try {
            limelight = hardware.getLimelight("limelight");
            limelight.setPipeline(0);
            limelight.start();

            prettyTelem.addLine("Limelight")
                    .addData("FPS", limelight::getFps)
                    .addData("April Tag", () -> {
                        this.aprilTag = limelight.getFirstObelisk() == null ? this.aprilTag : limelight.getFirstObelisk();
                        return this.aprilTag == null ? "None" : this.aprilTag.toString();
                    });
        } catch (Exception e) {
            limelight = null;
            prettyTelem.error("Limelight failed to initialize, skipping: " + e.getMessage());
        }

        try {
            driveBase = new DriveBase(hardwareMap, configBuilder.build(), true);
        } catch (Exception e) {
            prettyTelem.error("Drive base failed to initialize, skipping: " + e.getMessage());
        }

        try {
            feeder = new Feeder(
                    hardwareMap.get(CRServo.class, "feederServo"),
                    hardware.getPotentiometer("feederPotentiometer", 270, 3.3)
            );
            indexer = new Indexer(hardware.getMotor("indexerMotor", true));
            collector = new Collector(hardware.getMotor("collectorMotor"));
            frontCameraSensor = hardware
                    .getCamera("colorCamera", new Pose(0, 0, 0))
                    .asColorSensor();
            storageController = new StorageController(
                    feeder,
                    indexer,
                    collector,
                    frontCameraSensor,
                    hardware.getLEDIndicator("leftLED"),
                    hardware.getLEDIndicator("rightLED"),
                    hardware.getLEDIndicator("frontLED")
            );
        } catch (Exception e) {
            prettyTelem.error("Storage system failed to initialize, skipping: " + e.getMessage());
        }

        try {
            launcher = new Launcher(hardware, hardware.getMotor("launcherMotor"));
        } catch (Exception e) {
            prettyTelem.error("Launcher failed to initialize, skipping: " + e.getMessage());
        }

        try {
            hood = new Hood(hardware.getServo("hoodServo"));
        } catch (Exception e) {
            prettyTelem.error("Hood failed to initialize, skipping: " + e.getMessage());
        }

        try {
            turret = new Turret(hardware.getMotor("turretMotor"), hardware.getMotor("turretMotor").getEncoder());
        } catch (Exception e) {
            prettyTelem.error("Turret failed to initialize, skipping: " + e.getMessage());
        }

        try {
            if (turret != null && hood != null && launcher != null && limelight != null) {
                SmartLEDIndicator launcherLED = hardware.getLEDIndicator("launcherLED");
                fcs = new FireControlSystem(
                        turret,
                        hood,
                        launcher,
                        limelight,
                        launcherLED,
                        driveBase,
                        null
                );
                fcs.setFallbackVelocity(targetVelocity);
                fcs.setAllianceColor(allianceColor);
            }
        } catch (Exception e) {
            prettyTelem.error("Fire Control System failed to initialize, skipping: " + e.getMessage());
        }

        prettyTelem.addLine("Auto Plan")
                .addData("Step", () -> activeStepName)
                .addData("Status", () -> activeStepStatus)
                .addData("Step Elapsed (s)", () -> activeStepElapsedSec)
                .addData("Drive Max Power", () -> autoFollowerMaxPower)
                .addData("Plan Result", () -> {
                    if (lastPlanResult == null) return "Not started";
                    if (lastPlanResult.success) return "Success";
                    return "Failed: " + lastPlanResult.failureReason;
                });

        if (driveBase != null) {
            prettyTelem.addLine("Localization")
                    .addData("X", () -> driveBase.getPoseSimple().x())
                    .addData("Y", () -> driveBase.getPoseSimple().y())
                    .addData("Heading", () -> driveBase.getPoseSimple().heading());
        } else {
            prettyTelem.warning("Localization telemetry disabled because drive base failed to initialize.");
        }
    }

    @Override
    protected final void onRun() {
        List<StepSpec> plan = buildPlan();
        if (plan == null || plan.isEmpty()) {
            activeStepName = "None";
            activeStepStatus = "NO_PLAN";
            lastPlanResult = new PlanResult(true, 0, null);
            onPlanFinished(lastPlanResult);
            return;
        }

        AutoContext context = new AutoContext();
        int completed = 0;

        int stepIndex = 0;
        while (stepIndex < plan.size()) {
            StepSpec spec = plan.get(stepIndex);
            if (!opModeIsActive()) {
                lastPlanResult = new PlanResult(false, completed, "OpMode stopped");
                onPlanFinished(lastPlanResult);
                return;
            }

            if (spec == null || spec.step == null) {
                String message = "Encountered null step spec";
                prettyTelem.error(message);
                lastPlanResult = new PlanResult(false, completed, message);
                onPlanFinished(lastPlanResult);
                return;
            }

            AutoStep step = spec.step;
            activeStepName = step.name();
            activeStepStatus = "STARTING";
            ElapsedTime stepTimer = new ElapsedTime();

            try {
                step.start(context);
            } catch (Throwable error) {
                String reason = "Step start failed: " + step.name() + " - " + error.getMessage();
                prettyTelem.error(reason);
                log.error(reason, error);
                onPlanError(step, error);
                if (!spec.continueOnFailure) {
                    lastPlanResult = new PlanResult(false, completed, reason);
                    onPlanFinished(lastPlanResult);
                    return;
                }
                stepIndex++;
                continue;
            }

            StepStatus status = StepStatus.RUNNING;
            Throwable stepError = null;
            while (opModeIsActive()) {
                tick();
                activeStepElapsedSec = stepTimer.seconds();

                if (spec.timeoutSec > 0 && stepTimer.seconds() > spec.timeoutSec) {
                    status = StepStatus.TIMED_OUT;
                    break;
                }

                try {
                    status = step.tick(context);
                } catch (Throwable error) {
                    status = StepStatus.FAILED;
                    stepError = error;
                    break;
                }

                if (status != StepStatus.RUNNING) {
                    break;
                }
            }

            try {
                step.stop(context);
            } catch (Throwable error) {
                prettyTelem.warning("Step stop threw: " + step.name() + ": " + error.getMessage());
                log.warn("Step stop threw: {}", step.name(), error);
            }

            activeStepStatus = status.name();
            if (status == StepStatus.COMPLETE) {
                completed++;
                stepIndex++;
                continue;
            }

            String reason;
            if (status == StepStatus.TIMED_OUT) {
                reason = "Timed out in step: " + step.name();
            } else if (status == StepStatus.FAILED) {
                reason = "Failed step: " + step.name() + (stepError != null ? " - " + stepError.getMessage() : "");
            } else {
                reason = "Interrupted in step: " + step.name();
            }

            prettyTelem.error(reason);
            if (stepError != null) {
                log.error(reason, stepError);
                onPlanError(step, stepError);
            }

            if (status == StepStatus.TIMED_OUT) {
                if (spec.timeoutAction == TimeoutAction.CANCEL_PLAN) {
                    lastPlanResult = new PlanResult(false, completed, reason);
                    onPlanFinished(lastPlanResult);
                    return;
                }

                if (spec.timeoutAction == TimeoutAction.JUMP_TO_STEP) {
                    int target = spec.timeoutJumpToStepIndex;
                    if (target < 0 || target >= plan.size()) {
                        String invalidTargetReason = reason + " - invalid timeout jump target step index: " + target;
                        prettyTelem.error(invalidTargetReason);
                        lastPlanResult = new PlanResult(false, completed, invalidTargetReason);
                        onPlanFinished(lastPlanResult);
                        return;
                    }
                    if (target == stepIndex) {
                        String invalidTargetReason = reason + " - timeout jump target must be a different step index: " + target;
                        prettyTelem.error(invalidTargetReason);
                        lastPlanResult = new PlanResult(false, completed, invalidTargetReason);
                        onPlanFinished(lastPlanResult);
                        return;
                    }
                    stepIndex = target;
                    continue;
                }

                stepIndex++;
                continue;
            }

            if (spec.continueOnFailure) {
                stepIndex++;
                continue;
            }

            lastPlanResult = new PlanResult(false, completed, reason);
            onPlanFinished(lastPlanResult);
            return;
        }

        lastPlanResult = new PlanResult(true, completed, null);
        activeStepName = "Complete";
        activeStepStatus = "COMPLETE";
        onPlanFinished(lastPlanResult);
    }

    @Override
    protected void onTick() {
        runObeliskAcquisitionAssist();

        if (storageController != null) {
            storageController.tick();
        }

        if (fcs != null && runFCS) {
            try {
                fcs.tick();
            } catch (Exception e) {
                runFCS = false;
                String message = e.getMessage();
                prettyTelem.error("FCS tick failed; disabling FCS. " +
                        e.getClass().getSimpleName() +
                        (message != null ? ": " + message : ""));
                log.error("FCS tick failed; disabling FCS", e);
            }
        } else {
            if (launcher != null) {
                launcher.tick();
            }
            if (turret != null) {
                turret.tick();
            }
        }

        if (driveBase != null && driveBase.getFollower() != null) {
            driveBase.getFollower().update();
        }

        persistMatchStateIfDue(false);
    }

    protected List<StepSpec> buildPlan() {
        return List.of();
    }

    protected MatchStateStore.AllianceColor getAutonomousAllianceColor() {
        return MatchStateStore.AllianceColor.BLUE;
    }

    protected final void persistMatchStateIfDue(boolean force) {
        long now = System.currentTimeMillis();
        long intervalMs = Math.max(100L, (long) matchStateSaveIntervalMs);
        if (!force && now - lastMatchStateSaveMs < intervalMs) {
            return;
        }
        SmartLimelight3A.AprilTag obeliskTag = aprilTag != null ? aprilTag : (limelight != null ? limelight.getFirstObelisk() : null);
        MatchStateStore.saveSnapshot(driveBase, storageController, indexer, turret, allianceColor, obeliskTag);
        lastMatchStateSaveMs = now;
    }

    private void runObeliskAcquisitionAssist() {
        if (!enableObeliskAcquisitionAssist || obeliskAssistComplete) {
            if (fcs != null) {
                fcs.setDepotAutoAimEnabled(true);
            }
            return;
        }

        if (limelight == null || turret == null || driveBase == null) {
            obeliskAssistComplete = true;
            if (fcs != null) {
                fcs.setDepotAutoAimEnabled(true);
            }
            return;
        }

        if (obeliskAssistStartSec == 0) {
            obeliskAssistStartSec = getRuntime();
        }

        SmartLimelight3A.AprilTag obeliskTag = limelight.getFirstObelisk();
        double elapsedSec = Math.max(0, getRuntime() - obeliskAssistStartSec);
        boolean timedOut = elapsedSec >= Math.max(0.1, obeliskAcquisitionTimeoutSec);

        if (obeliskTag != null) {
            aprilTag = obeliskTag;
            obeliskAssistComplete = true;
            if (fcs != null) {
                fcs.setDepotAutoAimEnabled(true);
            }
            return;
        }

        if (timedOut) {
            obeliskAssistComplete = true;
            if (fcs != null) {
                fcs.setDepotAutoAimEnabled(true);
            }
            return;
        }

        if (fcs != null) {
            fcs.setDepotAutoAimEnabled(false);
        }

        Pose currentPose = driveBase.getPoseSimple();
        Pose obeliskPose = getAllianceObeliskPose();
        double deltaX = obeliskPose.x() - currentPose.x();
        double deltaY = obeliskPose.y() - currentPose.y();
        if (Math.hypot(deltaX, deltaY) < 1e-6) {
            return;
        }

        double headingToObeliskDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double relativeBearingDeg = normalizeDegrees(currentPose.heading() - headingToObeliskDeg);
        turret.setTargetPosition(relativeBearingDeg);
    }

    private Pose getAllianceObeliskPose() {
        if (allianceColor == MatchStateStore.AllianceColor.RED) {
            return new Pose(redObeliskPoseX, redObeliskPoseY);
        }
        return new Pose(blueObeliskPoseX, blueObeliskPoseY);
    }

    private static double normalizeDegrees(double angleDeg) {
        double normalized = angleDeg % 360.0;
        if (normalized > 180.0) {
            normalized -= 360.0;
        } else if (normalized < -180.0) {
            normalized += 360.0;
        }
        return normalized;
    }

    protected void onPlanFinished(PlanResult result) {}

    protected void onPlanError(AutoStep step, Throwable error) {}

    protected SmartLimelight3A.AprilTag.Type detectTagOrDefault(SmartLimelight3A.AprilTag.Type fallback) {
        SmartLimelight3A.AprilTag tag = this.aprilTag;
        if (tag == null && limelight != null) {
            tag = limelight.getFirstObelisk();
        }
        if (tag == null || tag.type() == null) {
            return fallback;
        }
        return tag.type();
    }

    protected boolean isFcsReady() {
        if (fcs == null || !runFCS) {
            return true;
        }
        return fcs.isTurretAligned() && fcs.isLauncherSpun();
    }

    protected void queueLoadForPattern(SmartLimelight3A.AprilTag.Type type) {
        if (storageController == null) {
            throw new IllegalStateException("Storage controller unavailable");
        }

        SmartLimelight3A.AprilTag.Type selected = type != null ? type : DEFAULT_TAG_PATTERN;
        switch (selected) {
            case OBELISK_GPP:
                storageController.loadGreen();
                storageController.loadPurple();
                storageController.loadPurple();
                break;
            case OBELISK_PGP:
                storageController.loadPurple();
                storageController.loadGreen();
                storageController.loadPurple();
                break;
            case OBELISK_PPG:
                storageController.loadPurple();
                storageController.loadPurple();
                storageController.loadGreen();
                break;
            default:
                storageController.loadGreen();
                storageController.loadPurple();
                storageController.loadPurple();
        }
    }

    protected StepSpec queuePatternWhenFcsReadyStep(
            String name,
            Supplier<SmartLimelight3A.AprilTag.Type> patternSupplier,
            double timeoutSec
    ) {
        return StepSpec.required(new AutoStep() {
            private List<StorageController.SlotContent> pendingLoads;
            private int queuedCount;
            private boolean waitingForStorageIdle;

            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                SmartLimelight3A.AprilTag.Type pattern = patternSupplier != null
                        ? patternSupplier.get()
                        : DEFAULT_TAG_PATTERN;
                pendingLoads = getPatternLoadOrder(pattern);
                queuedCount = 0;
                waitingForStorageIdle = false;
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                if (storageController == null) {
                    return StepStatus.FAILED;
                }

                if (queuedCount >= pendingLoads.size()) {
                    return waitingForStorageIdle && !storageController.allTasksComplete()
                            ? StepStatus.RUNNING
                            : StepStatus.COMPLETE;
                }

                if (waitingForStorageIdle) {
                    if (storageController.allTasksComplete()) {
                        waitingForStorageIdle = false;
                    }
                    return StepStatus.RUNNING;
                }

                if (!storageController.allTasksComplete()) {
                    return StepStatus.RUNNING;
                }

                if (!isFcsReady()) {
                    return StepStatus.RUNNING;
                }

                StorageController.SlotContent next = pendingLoads.get(queuedCount);
                if (next == StorageController.SlotContent.GREEN) {
                    storageController.loadGreen();
                } else if (next == StorageController.SlotContent.PURPLE) {
                    storageController.loadPurple();
                } else {
                    return StepStatus.FAILED;
                }

                queuedCount++;
                waitingForStorageIdle = true;
                return StepStatus.RUNNING;
            }
        }, timeoutSec);
    }

    private List<StorageController.SlotContent> getPatternLoadOrder(SmartLimelight3A.AprilTag.Type type) {
        SmartLimelight3A.AprilTag.Type selected = type != null ? type : DEFAULT_TAG_PATTERN;
        switch (selected) {
            case OBELISK_GPP:
                return List.of(
                        StorageController.SlotContent.GREEN,
                        StorageController.SlotContent.PURPLE,
                        StorageController.SlotContent.PURPLE
                );
            case OBELISK_PGP:
                return List.of(
                        StorageController.SlotContent.PURPLE,
                        StorageController.SlotContent.GREEN,
                        StorageController.SlotContent.PURPLE
                );
            case OBELISK_PPG:
                return List.of(
                        StorageController.SlotContent.PURPLE,
                        StorageController.SlotContent.PURPLE,
                        StorageController.SlotContent.GREEN
                );
            default:
                return List.of(
                        StorageController.SlotContent.GREEN,
                        StorageController.SlotContent.PURPLE,
                        StorageController.SlotContent.PURPLE
                );
        }
    }

    protected StepSpec instantStep(String name, Runnable action) {
        return StepSpec.required(new AutoStep() {
            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                action.run();
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                return StepStatus.COMPLETE;
            }
        }, 0);
    }

    protected StepSpec waitUntilStep(String name, BooleanSupplier condition, double timeoutSec) {
        return StepSpec.required(new AutoStep() {
            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {}

            @Override
            public StepStatus tick(AutoContext ctx) {
                return condition.getAsBoolean() ? StepStatus.COMPLETE : StepStatus.RUNNING;
            }
        }, timeoutSec);
    }

    protected StepSpec waitSecondsStep(String name, double durationSec) {
        return StepSpec.required(new AutoStep() {
            private ElapsedTime timer;

            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                timer = new ElapsedTime();
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                if (timer == null) {
                    return StepStatus.FAILED;
                }
                return timer.seconds() >= durationSec ? StepStatus.COMPLETE : StepStatus.RUNNING;
            }
        }, durationSec + 0.5);
    }

    protected StepSpec followPathStep(String name, Supplier<PathChain> pathSupplier, double timeoutSec) {
        return StepSpec.required(new AutoStep() {
            private Follower follower;

            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                follower = ctx.follower();
                if (follower == null) {
                    throw new IllegalStateException("Follower unavailable");
                }
                PathChain path = pathSupplier.get();
                if (path == null) {
                    throw new IllegalStateException("Path supplier returned null");
                }
                follower.followPath(path);
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                if (follower == null) {
                    return StepStatus.FAILED;
                }
                return follower.isBusy() ? StepStatus.RUNNING : StepStatus.COMPLETE;
            }
        }, timeoutSec);
    }

    protected PathBuilder autoPathBuilder(Follower follower) {
        if (follower == null) {
            throw new IllegalArgumentException("Follower unavailable");
        }

        PathConstraints constraints = new PathConstraints(
                autoPathEndTValueConstraint,
                autoPathEndVelocityConstraint,
                autoPathEndTranslationalConstraint,
                autoPathEndHeadingConstraint,
                autoPathEndTimeoutConstraintMs,
                autoPathBrakingStrength,
                Math.max(1, autoPathBezierSearchLimit),
                autoPathBrakingStart
        );

        PathBuilder builder = new PathBuilder(follower, constraints);
        if (autoPathDisableDeceleration) {
            builder.setNoDeceleration();
        } else if (autoPathUseGlobalDeceleration) {
            builder.setGlobalDeceleration(autoPathBrakingStart);
        }
        return builder;
    }

    protected StepSpec turnDegreesStep(String name, double degrees, boolean isLeft, double timeoutSec) {
        return StepSpec.required(new AutoStep() {
            private Follower follower;

            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                follower = ctx.follower();
                if (follower == null) {
                    throw new IllegalStateException("Follower unavailable");
                }
                follower.turnDegrees(degrees, isLeft);
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                if (follower == null) {
                    return StepStatus.FAILED;
                }
                return follower.isBusy() ? StepStatus.RUNNING : StepStatus.COMPLETE;
            }
        }, timeoutSec);
    }

    protected List<StepSpec> plan(StepSpec... steps) {
        List<StepSpec> specs = new ArrayList<>();
        if (steps == null) {
            return specs;
        }
        for (StepSpec step : steps) {
            if (step != null) {
                specs.add(step);
            }
        }
        return specs;
    }

    @Deprecated
    public PathChain buildPath(PathBuilder pathBuilder, Follower follower) {
        return pathBuilder.build();
    }
}

