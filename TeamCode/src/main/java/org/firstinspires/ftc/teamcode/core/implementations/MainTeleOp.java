package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.subsystems.FeedSystem;
import org.firstinspires.ftc.teamcode.components.subsystems.FireControlSystem;
import org.firstinspires.ftc.teamcode.components.subsystems.IndexerStorage;
import org.firstinspires.ftc.teamcode.components.subsystems.VolleyFireStorageManager;
import org.firstinspires.ftc.teamcode.components.mechanisms.*;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.core.teleoptasks.CancelReason;
import org.firstinspires.ftc.teamcode.core.teleoptasks.TeleOpTaskContext;
import org.firstinspires.ftc.teamcode.core.teleoptasks.TeleOpTaskManager;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.SmartCameraColorSensor;
import org.firstinspires.ftc.teamcode.hardware.SmartColorSensor;
import org.firstinspires.ftc.teamcode.hardware.SmartLEDIndicator;
import org.firstinspires.ftc.teamcode.hardware.SmartLimelight3A;
import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

@Configurable
@TeleOp(name = "1 - Main TeleOp")
public class MainTeleOp extends TeleOpCore {
    protected static DriveBase driveBase;
    protected static FeedWheels feedWheels;
    protected static FeedRamp feedRamp;
    protected static FeedSystem feeder;
    protected static Turret turret;
    protected static Collector collector;
    protected static Indexer indexer;
    protected static Launcher launcher;
    protected static IndexerStorage indexerStorage;
    protected static VolleyFireStorageManager volleyStorageManager;
    protected static SmartCameraColorSensor frontCameraSensor;
    protected static SmartColorSensor frontColorSensor;
    protected static Hood hood;
    protected static FireControlSystem fcs;
    protected static SmartLimelight3A limelight;
    protected static Limelight3A limelight3A;
    protected ElapsedTime tickTimer = new ElapsedTime();
    protected DataFilter tickTimeFilter = new RollingAverage(10);


    public static double launchVelocity = 2150;
    public static boolean runFCS = true;
    public static double matchStateFreshnessMs = 10000;
    public static double matchStateSaveIntervalMs = 500;
    public static double teleOpFollowerMaxPower = 1.0;
    public static MatchStateStore.AllianceColor defaultAllianceColor = MatchStateStore.AllianceColor.BLUE;
    public static double indexerZeroBumpTicksPerTriggerUnit = 20.0;
    public static double turretZeroBumpTicksPerTriggerUnit = -200.0;
    public static SmartLEDIndicator.IndicatorColor turretZeroTrimLedColor = SmartLEDIndicator.IndicatorColor.INDIGO;
    public static double maintenancePoseTrimInchesPerTouchpadUnit = 12.0;
    private MatchStateStore.AllianceColor allianceColor = defaultAllianceColor;
    private MatchStateStore.Snapshot startupSnapshot;
    private boolean loadedFreshSnapshot = false;
    private long lastMatchStateSaveMs = 0;
    private TeleOpTaskManager teleOpTaskManager;
    public static boolean clearLeftSlotWhenFeederReturns = false;

    private static void resetSubsystemReferences() {
        if (limelight != null) {
            limelight.stop();
        }

        driveBase = null;
        feedWheels = null;
        feedRamp = null;
        feeder = null;
        turret = null;
        collector = null;
        indexer = null;
        launcher = null;
        indexerStorage = null;
        volleyStorageManager = null;
        if (frontCameraSensor != null) {
            frontCameraSensor.close();
        }
        frontCameraSensor = null;
        frontColorSensor = null;
        hood = null;
        fcs = null;
        limelight = null;
        limelight3A = null;
        limelightLocalizer = null;
    }

    @Override
    protected void onInitialize(){
        //noinspection DuplicatedCode
        super.onInitialize();
        resetSubsystemReferences();
        startupSnapshot = MatchStateStore.getFreshSnapshot(Math.max(1000L, (long) matchStateFreshnessMs));
        allianceColor = startupSnapshot != null
                ? MatchStateStore.parseAllianceColor(startupSnapshot.allianceColor, defaultAllianceColor)
                : defaultAllianceColor;
        loadedFreshSnapshot = startupSnapshot != null;
        lastMatchStateSaveMs = 0;
        teleOpTaskManager = null;
        clearLeftSlotWhenFeederReturns = false;
        runFCS = true;

        DriveBaseMotorConfig.DriveBaseMotorConfigBuilder configBuilder = new DriveBaseMotorConfig.DriveBaseMotorConfigBuilder();
        configBuilder.leftFront("LFront", Direction.FORWARD);
        configBuilder.leftRear("LRear", Direction.FORWARD);
        configBuilder.rightFront("RFront", Direction.REVERSE);
        configBuilder.rightRear("RRear", Direction.FORWARD);

        Constants.setMecanumMaxPower(teleOpFollowerMaxPower);

        try {
            feedWheels = new FeedWheels(
                    hardwareMap.get(CRServo.class, "leftFeedServo"),
                    hardwareMap.get(CRServo.class, "rightFeedServo")
            );
            feedRamp = new FeedRamp(
                    hardware.getServo("leftFeedRampServo"),
                    hardware.getServo("rightFeedRampServo")
            );

            feeder = new FeedSystem(feedWheels, feedRamp);
            feeder.stopFeeding();
        } catch (Exception e) {
            prettyTelem.error("Feeder failed to initialize, skipping: " + e.getMessage());
        }

        try {
            driveBase = new DriveBase(hardwareMap, configBuilder.build(), true);
            applyPersistedPoseIfAvailable();
            if (driveBase.getFollower() != null) {
                driveBase.getFollower().startTeleOpDrive(true);
                driveBase.getFollower().setTeleOpDrive(0, 0, 0, true);
            }
        } catch (Exception e) {
            prettyTelem.error("Drive base failed to initialize, skipping: " + e.getMessage());
        }

        try {
            launcher = new Launcher(hardware, hardware.getMotor("launcherMotor"));
            hood = new Hood(hardware.getServo("hoodServo"));
            turret = new Turret(hardware.getMotor("turretMotor"), hardware.getMotor("turretMotor").getEncoder());
            limelight = hardware.getLimelight("limelight");
            limelight.setPipeline(0);
            limelight.start();

            prettyTelem.addLine("lime light")
                    .addData("limelight results", limelight::getAprilTags);

            fcs = new FireControlSystem(
                    turret,
                    hood,
                    launcher,
                    limelight,
                    hardware.getLEDIndicator("launcherLED"),
                    driveBase,
                    null
            );
            fcs.setFallbackVelocity(launchVelocity);
            fcs.setAllianceColor(allianceColor);
        } catch (Exception e) {
            prettyTelem.error("Fire Control System failed to initialize, skipping: " + e.getMessage());
        }

        try {
            indexer = new Indexer(hardware.getMotor("indexerMotor", true));
            collector = new Collector(hardware.getMotor("collectorMotor"));
            frontColorSensor = hardware.getColorSensor("frontColorSensor");
            if (feeder == null) {
                throw new IllegalStateException("Feeder unavailable");
            }
            if (fcs == null) {
                throw new IllegalStateException("Fire control system unavailable");
            }
            if (frontColorSensor == null) {
                throw new IllegalStateException("Color sensor failed to initialize");
            }
            if (!frontColorSensor.hasDistanceSensing()) {
                throw new IllegalStateException("Front color sensor must support distance sensing");
            }
            indexerStorage = new IndexerStorage(
                    indexer,
                    frontColorSensor,
                    hardware.getLEDIndicator("leftLED"),
                    hardware.getLEDIndicator("rightLED"),
                    hardware.getLEDIndicator("frontLED")
            );
            volleyStorageManager = new VolleyFireStorageManager(
                    feeder,
                    indexer,
                    collector,
                    indexerStorage,
                    fcs
            );
            applyPersistedStorageStateIfAvailable();
        } catch (Exception e) {
            prettyTelem.error("Storage failed to initialize, skipping: " + e.getMessage());
        }

        teleOpTaskManager = new TeleOpTaskManager(
                new TeleOpTaskContext(
                        () -> driveBase,
                        () -> indexerStorage,
                        () -> volleyStorageManager,
                        () -> fcs,
                        () -> allianceColor,
                        this::getRuntime,
                        message -> prettyTelem.warning("TeleOp Task: " + message)
                )
        );
        rebuildTelemetryLayout();
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        double driveX = -gamepad1.leftStickX;
        double driveY = -gamepad1.leftStickY;
        double driveTurn = -gamepad1.rightStickX;
        boolean hasDriverOverrideInput = Math.abs(driveX) > TeleOpTaskManager.driverOverrideDeadband
                || Math.abs(driveY) > TeleOpTaskManager.driverOverrideDeadband
                || Math.abs(driveTurn) > TeleOpTaskManager.driverOverrideDeadband;

        boolean turretTrimActive = false;

        if (gamepad2.startPressed()) {
            resetPoseToCloseAutoStart();
        }
        if (gamepad2.bPressed()) {
            toggleAllianceColor();
        }
        turretTrimActive |= applyZeroTrim(gamepad2);

        boolean maintenanceMode = gamepad1.share;
        if (maintenanceMode) {
            if (gamepad1.startPressed()) {
                resetPoseToCloseAutoStart();
            }

            if (gamepad1.bPressed()) {
                toggleAllianceColor();
            }
            if (gamepad1.leftStickButtonPressed()) {
                resetTurretZeroToCurrent();
            }
            turretTrimActive |= applyZeroTrim(gamepad1);
            applyMaintenanceTouchpadPoseTrim(gamepad1);
            applyTurretTrimLedOverride(turretTrimActive);

            return;
        }
        applyTurretTrimLedOverride(turretTrimActive);

        if (teleOpTaskManager != null) {
            if (teleOpTaskManager.hasActiveTask()) {
                if (gamepad1.guidePressed()) {
                    teleOpTaskManager.onGuidePressed();
                } else if (hasDriverOverrideInput) {
                    teleOpTaskManager.cancelActive(CancelReason.DRIVER_STICK_OVERRIDE);
                }
            } else if (gamepad1.guidePressed()) {
                teleOpTaskManager.startFarFiringTask();
            }
        }

        if(driveBase != null && (teleOpTaskManager == null || !teleOpTaskManager.hasActiveTask())){
            driveBase.moveUsingPower(driveX, driveY, driveTurn);
        }

        if (feeder != null) {
            if (gamepad1.yPressed()) {
                feeder.toggleFeeding();
            }
        }

        if(volleyStorageManager != null){
            if(gamepad1.leftBumperPressed()){
                if (feeder != null) {
                    feeder.stopFeeding();
                }
                indexer.advanceIndexCounterclockwise();
                volleyStorageManager.dropFreshFlag();
            }
            if(gamepad1.rightBumperPressed()){
                if (feeder != null) {
                    feeder.stopFeeding();
                }
                indexer.advanceIndexClockwise();
                volleyStorageManager.dropFreshFlag();
            }
            if(gamepad1.dpadLeftPressed()){
                if(fcs == null || fcs.isLauncherRunning()) {
                    volleyStorageManager.firePPG();
                }
            }
            if(gamepad1.dpadRightPressed()){
                if(fcs == null || fcs.isLauncherRunning()) {
                    volleyStorageManager.firePGP();
                }
            }
            if(gamepad1.dpadUpPressed()){
                if(fcs == null || fcs.isLauncherRunning()){
                    volleyStorageManager.fireGPP();
                }
            }
            if(gamepad1.dpadDownPressed()){
                if(fcs == null || fcs.isLauncherRunning()){
                    volleyStorageManager.fireAny();
                }
            }
            if(gamepad1.startPressed()){
                indexerStorage.setLeftContent(IndexerStorage.SlotContent.OPEN);
                indexerStorage.setRightContent(IndexerStorage.SlotContent.OPEN);
                indexerStorage.setFrontContent(IndexerStorage.SlotContent.OPEN);
            }
        }
        if(fcs != null){
            if(gamepad1.xPressed()){
                fcs.toggleLauncher();
            }
            if(gamepad2.dpadDownPressed()){
                launchVelocity -= 50;
                fcs.setFallbackVelocity(launchVelocity);
            }

            if(gamepad2.dpadUpPressed()){
                launchVelocity += 50;
                fcs.setFallbackVelocity(launchVelocity);
            }
        }

        //noinspection DuplicatedCode
        if(collector != null){
            if(gamepad1.aPressed()){
                double forwardPower = 1;
                if(collector.getPower() == forwardPower){
                    collector.stop();
                } else {
                    collector.setPower(forwardPower);
                }
            }

            if(gamepad1.bPressed()){
                double reversePower = -0.5;
                if(collector.getPower() == reversePower){
                    collector.stop();
                } else {
                    collector.setPower(reversePower);
                }
            }
        }
    }

    private void toggleAllianceColor() {
        allianceColor = allianceColor.opposite();
        if (fcs != null) {
            fcs.setAllianceColor(allianceColor);
        }
        persistMatchStateIfDue(true);
    }

    @Override
    protected void beforeTick(){
        tickTimer.reset();
    }

    @Override
    protected void onTick(){
        if(volleyStorageManager != null){
            volleyStorageManager.tick();
        }

        if(fcs != null && runFCS){
            try {
                fcs.tick();
            } catch (Exception e) {
                runFCS = false;
                String message = e.getMessage();
                prettyTelem.error("FCS tick failed; disabling FCS. " +
                        e.getClass().getSimpleName() +
                        (message != null ? ": " + message : ""));
            }
        }

        if(driveBase != null && driveBase.getFollower() != null){
            if (teleOpTaskManager != null) {
                teleOpTaskManager.update();
                if (teleOpTaskManager.hasActiveTask()) {
                    driveBase.getFollower().update();
                } else {
                    driveBase.getFollower().updatePose();
                }
            } else {
                driveBase.getFollower().updatePose();
            }
        }

        persistMatchStateIfDue(false);
    }

    private void applyPersistedPoseIfAvailable() {
        if (startupSnapshot == null || driveBase == null || driveBase.getFollower() == null) {
            return;
        }

        driveBase.getFollower().setStartingPose(
                new com.pedropathing.geometry.Pose(
                        startupSnapshot.poseXInches,
                        startupSnapshot.poseYInches,
                        Math.toRadians(startupSnapshot.poseHeadingDegrees)
                )
        );
    }

    private void applyPersistedStorageStateIfAvailable() {
        if (startupSnapshot == null || indexerStorage == null || indexer == null) {
            return;
        }

        indexerStorage.setFrontContent(
                MatchStateStore.parseSlotContent(startupSnapshot.frontContent, IndexerStorage.SlotContent.OPEN)
        );
        indexerStorage.setRightContent(
                MatchStateStore.parseSlotContent(startupSnapshot.rightContent, IndexerStorage.SlotContent.OPEN)
        );
        indexerStorage.setLeftContent(
                MatchStateStore.parseSlotContent(startupSnapshot.leftContent, IndexerStorage.SlotContent.OPEN)
        );
        indexer.setTargetIndex(startupSnapshot.indexerTargetIndex);
    }

    private void persistMatchStateIfDue(boolean force) {
        long now = System.currentTimeMillis();
        long intervalMs = Math.max(100L, (long) matchStateSaveIntervalMs);
        if (!force && now - lastMatchStateSaveMs < intervalMs) {
            return;
        }
        SmartLimelight3A.AprilTag obeliskTag = limelight != null ? limelight.getFirstObelisk() : null;
        MatchStateStore.saveSnapshot(driveBase, indexerStorage, indexer, turret, allianceColor, obeliskTag);
        lastMatchStateSaveMs = now;
    }

    private void rebuildTelemetryLayout() {
        prettyTelem.resetLayout();
        registerDebugTelemetry();
    }

    private void registerDebugTelemetry() {
        prettyTelem.addData("Tick Time", () -> tickTimeFilter.compute(tickTimer.milliseconds()));

        prettyTelem.addLine("Match")
                .addData("Alliance", () -> allianceColor.name())
                .addData("Loaded Fresh Snapshot", () -> loadedFreshSnapshot);

        prettyTelem.addLine("Localization")
                .addData("X", () -> driveBase == null ? "n/a" : driveBase.getPoseSimple().x())
                .addData("Y", () -> driveBase == null ? "n/a" : driveBase.getPoseSimple().y())
                .addData("Heading", () -> driveBase == null ? "n/a" : driveBase.getPoseSimple().heading());
        prettyTelem.addLine("Hood")
                .addData("Target Pos", () -> hood == null ? "n/a" : hood.getTargetPosition());
        prettyTelem.addLine("Indexer")
                .addData("Current Position", () -> indexer == null ? "n/a" : indexer.getCurrentPosition())
                .addData("Target Position", () -> indexer == null ? "n/a" : indexer.getTargetPosition())
                .addData("Is Busy", () -> indexer != null && indexer.isBusy())
                .addData("Current Index", () -> indexer == null ? "n/a" : indexer.getCurrentIndex())
                .addData("Target Index", () -> indexer == null ? "n/a" : indexer.getTargetIndex())
                .addData("Velocity", () -> indexer == null ? "n/a" : indexer.getVelocity());
        prettyTelem.addLine("Launcher")
                .addData("Current Velocity", () -> launcher == null ? "n/a" : launcher.getVelocity())
                .addData("Target Velocity", () -> launcher == null ? "n/a" : launcher.getTargetVelocity())
                .addData("Power", () -> launcher == null ? "n/a" : launcher.getPower())
                .addData("PID Result", () -> launcher == null ? "n/a" : launcher.getPidResult());
        prettyTelem.addLine("Turret")
                .addData("Current Angle", () -> turret == null ? "n/a" : turret.getCurrentPosition())
                .addData("Target Angle", () -> turret == null ? "n/a" : turret.getTargetPosition())
                .addData("Power", () -> turret == null ? "n/a" : turret.getPower())
                .addData("Bearing To Tag", () -> FireControlSystem.bearingToDepot);
        prettyTelem.addLine("Storage Controller")
                .addData("State", () -> volleyStorageManager == null ? "UNKNOWN" : volleyStorageManager.getState())
                .addData("Front Content", () -> indexerStorage == null ? "n/a" : indexerStorage.getFrontContent())
                .addData("Right Content", () -> indexerStorage == null ? "n/a" : indexerStorage.getRightContent())
                .addData("Left Content", () -> indexerStorage == null ? "n/a" : indexerStorage.getLeftContent())
                .addData("Indexer Velocity", () -> indexer == null ? "n/a" : indexer.getVelocity())
                .addData("Active Task", () -> volleyStorageManager == null ? "None" : volleyStorageManager.getActiveTaskName())
                .addData("Task Queue", () -> volleyStorageManager == null ? "[]" : volleyStorageManager.getTaskQueueSummary());
        prettyTelem.addLine("Color Sensor")
                .addData("Closest Match", () -> indexerStorage == null ? "N/A" : indexerStorage.getFrontClosestColorMatch())
                .addData("Front Sensor Hue", () -> indexerStorage == null ? "n/a" : indexerStorage.getFrontSensorHue())
                .addData("Front Sensor Distance", () -> frontColorSensor == null ? "n/a" : frontColorSensor.getDistance(DistanceUnit.MM));
    }

    private void resetPoseToCloseAutoStart() {
        if (driveBase == null || driveBase.getFollower() == null) {
            return;
        }
        double poseX = AutonomousConfiguration.closeAutoStartPoseXIn(allianceColor);
        double poseY = AutonomousConfiguration.closeAutoStartPoseYIn(allianceColor);
        double headingDeg = AutonomousConfiguration.closeAutoStartPoseHeadingDeg(allianceColor);
        double flippedHeadingDeg = headingDeg + 180.0;
        driveBase.getFollower().setPose(
                new com.pedropathing.geometry.Pose(
                        poseX,
                        poseY,
                        Math.toRadians(flippedHeadingDeg)
                )
        );
        persistMatchStateIfDue(true);
    }

    private void resetTurretZeroToCurrent() {
        if (turret == null) {
            return;
        }
        turret.setCurrentAsZero();
        turret.setTargetPosition(0);
        persistMatchStateIfDue(true);
    }

    private boolean applyZeroTrim(SmartGamepad gamepad) {
        if (gamepad.leftBumper && turret != null) {
            turret.setTargetPosition(0);
        }

        double triggerDelta = gamepad.rightTrigger - gamepad.leftTrigger;
        if (gamepad.leftBumper && turret != null) {
            if (Math.abs(triggerDelta) > 1e-6) {
                int turretBump = (int) (triggerDelta * turretZeroBumpTicksPerTriggerUnit);
                if (turretBump != 0) {
                    turret.bumpZero(turretBump);
                }
            }
            return true;
        }

        if (Math.abs(triggerDelta) <= 1e-6) {
            return false;
        }

        if (indexer != null) {
            int indexerBump = (int) (triggerDelta * indexerZeroBumpTicksPerTriggerUnit);
            if (indexerBump != 0) {
                indexer.bumpZero(indexerBump);
            }
        }
        return false;
    }

    private void applyTurretTrimLedOverride(boolean turretTrimActive) {
        if (fcs == null) {
            return;
        }
        fcs.setLedOverrideColor(turretTrimActive ? turretZeroTrimLedColor : null);
    }

    private void applyMaintenanceTouchpadPoseTrim(SmartGamepad gamepad) {
        if (driveBase == null || driveBase.getFollower() == null || !gamepad.touchpadFinger1) {
            return;
        }

        double deltaX = gamepad.touchpadFinger1DeltaX();
        double deltaY = gamepad.touchpadFinger1DeltaY();
        if (Math.abs(deltaX) <= 1e-6 && Math.abs(deltaY) <= 1e-6) {
            return;
        }

        com.pedropathing.geometry.Pose currentPose = driveBase.getFollower().getPose();
        double trimScale = maintenancePoseTrimInchesPerTouchpadUnit;
        double newX = currentPose.getX() + (deltaX * trimScale);
        double newY = currentPose.getY() - (deltaY * trimScale);

        driveBase.getFollower().setPose(
                new com.pedropathing.geometry.Pose(
                        newX,
                        newY,
                        currentPose.getHeading()
                )
        );
    }
}
