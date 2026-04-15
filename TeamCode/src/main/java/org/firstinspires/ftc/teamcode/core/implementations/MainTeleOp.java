package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.*;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.core.teleoptasks.CancelReason;
import org.firstinspires.ftc.teamcode.core.teleoptasks.TeleOpTaskContext;
import org.firstinspires.ftc.teamcode.core.teleoptasks.TeleOpTaskManager;
import org.firstinspires.ftc.teamcode.core.teleoptasks.tasks.FarFiringTask;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartCameraColorSensor;
import org.firstinspires.ftc.teamcode.hardware.SmartLEDIndicator;
import org.firstinspires.ftc.teamcode.hardware.SmartLimelight3A;
import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Configurable
@TeleOp(name = "1 - Main TeleOp")
public class MainTeleOp extends TeleOpCore {
    private enum TelemetryMode {
        DISTILLED,
        DEBUG
    }

    private enum ObeliskRelocalizeState {
        IDLE,
        WAITING_FOR_OBELISK
    }

    private static final Logger log = LoggerFactory.getLogger(MainTeleOp.class);
    protected static DriveBase driveBase;
    protected static Feeder feeder;
    protected static Turret turret;
    protected static Collector collector;
    protected static Indexer indexer;
    protected static Launcher launcher;
    protected static StorageController storageController;
    protected static SmartCameraColorSensor frontCameraSensor;
    protected static Hood hood;
    protected static SimpleFCS fcs;
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
    public static double taskBaseBlueXIn = -55.5, taskBaseBlueYIn = -48, taskBaseBlueHeadingDeg = -105;
    public static double taskBaseRedXIn = -55.5, taskBaseRedYIn = 48, taskBaseRedHeadingDeg = 105;
    public static double taskDriveToBaseTimeoutSec = 4.0;
    public static double taskWaitForFullTimeoutSec = 10.0;
    public static double taskReturnTimeoutSec = 4.0;
    public static double taskReadyToFireTimeoutSec = 4.0;
    public static double taskStorageDrainTimeoutSec = 10.0;
    public static double taskRepeatDelaySec = 0.5;
    public static double taskInterruptDeadband = 0.08;
    public static double obeliskFieldXIn = 73.0;
    public static double obeliskFieldYIn = 0.0;
//    public static double turretAxisOffsetRobotXIn = -3.0;
//    public static double turretAxisOffsetRobotYIn = -2.0;
//    public static double cameraOffsetTurretXIn = 0.0;
//    public static double cameraOffsetTurretYIn = 6.5;
//    public static double cameraYawOffsetDeg = 0.0;
    public static double indexerZeroBumpTicksPerTriggerUnit = 20.0;
    public static double turretZeroBumpTicksPerTriggerUnit = -75.0;
    public static SmartLEDIndicator.IndicatorColor turretZeroTrimLedColor = SmartLEDIndicator.IndicatorColor.INDIGO;
    public static double maintenancePoseTrimInchesPerTouchpadUnit = 12.0;
    public static double closeAutoStartBluePoseXIn = 54.7;
    public static double closeAutoStartBluePoseYIn = 50.8;
    public static double closeAutoStartBluePoseHeadingDeg = -127.483;
    public static double closeAutoStartRedPoseXIn = 54.7;
    public static double closeAutoStartRedPoseYIn = -50.8;
    public static double closeAutoStartRedPoseHeadingDeg = 127.483;
    private MatchStateStore.AllianceColor allianceColor = defaultAllianceColor;
    private MatchStateStore.Snapshot startupSnapshot;
    private boolean loadedFreshSnapshot = false;
    private long lastMatchStateSaveMs = 0;
    private TeleOpTaskManager teleOpTaskManager;
    private TelemetryMode telemetryMode = TelemetryMode.DEBUG;
    private ObeliskRelocalizeState obeliskRelocalizeState = ObeliskRelocalizeState.IDLE;
    private String obeliskRelocalizeStatus = "IDLE";
    private double lastSolvedObeliskPoseXIn = 0;
    private double lastSolvedObeliskPoseYIn = 0;
    private double lastSeenObeliskCamXIn = 0;
    private double lastSeenObeliskCamZIn = 0;
    private boolean clearLeftSlotWhenFeederReturns = false;

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
        turret = null;
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
        fcs = null;
        limelight = null;
        limelight3A = null;
    }

    @Override
    protected void onInitialize(){
        //noinspection DuplicatedCode
        resetSubsystemReferences();
        startupSnapshot = MatchStateStore.getFreshSnapshot(Math.max(1000L, (long) matchStateFreshnessMs));
        allianceColor = startupSnapshot != null
                ? MatchStateStore.parseAllianceColor(startupSnapshot.allianceColor, defaultAllianceColor)
                : defaultAllianceColor;
        loadedFreshSnapshot = startupSnapshot != null;
        lastMatchStateSaveMs = 0;
        teleOpTaskManager = null;
        telemetryMode = TelemetryMode.DEBUG;
        obeliskRelocalizeState = ObeliskRelocalizeState.IDLE;
        obeliskRelocalizeStatus = "IDLE";
        clearLeftSlotWhenFeederReturns = false;

        DriveBaseMotorConfig.DriveBaseMotorConfigBuilder configBuilder = new DriveBaseMotorConfig.DriveBaseMotorConfigBuilder();
        configBuilder.leftFront("LFront", Direction.FORWARD);
        configBuilder.leftRear("LRear", Direction.FORWARD);
        configBuilder.rightFront("RFront", Direction.REVERSE);
        configBuilder.rightRear("RRear", Direction.FORWARD);

        Constants.setMecanumMaxPower(teleOpFollowerMaxPower);

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
            feeder = new Feeder(
                    hardwareMap.get(CRServo.class, "feederServo"),
                    Hardware.getPotentiometer("feederPotentiometer", 270, 3.3)
            );
            indexer = new Indexer(Hardware.getMotor("indexerMotor", true));
            collector = new Collector(Hardware.getMotor("collectorMotor"));
            frontCameraSensor = Hardware
                    .getCamera("colorCamera", new Pose(0, 0, 0))
                    .asColorSensor();
            storageController = new StorageController(
                    feeder,
                    indexer,
                    collector,
                    frontCameraSensor,
                    Hardware.getLEDIndicator("leftLED"),
                    Hardware.getLEDIndicator("rightLED"),
                    Hardware.getLEDIndicator("frontLED")
            );
            applyPersistedStorageStateIfAvailable();
        } catch (Exception e) {
            prettyTelem.error("Feeder failed to initialize, skipping: " + e.getMessage());
        }

        try {
            launcher = new Launcher(Hardware.getMotor("launcherMotor"));
            hood = new Hood(Hardware.getServo("hoodServo"));
            turret = new Turret(Hardware.getMotor("turretMotor"), Hardware.getMotor("turretMotor").getEncoder());
            limelight = Hardware.getLimelight("limelight");
            limelight.setPipeline(0);
            limelight.start();

            fcs = new SimpleFCS(
                    turret,
                    hood,
                    launcher,
                    limelight,
                    Hardware.getLEDIndicator("launcherLED"),
                    driveBase,
                    null
            );
            fcs.setFallbackVelocity(launchVelocity);
            fcs.setAllianceColor(allianceColor);
        } catch (Exception e) {
            prettyTelem.error("Fire Control System failed to initialize, skipping: " + e.getMessage());
        }

        teleOpTaskManager = new TeleOpTaskManager(
                new TeleOpTaskContext(
                        () -> driveBase,
                        () -> storageController,
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
        boolean hasDriverOverrideInput = Math.abs(driveX) > taskInterruptDeadband
                || Math.abs(driveY) > taskInterruptDeadband
                || Math.abs(driveTurn) > taskInterruptDeadband;

        boolean turretTrimActive = false;

        if (gamepad2.startPressed()) {
            resetPoseToCloseAutoStart();
        }
        if (gamepad2.bPressed()) {
            resetTurretZeroToCurrent();
        }
        turretTrimActive |= applyZeroTrim(gamepad2);

        boolean maintenanceMode = gamepad1.leftStickButton;
        if (maintenanceMode) {
            if (gamepad1.startPressed()) {
                resetPoseToCloseAutoStart();
            }

            if (gamepad1.bPressed()) {
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
                teleOpTaskManager.start(new FarFiringTask(
                        taskBaseBlueXIn,
                        taskBaseBlueYIn,
                        taskBaseBlueHeadingDeg,
                        taskBaseRedXIn,
                        taskBaseRedYIn,
                        taskBaseRedHeadingDeg,
                        taskDriveToBaseTimeoutSec,
                        taskWaitForFullTimeoutSec,
                        taskReturnTimeoutSec,
                        taskReadyToFireTimeoutSec,
                        taskStorageDrainTimeoutSec,
                        taskRepeatDelaySec
                ));
            }
        }

        if(driveBase != null && (teleOpTaskManager == null || !teleOpTaskManager.hasActiveTask())){
            driveBase.moveUsingPower(driveX, driveY, driveTurn);
        }

        if(feeder != null){
            if(gamepad1.yPressed()){
                feeder.trigger();
                clearLeftSlotWhenFeederReturns = true;
            }
        }

        if(storageController != null){
            if(gamepad1.leftBumperPressed() && feeder.getState() == Feeder.State.RESTING){
                indexer.advanceIndexCounterclockwise();
                storageController.dropFreshFlag();
            }
            if(gamepad1.rightBumperPressed() && feeder.getState() == Feeder.State.RESTING){
                indexer.advanceIndexClockwise();
                storageController.dropFreshFlag();
            }
            if(gamepad1.dpadLeftPressed()){
                if(fcs == null || fcs.isLauncherRunning()) {
                    storageController.loadGreen();
                }
            }
            if(gamepad1.dpadRightPressed()){
                if(fcs == null || fcs.isLauncherRunning()) {
                    storageController.loadPurple();
                }
            }
            if(gamepad1.startPressed()){
                storageController.setLeftContent(StorageController.SlotContent.OPEN);
                storageController.setRightContent(StorageController.SlotContent.OPEN);
                storageController.setFrontContent(StorageController.SlotContent.OPEN);
            }
        }
        if (gamepad1.sharePressed()) {
            allianceColor = allianceColor.opposite();
            if (fcs != null) {
                fcs.setAllianceColor(allianceColor);
            }
            persistMatchStateIfDue(true);
        }

        if(fcs != null){
            if(gamepad1.xPressed()){
                fcs.toggleLauncher();
            }
            if(gamepad1.dpadDownPressed()){
                launchVelocity -= 50;
                fcs.setFallbackVelocity(launchVelocity);
            }

            if(gamepad1.dpadUpPressed()){
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

    @Override
    protected void beforeTick(){
        tickTimer.reset();
    }

    @Override
    protected void onTick(){
        if (clearLeftSlotWhenFeederReturns && feeder != null && feeder.getState() == Feeder.State.RESTING) {
            Double lastTriggerDurationMs = feeder.getLastTriggerDurationMs();
            if(lastTriggerDurationMs != null && lastTriggerDurationMs < 2000 && storageController != null){
                storageController.setLeftContent(StorageController.SlotContent.OPEN);
            }
            clearLeftSlotWhenFeederReturns = false;
        }

        if(storageController != null){
            storageController.tick();
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
                log.error("FCS tick failed; disabling FCS", e);
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
        if (startupSnapshot == null || storageController == null || indexer == null) {
            return;
        }

        storageController.setFrontContent(
                MatchStateStore.parseSlotContent(startupSnapshot.frontContent, StorageController.SlotContent.OPEN)
        );
        storageController.setRightContent(
                MatchStateStore.parseSlotContent(startupSnapshot.rightContent, StorageController.SlotContent.OPEN)
        );
        storageController.setLeftContent(
                MatchStateStore.parseSlotContent(startupSnapshot.leftContent, StorageController.SlotContent.OPEN)
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
        MatchStateStore.saveSnapshot(driveBase, storageController, indexer, turret, allianceColor, obeliskTag);
        lastMatchStateSaveMs = now;
    }

    private void rebuildTelemetryLayout() {
        prettyTelem.resetLayout();
        registerDebugTelemetry();
    }

    private void registerDistilledTelemetry() {
        prettyTelem.addLine("HUD")
                .addData("Mode", () -> telemetryMode.name())
                .addData("Alliance", () -> allianceColor.name())
                .addData("Task", () -> teleOpTaskManager == null ? "NONE" : teleOpTaskManager.activeTaskState());
        prettyTelem.addLine("Pose")
                .addData("X", () -> driveBase == null ? "n/a" : driveBase.getPoseSimple().x())
                .addData("Y", () -> driveBase == null ? "n/a" : driveBase.getPoseSimple().y())
                .addData("Hdg", () -> driveBase == null ? "n/a" : driveBase.getPoseSimple().heading());
        prettyTelem.addLine("FCS")
                .addData("Running", () -> fcs != null && fcs.isLauncherRunning())
                .addData("Launcher Spun", () -> fcs != null && fcs.isLauncherSpun())
                .addData("Turret Aligned", () -> fcs != null && fcs.isTurretAligned());
        prettyTelem.addLine("Storage")
                .addData("G/P/O", () -> storageController == null
                        ? "n/a"
                        : storageController.countGreen() + "/" + storageController.countPurple() + "/" + storageController.countOpen())
                .addData("Full", () -> storageController != null && storageController.isFull())
                .addData("Busy", () -> storageController != null && !storageController.allTasksComplete());
        prettyTelem.addLine("Relocalize")
                .addData("Status", () -> obeliskRelocalizeStatus);
        prettyTelem.addLine("Controls")
                .addData("G1 LSB+Start", () -> "Reset pose close auto")
                .addData("G1 LSB+B", () -> "Set turret current=zero")
                .addData("G1 LSB+RT/LT", () -> "Indexer zero trim")
                .addData("G1 LSB+LB+RT/LT", () -> "Turret zero trim + target zero")
                .addData("G2 Start/B", () -> "Reset pose / turret current=zero")
                .addData("G2 RT/LT", () -> "Indexer zero trim")
                .addData("G2 LB+RT/LT", () -> "Turret zero trim + target zero");
    }

    private void registerDebugTelemetry() {
        prettyTelem.addData("Tick Time", () -> tickTimeFilter.compute(tickTimer.milliseconds()));

        prettyTelem.addLine("Match")
                .addData("Telemetry Mode", () -> telemetryMode.name())
                .addData("Alliance", () -> allianceColor.name())
                .addData("Loaded Fresh Snapshot", () -> loadedFreshSnapshot);
        prettyTelem.addLine("TeleOp Task")
                .addData("Task Active", () -> teleOpTaskManager != null && teleOpTaskManager.hasActiveTask())
                .addData("Task Name", () -> teleOpTaskManager == null ? "NONE" : teleOpTaskManager.activeTaskName())
                .addData("Task State", () -> teleOpTaskManager == null ? "IDLE" : teleOpTaskManager.activeTaskState())
                .addData("Last Exit", () -> teleOpTaskManager == null ? "NONE" : teleOpTaskManager.lastExitReason());
        prettyTelem.addLine("Obelisk Relocalize")
                .addData("State", () -> obeliskRelocalizeState.name())
                .addData("Status", () -> obeliskRelocalizeStatus)
                .addData("Solved X", () -> lastSolvedObeliskPoseXIn)
                .addData("Solved Y", () -> lastSolvedObeliskPoseYIn)
                .addData("Tag Cam X(in)", () -> lastSeenObeliskCamXIn)
                .addData("Tag Cam Z(in)", () -> lastSeenObeliskCamZIn);
        prettyTelem.addLine("Controls")
                .addData("G1 LSB+Start", () -> "Reset pose to close auto start")
                .addData("G1 LSB+B", () -> "Set turret current angle as zero")
                .addData("G1 LSB+RT/LT", () -> "Indexer zero trim")
                .addData("G1 LSB+LB+RT/LT", () -> "Turret zero trim + target zero")
                .addData("G2 Start/B", () -> "Reset pose / turret current=zero")
                .addData("G2 RT/LT", () -> "Indexer zero trim")
                .addData("G2 LB+RT/LT", () -> "Turret zero trim + target zero");

        prettyTelem.addLine("Localization")
                .addData("X", () -> driveBase == null ? "n/a" : driveBase.getPoseSimple().x())
                .addData("Y", () -> driveBase == null ? "n/a" : driveBase.getPoseSimple().y())
                .addData("Heading", () -> driveBase == null ? "n/a" : driveBase.getPoseSimple().heading());

        prettyTelem.addLine("Feeder")
                .addData("Current Angle", () -> feeder == null ? "n/a" : feeder.getCurrentPosition())
                .addData("Target Angle", () -> feeder == null ? "n/a" : feeder.getTargetPosition());
        prettyTelem.addLine("Hood")
                .addData("Target Pos", () -> hood == null ? "n/a" : hood.getTargetPosition());
        prettyTelem.addLine("Indexer")
                .addData("Current Position", () -> indexer == null ? "n/a" : indexer.getCurrentPosition())
                .addData("Target Position", () -> indexer == null ? "n/a" : indexer.getTargetPosition())
                .addData("Is Busy", () -> indexer != null && indexer.isBusy())
                .addData("Current Index", () -> indexer == null ? "n/a" : indexer.getCurrentIndex())
                .addData("Target Index", () -> indexer == null ? "n/a" : indexer.getTargetIndex())
                .addData("Velocity", () -> indexer.getVelocity());
        prettyTelem.addLine("Launcher")
                .addData("Current Velocity", () -> launcher == null ? "n/a" : launcher.getVelocity())
                .addData("Target Velocity", () -> launcher == null ? "n/a" : launcher.getTargetVelocity())
                .addData("Power", () -> launcher == null ? "n/a" : launcher.getPower())
                .addData("PID Result", () -> launcher == null ? "n/a" : launcher.getPidResult());
        prettyTelem.addLine("Turret")
                .addData("Current Angle", () -> turret == null ? "n/a" : turret.getCurrentPosition())
                .addData("Target Angle", () -> turret == null ? "n/a" : turret.getTargetPosition())
                .addData("Power", () -> turret == null ? "n/a" : turret.getPower())
                .addData("Bearing To Tag", () -> SimpleFCS.bearingToDepot);
        prettyTelem.addLine("Storage Controller")
                .addData("State", () -> storageController == null ? "UNKNOWN" : storageController.getState())
                .addData("Front Content", () -> storageController == null ? "n/a" : storageController.getFrontContent())
                .addData("Right Content", () -> storageController == null ? "n/a" : storageController.getRightContent())
                .addData("Left Content", () -> storageController == null ? "n/a" : storageController.getLeftContent())
                .addData("Indexer Velocity", () -> indexer == null ? "n/a" : indexer.getVelocity())
                .addData("Active Task", () -> storageController == null ? "None" : storageController.getActiveTaskName())
                .addData("Is Jam Correcting", () -> storageController != null && storageController.isJamCorrecting())
                .addData("Jam Timer", () -> storageController == null ? "n/a" : storageController.getJamTimerMs())
                .addData("Task Queue", () -> storageController == null ? "[]" : storageController.getTaskQueueSummary());
        prettyTelem.addLine("Color Sensor")
                .addData("Sensor Color", () -> storageController == null ? "NONE" : storageController.getFrontSensorColorName())
                .addData("Closest Match", () -> storageController == null ? "N/A" : storageController.getFrontClosestColorMatch())
                .addData("Front Sensor Hue", () -> storageController == null ? "n/a" : storageController.getFrontSensorHue())
                .addData("Front Sensor Saturation", () -> storageController == null ? "n/a" : storageController.getFrontSensorSaturation())
                .addData("Front Sensor Value", () -> storageController == null ? "n/a" : storageController.getFrontSensorValue());
    }

//    private void processObeliskRelocalization() {
//        if (obeliskRelocalizeState != ObeliskRelocalizeState.WAITING_FOR_OBELISK) {
//            return;
//        }
//        if (driveBase == null || driveBase.getFollower() == null || turret == null || limelight == null) {
//            obeliskRelocalizeState = ObeliskRelocalizeState.IDLE;
//            obeliskRelocalizeStatus = "Canceled: subsystem unavailable";
//            if (fcs != null) {
//                fcs.setDepotAutoAimEnabled(true);
//            }
//            return;
//        }
//
//        SmartLimelight3A.AprilTag obelisk = limelight.getFirstObelisk();
//        if (obelisk == null || obelisk.tagInCameraPose() == null || obelisk.tagInCameraPose().getPosition() == null) {
//            return;
//        }
//
//        Pose currentPose = driveBase.getPoseSimple();
//        double headingDeg = currentPose.heading();
//        double turretDeg = turret.getCurrentPosition();
//
//        double xCamIn = metersToInches(obelisk.tagInCameraPose().getPosition().x);
//        double zCamIn = metersToInches(obelisk.tagInCameraPose().getPosition().z);
//        lastSeenObeliskCamXIn = xCamIn;
//        lastSeenObeliskCamZIn = zCamIn;
//
//        double cameraHeadingDeg = headingDeg - (turretDeg + cameraYawOffsetDeg);
//        double cameraHeadingRad = Math.toRadians(cameraHeadingDeg);
//        double camToTagWorldX = Math.cos(cameraHeadingRad) * zCamIn + Math.sin(cameraHeadingRad) * xCamIn;
//        double camToTagWorldY = Math.sin(cameraHeadingRad) * zCamIn - Math.cos(cameraHeadingRad) * xCamIn;
//
//        double cameraWorldX = obeliskFieldXIn - camToTagWorldX;
//        double cameraWorldY = obeliskFieldYIn - camToTagWorldY;
//
//        double turretHeadingRad = Math.toRadians(headingDeg - turretDeg);
//        double cameraOffsetWorldX = rotateX(cameraOffsetTurretXIn, cameraOffsetTurretYIn, turretHeadingRad);
//        double cameraOffsetWorldY = rotateY(cameraOffsetTurretXIn, cameraOffsetTurretYIn, turretHeadingRad);
//        double turretAxisWorldX = cameraWorldX - cameraOffsetWorldX;
//        double turretAxisWorldY = cameraWorldY - cameraOffsetWorldY;
//
//        double headingRad = Math.toRadians(headingDeg);
//        double turretAxisOffsetWorldX = rotateX(turretAxisOffsetRobotXIn, turretAxisOffsetRobotYIn, headingRad);
//        double turretAxisOffsetWorldY = rotateY(turretAxisOffsetRobotXIn, turretAxisOffsetRobotYIn, headingRad);
//        double robotWorldX = turretAxisWorldX - turretAxisOffsetWorldX;
//        double robotWorldY = turretAxisWorldY - turretAxisOffsetWorldY;
//
//        driveBase.getFollower().setPose(
//                new com.pedropathing.geometry.Pose(
//                        robotWorldX,
//                        robotWorldY,
//                        Math.toRadians(headingDeg)
//                )
//        );
//
//        lastSolvedObeliskPoseXIn = robotWorldX;
//        lastSolvedObeliskPoseYIn = robotWorldY;
//        obeliskRelocalizeState = ObeliskRelocalizeState.IDLE;
//        obeliskRelocalizeStatus = "Solved from obelisk " + obelisk.type();
//        if (fcs != null) {
//            fcs.setDepotAutoAimEnabled(true);
//        }
//        persistMatchStateIfDue(true);
//    }

    private static double metersToInches(double meters) {
        return meters * 39.37007874015748;
    }

    private static double rotateX(double x, double y, double angleRad) {
        return x * Math.cos(angleRad) - y * Math.sin(angleRad);
    }

    private static double rotateY(double x, double y, double angleRad) {
        return x * Math.sin(angleRad) + y * Math.cos(angleRad);
    }

    private void resetPoseToCloseAutoStart() {
        if (driveBase == null || driveBase.getFollower() == null) {
            obeliskRelocalizeStatus = "Pose reset unavailable (drive/follower missing)";
            return;
        }
        boolean isRedAlliance = allianceColor == MatchStateStore.AllianceColor.RED;
        double poseX = isRedAlliance ? closeAutoStartRedPoseXIn : closeAutoStartBluePoseXIn;
        double poseY = isRedAlliance ? closeAutoStartRedPoseYIn : closeAutoStartBluePoseYIn;
        double headingDeg = isRedAlliance ? closeAutoStartRedPoseHeadingDeg : closeAutoStartBluePoseHeadingDeg;
        double flippedHeadingDeg = headingDeg + 180.0;
        driveBase.getFollower().setPose(
                new com.pedropathing.geometry.Pose(
                        poseX,
                        poseY,
                        Math.toRadians(flippedHeadingDeg)
                )
        );
        obeliskRelocalizeStatus = "Pose reset to "
                + (isRedAlliance ? "red" : "blue")
                + " close auto start (180 flipped)";
        persistMatchStateIfDue(true);
    }

    private void resetTurretZeroToCurrent() {
        if (turret == null) {
            obeliskRelocalizeStatus = "Turret zero reset unavailable";
            return;
        }
        turret.setCurrentAsZero();
        turret.setTargetPosition(0);
        obeliskRelocalizeStatus = "Turret zeroed at current position";
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
        obeliskRelocalizeStatus = "Pose trim via touchpad";
    }
}
