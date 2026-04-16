package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.mechanisms.Collector;
import org.firstinspires.ftc.teamcode.components.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.hardware.ColorMatchConfig;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.SmartColorSensor;
import org.firstinspires.ftc.teamcode.hardware.SmartLEDIndicator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

@Configurable
@Disabled
@TeleOp(name = "6 - Auto Color Sensor Tuner")
public class ColorSensorAutoTuningTeleOp extends TeleOpCore {
    // Known slot content at opmode start:
    // slot 0 = front, slot 1 = back-right, slot 2 = back-left.
    // 0 = OPEN, 1 = GREEN, 2 = PURPLE.
    public static int slot0Content = 0;
    public static int slot1Content = 1;
    public static int slot2Content = 2;

    // Sampling configuration.
    public static int sampleCycles = 6;
    public static int samplesPerPosition = 8;
    public static int moveTimeoutMs = 1750;
    public static int settleHoldMs = 120;
    public static int readSettleHoldMs = 320;
    public static int readSettleStableMs = 160;
    public static int readSettleMaxExtraMs = 1200;
    public static double readSettleVelocityThreshold = 2.0;
    public static int sampleDelayMs = 12;

    // Validation test configuration.
    public static int testCycles = 3;
    public static int testSamplesPerPosition = 6;
    public static int testSampleDelayMs = 12;
    public static double passingScore = 0.85;

    // Jam handling (modeled after StorageController).
    public static double jamVelocityThreshold = 25;
    public static int jamHoldMs = 300;
    public static int moveTimeoutRecoveryAttempts = 2;
    public static int moveTimeoutRecoveryExtraMs = 1000;
    public static int moveTimeoutRecoveryHoldMs = 220;

    // Collector assist while indexer rotates to help avoid jams.
    // Positive power is inward intake on this robot.
    public static double collectorAssistPower = 0.4;
    public static int collectorAssistKeepAliveMs = 450;
    public static double collectorAssistMovingVelocityThreshold = 2.0;

    // Fitting configuration.
    public static double minHueToleranceDeg = 4.0;
    public static double maxHueToleranceDeg = 45.0;
    public static double hueSafetyMarginDeg = 2.0;
    public static double hueSpreadPercentile = 0.95;

    private Indexer indexer;
    private Collector collector;
    private SmartColorSensor frontSensor;
    private SmartColorSensor leftSensor;
    private SmartColorSensor rightSensor;
    private SmartLEDIndicator frontLED;
    private SmartLEDIndicator leftLED;
    private SmartLEDIndicator rightLED;

    private final SensorSamples frontSamples = new SensorSamples();
    private final SensorSamples leftSamples = new SensorSamples();
    private final SensorSamples rightSamples = new SensorSamples();

    private KnownContent[] knownSlots = new KnownContent[3];
    private String status = "Init";
    private int positionsSampled = 0;
    private int totalSamples = 0;
    private long baseIndex = 0;
    private int baseNormalizedIndex = 0;

    private String frontResult = "n/a";
    private String leftResult = "n/a";
    private String rightResult = "n/a";
    private String errors = "none";
    private String saveStatus = "not saved";
    private String overallTestScore = "not run";
    private String frontTestScore = "n/a";
    private String leftTestScore = "n/a";
    private String rightTestScore = "n/a";
    private String statusPrimary = "Ready";
    private String statusDetail = "Press A to tune+test, X to test only";
    private String statusFront = "-";
    private String statusRight = "-";
    private String statusLeft = "-";
    private String statusActions = "A=tune+test, X=test only";
    private String statusResult = "-";

    private boolean isJamCorrecting = false;
    private boolean actionRunning = false;
    private int jamRecoveryCount = 0;
    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime jamCorrectionTimer = new ElapsedTime();
    private final ElapsedTime collectorNoMotionTimer = new ElapsedTime();
    private boolean collectorAssistLatched = false;

    @Override
    protected void onInitialize() {

        try {
            indexer = new Indexer(Hardware.getMotor("indexerMotor", true));
            frontSensor = Hardware.getColorSensor(ColorMatchConfig.FRONT_SENSOR_NAME);
            leftSensor = Hardware.getColorSensor(ColorMatchConfig.FRONT_SENSOR_NAME);
            rightSensor = Hardware.getColorSensor(ColorMatchConfig.FRONT_SENSOR_NAME);
            frontLED = tryGetLed("frontLED");
            leftLED = tryGetLed("leftLED");
            rightLED = tryGetLed("rightLED");
            collector = tryGetCollector("collectorMotor");
            status = "Ready";
            setAllLeds(SmartLEDIndicator.IndicatorColor.BLUE);
        } catch (Exception e) {
            status = "Init failed";
            errors = e.getClass().getSimpleName() + ": " + e.getMessage();
            prettyTelem.error("Auto color tuner init failed: " + errors);
            setAllLeds(SmartLEDIndicator.IndicatorColor.RED);
        }

        if ("Init failed".equals(status)) {
            setStatusError("Init failed", errors);
        } else {
            setStatusIdle("Ready");
        }

        prettyTelem.addLine("Status").addData("Now", this::buildStatusSummary);
    }

    @Override
    protected void onRun() {
        if (indexer == null || frontSensor == null || leftSensor == null || rightSensor == null) {
            setAllLeds(SmartLEDIndicator.IndicatorColor.RED);
            return;
        }
        // Capture the slot-to-index baseline once at opmode start.
        baseIndex = indexer.getCurrentIndex();
        baseNormalizedIndex = Math.floorMod(indexer.getNormalizedCurrentIndex(), 3);
        runTuneAndValidation();
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        if (actionRunning) {
            return;
        }

        if (gamepad1.aPressed()) {
            runTuneAndValidation();
        }

        if (gamepad1.xPressed()) {
            runValidationOnly();
        }
    }

    @Override
    protected void beforeTick() {
        if (indexer != null) {
            indexer.tick();
        }
    }

    private void runTuneAndValidation() {
        actionRunning = true;
        try {
            if (!loadKnownSlots()) {
                setAllLeds(SmartLEDIndicator.IndicatorColor.RED);
                setStatusError("Invalid slot setup", errors);
                return;
            }
            if (!promptForCollectorVerification()) {
                status = "Cancelled";
                setStatusIdle("Cancelled: collector state not confirmed");
                setAllLeds(SmartLEDIndicator.IndicatorColor.BLUE);
                return;
            }

            ProfileFit frontBackup = getCurrentProfile(SensorRole.FRONT);
            ProfileFit leftBackup = getCurrentProfile(SensorRole.LEFT);
            ProfileFit rightBackup = getCurrentProfile(SensorRole.RIGHT);

            if (!autoTune()) {
                return;
            }

            PostTuneChoice choice = promptForPostTuneChoice();
            if (!opModeIsActive()) {
                return;
            }

            if (choice == PostTuneChoice.CANCEL) {
                applyFit(frontBackup, SensorRole.FRONT);
                applyFit(leftBackup, SensorRole.LEFT);
                applyFit(rightBackup, SensorRole.RIGHT);
                saveStatus = "skipped (cancelled)";
                status = "Cancelled";
                setStatusIdle("Cancelled: reverted tuned values");
                setAllLeds(SmartLEDIndicator.IndicatorColor.BLUE);
                return;
            }

            if (choice == PostTuneChoice.SAVE_NOW) {
                persistCurrentConfig();
                return;
            }

            boolean validationPassed = runValidationTest();
            if (!opModeIsActive()) {
                return;
            }

            if (!validationPassed) {
                applyFit(frontBackup, SensorRole.FRONT);
                applyFit(leftBackup, SensorRole.LEFT);
                applyFit(rightBackup, SensorRole.RIGHT);
                saveStatus = "skipped (validation failed)";
                status = "Validation failed";
                setStatusError("Validation failed; reverted tuned values", "Not saved");
                setAllLeds(SmartLEDIndicator.IndicatorColor.YELLOW);
                return;
            }

            if (!promptForSaveConfirmation()) {
                saveStatus = "skipped (user declined)";
                status = "Complete (not saved)";
                setStatusIdle("Validation passed; save skipped");
                setAllLeds(SmartLEDIndicator.IndicatorColor.YELLOW);
                return;
            }

            persistCurrentConfig();
        } finally {
            stopCollectorAssist();
            actionRunning = false;
        }
    }

    private void runValidationOnly() {
        actionRunning = true;
        try {
            if (!loadKnownSlots()) {
                setAllLeds(SmartLEDIndicator.IndicatorColor.RED);
                setStatusError("Invalid slot setup", errors);
                return;
            }
            if (!promptForCollectorVerification()) {
                status = "Cancelled";
                setStatusIdle("Cancelled: collector state not confirmed");
                setAllLeds(SmartLEDIndicator.IndicatorColor.BLUE);
                return;
            }
            runValidationTest();
        } finally {
            stopCollectorAssist();
            actionRunning = false;
        }
    }

    private void persistCurrentConfig() {
        status = "Saving";
        statusPrimary = "Saving tuned config...";
        statusDetail = "-";
        statusActions = "-";
        clearStatusLayout();
        setAllLeds(SmartLEDIndicator.IndicatorColor.AZURE);

        boolean saved = ColorMatchConfig.saveToPersistentStorage();
        saveStatus = saved ? "saved" : "failed (storage unavailable)";
        status = saved ? "Complete" : "Complete (not persisted)";
        statusPrimary = saved ? "Saved tuned config" : "Could not save (storage unavailable)";
        statusDetail = "Validation: " + overallTestScore;
        statusActions = "A=tune+test, X=test only";
        statusResult = "Save: " + saveStatus;
        setAllLeds(saved ? SmartLEDIndicator.IndicatorColor.GREEN : SmartLEDIndicator.IndicatorColor.YELLOW);
    }

    private boolean autoTune() {
        status = "Tuning";
        errors = "none";
        saveStatus = "not saved";
        overallTestScore = "not run";
        frontTestScore = "n/a";
        leftTestScore = "n/a";
        rightTestScore = "n/a";
        positionsSampled = 0;
        totalSamples = 0;
        jamRecoveryCount = 0;
        isJamCorrecting = false;
        setAllLeds(SmartLEDIndicator.IndicatorColor.ORANGE);

        frontSamples.clear();
        leftSamples.clear();
        rightSamples.clear();
        phaseTimer.reset();

        baseIndex = indexer.getCurrentIndex();
        if (!moveToIndex(baseIndex, moveTimeoutMs)) {
            status = "Failed to settle at base index";
            setAllLeds(SmartLEDIndicator.IndicatorColor.RED);
            setStatusError("Failed to settle indexer at start", "");
            return false;
        }

        int totalPositions = Math.max(1, sampleCycles) * 3;
        setStatusReadingProgress(0, totalPositions);
        for (int i = 0; i < totalPositions && opModeIsActive(); i++) {
            long targetIndex = baseIndex + i;
            if (!moveToIndex(targetIndex, moveTimeoutMs)) {
                status = "Move timeout at index " + targetIndex;
                setAllLeds(SmartLEDIndicator.IndicatorColor.RED);
                setStatusError("Move timeout while reading", "Index " + targetIndex);
                return false;
            }

            holdSteadyForRead(readSettleHoldMs);
            sampleCurrentPosition();
            positionsSampled++;
            setStatusReadingProgress(positionsSampled, totalPositions);
        }

        if (!opModeIsActive()) {
            status = "Stopped";
            setAllLeds(SmartLEDIndicator.IndicatorColor.OFF);
            setStatusIdle("Stopped");
            return false;
        }

        ProfileFit frontFit = fitProfile(frontSamples, SensorRole.FRONT);
        ProfileFit leftFit = fitProfile(leftSamples, SensorRole.LEFT);
        ProfileFit rightFit = fitProfile(rightSamples, SensorRole.RIGHT);

        applyFit(frontFit, SensorRole.FRONT);
        applyFit(leftFit, SensorRole.LEFT);
        applyFit(rightFit, SensorRole.RIGHT);

        frontResult = formatFit(frontFit);
        leftResult = formatFit(leftFit);
        rightResult = formatFit(rightFit);
        status = "Tuned (pending validation)";
        statusResult = "Tuned: F " + frontResult + " | R " + rightResult + " | L " + leftResult;
        return true;
    }

    private boolean moveToIndex(long targetIndex, long timeoutMs) {
        int attempts = Math.max(0, moveTimeoutRecoveryAttempts);
        long baseTimeout = Math.max(100, timeoutMs);
        long extraPerAttempt = Math.max(0, moveTimeoutRecoveryExtraMs);

        for (int attempt = 0; attempt <= attempts && opModeIsActive(); attempt++) {
            long attemptTimeout = baseTimeout + (attempt * extraPerAttempt);
            indexer.setTargetIndex(targetIndex);
            jamTimer.reset();
            ElapsedTime timer = new ElapsedTime();

            while (opModeIsActive() && timer.milliseconds() < attemptTimeout) {
                updateIndexerWithJamRecovery(targetIndex);
                if (!isJamCorrecting && !indexer.isBusy() && indexer.getTargetIndex() == targetIndex) {
                    return true;
                }
            }

            if (!opModeIsActive()) {
                return true;
            }

            if (attempt < attempts) {
                statusPrimary = String.format(
                        Locale.US,
                        "Move timeout, recovering %d/%d",
                        attempt + 1,
                        attempts
                );
                statusDetail = String.format(Locale.US, "Target index: %d", targetIndex);
                statusActions = "-";

                indexer.setTargetIndex(indexer.getCurrentIndex());
                holdSteady(Math.max(0, moveTimeoutRecoveryHoldMs));
                jamTimer.reset();
            }
        }

        return false;
    }

    private void holdSteady(long holdMs) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < holdMs) {
            updateIndexerWithJamRecovery(indexer.getTargetIndex());
        }
    }

    private void holdSteadyForRead(long holdMs) {
        long minimumHoldMs = Math.max(0, holdMs);
        long stableMs = Math.max(0, readSettleStableMs);
        long maxExtraMs = Math.max(0, readSettleMaxExtraMs);
        double velocityThreshold = Math.max(0.0, readSettleVelocityThreshold);

        ElapsedTime totalTimer = new ElapsedTime();
        ElapsedTime restTimer = new ElapsedTime();
        boolean wasResting = false;
        long maxTotalMs = minimumHoldMs + maxExtraMs;

        while (opModeIsActive() && totalTimer.milliseconds() < maxTotalMs) {
            updateIndexerWithJamRecovery(indexer.getTargetIndex());

            boolean resting = !isJamCorrecting
                    && !indexer.isBusy()
                    && Math.abs(indexer.getVelocity()) <= velocityThreshold;

            if (resting) {
                if (!wasResting) {
                    restTimer.reset();
                    wasResting = true;
                }
            } else {
                wasResting = false;
            }

            boolean heldLongEnough = totalTimer.milliseconds() >= minimumHoldMs;
            boolean stableLongEnough = stableMs == 0 || (wasResting && restTimer.milliseconds() >= stableMs);
            if (heldLongEnough && stableLongEnough) {
                return;
            }
        }
    }

    private void sampleCurrentPosition() {
        for (int i = 0; i < Math.max(1, samplesPerPosition) && opModeIsActive(); i++) {
            updateIndexerWithJamRecovery(indexer.getTargetIndex());

            int normalized = Math.floorMod(indexer.getNormalizedCurrentIndex(), 3);
            int frontSlotRelativeToStart = Math.floorMod(normalized - baseNormalizedIndex, 3);
            KnownContent frontExpected = knownSlots[frontSlotRelativeToStart];
            KnownContent leftExpected = knownSlots[(frontSlotRelativeToStart + 1) % 3];
            KnownContent rightExpected = knownSlots[(frontSlotRelativeToStart + 2) % 3];

            if (!isJamCorrecting) {
                setExpectedContentLeds(frontExpected, leftExpected, rightExpected);
            }

            addSample(frontSamples.get(frontExpected), frontSensor.getHSV());
            addSample(leftSamples.get(leftExpected), leftSensor.getHSV());
            addSample(rightSamples.get(rightExpected), rightSensor.getHSV());

            totalSamples += 3;

            if (sampleDelayMs > 0) {
                sleep(sampleDelayMs);
            }
        }
    }

    private void updateIndexerWithJamRecovery(long desiredTarget) {
        indexer.tick();
        checkForIndexerJam();
        updateCollectorAssist();

        if (isJamCorrecting) {
            setStatusJamCorrecting();
        }

        if (isJamCorrecting && !indexer.isBusy()) {
            isJamCorrecting = false;
            jamRecoveryCount++;
            indexer.setTargetIndex(desiredTarget);
            jamTimer.reset();
            status = "Resuming move";
            setAllLeds(SmartLEDIndicator.IndicatorColor.ORANGE);
        }

        frameworkTick();
    }

    private void updateCollectorAssist() {
        if (collector == null) {
            return;
        }

        boolean movingNow = indexer.isBusy()
                || Math.abs(indexer.getVelocity()) > collectorAssistMovingVelocityThreshold
                || isJamCorrecting;

        if (movingNow) {
            collectorAssistLatched = true;
            collectorNoMotionTimer.reset();
            collector.setPower(clamp(collectorAssistPower, -1.0, 1.0));
            return;
        }

        if (collectorAssistLatched
                && collectorNoMotionTimer.milliseconds() <= Math.max(0, collectorAssistKeepAliveMs)) {
            collector.setPower(clamp(collectorAssistPower, -1.0, 1.0));
            return;
        }

        collectorAssistLatched = false;
        collector.stop();
    }

    private void stopCollectorAssist() {
        if (collector != null) {
            collector.stop();
        }
        collectorAssistLatched = false;
        collectorNoMotionTimer.reset();
    }

    private boolean isCollectorAssistActive() {
        return collector != null && Math.abs(collector.getPower()) > 1e-6;
    }

    private void setStatusIdle(String message) {
        statusPrimary = message;
        statusDetail = String.format(Locale.US, "Save: %s | Validation: %s", saveStatus, overallTestScore);
        clearStatusLayout();
        statusActions = "A=tune+test, X=test only";
        statusResult = String.format(Locale.US, "Jams: %d | Samples: %d", jamRecoveryCount, totalSamples);
    }

    private String buildStatusSummary() {
        StringBuilder out = new StringBuilder(statusPrimary == null ? "-" : statusPrimary);

        if (!"-".equals(statusFront) || !"-".equals(statusRight) || !"-".equals(statusLeft)) {
            out.append(" | F:").append(statusFront)
                    .append(" R:").append(statusRight)
                    .append(" L:").append(statusLeft);
        }
        if (statusDetail != null && !statusDetail.isEmpty() && !"-".equals(statusDetail)) {
            out.append(" | ").append(statusDetail);
        }
        if (statusActions != null && !statusActions.isEmpty() && !"-".equals(statusActions)) {
            out.append(" | ").append(statusActions);
        }
        if (statusResult != null && !statusResult.isEmpty() && !"-".equals(statusResult)) {
            out.append(" | ").append(statusResult);
        }
        return out.toString();
    }

    private void setStatusError(String message, String detail) {
        statusPrimary = message;
        statusDetail = (detail == null || detail.isEmpty()) ? "-" : detail;
        clearStatusLayout();
        statusActions = "A=tune+test, X=test only";
        statusResult = "Save: " + saveStatus;
    }

    private void setStatusReadingProgress(int completedPositions, int totalPositions) {
        statusPrimary = String.format(
                Locale.US,
                "Getting readings %d/%d (%s)",
                completedPositions,
                totalPositions,
                formatElapsed(phaseTimer)
        );
        statusDetail = String.format(
                Locale.US,
                "Collector assist: %s",
                collector == null ? "n/a" : (isCollectorAssistActive() ? "ON" : "OFF")
        );
        clearStatusLayout();
        statusActions = "-";
        statusResult = String.format(Locale.US, "Jams corrected: %d", jamRecoveryCount);
    }

    private void setStatusJamCorrecting() {
        statusPrimary = String.format(
                Locale.US,
                "Detected jam, correcting... (%s)",
                formatElapsed(jamCorrectionTimer)
        );
        statusDetail = "Indexer paused, recovering";
        clearStatusLayout();
        statusActions = "-";
        statusResult = String.format(Locale.US, "Recoveries: %d", jamRecoveryCount);
    }

    private void setStatusTestingProgress(ValidationRunProgress progress) {
        int untested = Math.max(0, progress.total - progress.processed);
        statusPrimary = String.format(
                Locale.US,
                "Testing... failed: %d, succeeded: %d, untested: %d (%s)",
                progress.failed,
                progress.succeeded,
                untested,
                formatElapsed(phaseTimer)
        );
        statusDetail = String.format(
                Locale.US,
                "Overall: %s | Front: %s | Right: %s | Left: %s",
                overallTestScore,
                frontTestScore,
                rightTestScore,
                leftTestScore
        );
        clearStatusLayout();
        statusActions = "-";
        statusResult = "Save: " + saveStatus;
    }

    private void clearStatusLayout() {
        statusFront = "-";
        statusRight = "-";
        statusLeft = "-";
    }

    private static String formatElapsed(ElapsedTime timer) {
        return String.format(Locale.US, "%ds", Math.round(timer.seconds()));
    }

    private static String formatKnown(KnownContent known) {
        switch (known) {
            case OPEN:
                return "Open";
            case GREEN:
                return "Green";
            case PURPLE:
            default:
                return "Purple";
        }
    }

    private void checkForIndexerJam() {
        if (indexer.isBusy() && Math.abs(indexer.getVelocity()) <= jamVelocityThreshold && !isJamCorrecting) {
            if (jamTimer.milliseconds() > jamHoldMs) {
                jamTimer.reset();
                long lastCurrent = indexer.getCurrentIndex();
                indexer.setTargetIndex(lastCurrent);
                isJamCorrecting = true;
                jamCorrectionTimer.reset();
                status = "Jam correcting";
                setAllLeds(SmartLEDIndicator.IndicatorColor.RED);
            }
        } else if (!isJamCorrecting) {
            jamTimer.reset();
        }
    }

    private ProfileFit chooseBestLabelAssignment(SensorSamples samples, ProfileFit fit) {
        ProfileFit normal = fit.copy();
        normal.trainingScore = evaluateFitAccuracy(samples, normal);
        normal.swappedGreenPurple = false;

        ProfileFit swapped = fit.copy();
        swapped.swappedGreenPurple = true;

        double tmpHue = swapped.greenHue;
        double tmpMinSat = swapped.greenMinSaturation;
        double tmpMinVal = swapped.greenMinValue;
        swapped.greenHue = swapped.purpleHue;
        swapped.greenMinSaturation = swapped.purpleMinSaturation;
        swapped.greenMinValue = swapped.purpleMinValue;
        swapped.purpleHue = tmpHue;
        swapped.purpleMinSaturation = tmpMinSat;
        swapped.purpleMinValue = tmpMinVal;
        swapped.trainingScore = evaluateFitAccuracy(samples, swapped);

        return swapped.trainingScore > normal.trainingScore ? swapped : normal;
    }

    private double evaluateFitAccuracy(SensorSamples samples, ProfileFit fit) {
        int correct = 0;
        int total = 0;

        int openCount = Math.min(samples.open.hue.size(), Math.min(samples.open.saturation.size(), samples.open.value.size()));
        for (int i = 0; i < openCount; i++) {
            if (classifyWithFit(samples.open.hue.get(i), samples.open.saturation.get(i), samples.open.value.get(i), fit) == KnownContent.OPEN) {
                correct++;
            }
            total++;
        }

        int greenCount = Math.min(samples.green.hue.size(), Math.min(samples.green.saturation.size(), samples.green.value.size()));
        for (int i = 0; i < greenCount; i++) {
            if (classifyWithFit(samples.green.hue.get(i), samples.green.saturation.get(i), samples.green.value.get(i), fit) == KnownContent.GREEN) {
                correct++;
            }
            total++;
        }

        int purpleCount = Math.min(samples.purple.hue.size(), Math.min(samples.purple.saturation.size(), samples.purple.value.size()));
        for (int i = 0; i < purpleCount; i++) {
            if (classifyWithFit(samples.purple.hue.get(i), samples.purple.saturation.get(i), samples.purple.value.get(i), fit) == KnownContent.PURPLE) {
                correct++;
            }
            total++;
        }

        return total == 0 ? 0.0 : (double) correct / total;
    }

    private KnownContent classifyWithFit(double hue, double saturation, double value, ProfileFit fit) {
        if (saturation < fit.sensorMinSaturation || value < fit.sensorMinValue) {
            return KnownContent.OPEN;
        }

        double greenConfidence = computeConfidence(hue, saturation, value, fit.greenHue, fit.greenMinSaturation, fit.greenMinValue, fit.hueTolerance);
        double purpleConfidence = computeConfidence(hue, saturation, value, fit.purpleHue, fit.purpleMinSaturation, fit.purpleMinValue, fit.hueTolerance);

        if (greenConfidence <= 0.0 && purpleConfidence <= 0.0) {
            return KnownContent.OPEN;
        }

        return greenConfidence >= purpleConfidence ? KnownContent.GREEN : KnownContent.PURPLE;
    }

    private static double computeConfidence(
            double hue,
            double saturation,
            double value,
            double targetHue,
            double minSaturation,
            double minValue,
            double hueTolerance
    ) {
        if (saturation < minSaturation || value < minValue) {
            return 0.0;
        }

        double safeTolerance = Math.max(0.001, hueTolerance);
        double hueDiff = circularDistance(hue, targetHue);
        if (hueDiff > safeTolerance) {
            return 0.0;
        }

        double hueScore = 1.0 - (hueDiff / safeTolerance);
        double satScore = Math.min(saturation / Math.max(minSaturation, 0.001), 1.0);
        double valScore = Math.min(value / Math.max(minValue, 0.001), 1.0);
        return (hueScore * 0.7) + (satScore * 0.15) + (valScore * 0.15);
    }

    private boolean runValidationTest() {
        status = "Testing config";
        setAllLeds(SmartLEDIndicator.IndicatorColor.AZURE);
        isJamCorrecting = false;
        jamTimer.reset();
        phaseTimer.reset();

        ValidationScore score = new ValidationScore();
        ValidationRunProgress progress = new ValidationRunProgress();
        long validationBaseIndex = indexer.getCurrentIndex();
        int totalPositions = Math.max(1, testCycles) * 3;
        progress.total = totalPositions * Math.max(1, testSamplesPerPosition);
        setStatusTestingProgress(progress);

        if (!moveToIndex(validationBaseIndex, moveTimeoutMs)) {
            status = "Validation start move failed";
            setAllLeds(SmartLEDIndicator.IndicatorColor.RED);
            setStatusError("Validation failed to start", "");
            return false;
        }

        for (int i = 0; i < totalPositions && opModeIsActive(); i++) {
            long targetIndex = validationBaseIndex + i;
            if (!moveToIndex(targetIndex, moveTimeoutMs)) {
                status = "Validation move timeout";
                setAllLeds(SmartLEDIndicator.IndicatorColor.RED);
                setStatusError("Validation move timeout", "Index " + targetIndex);
                return false;
            }

            holdSteady(settleHoldMs);
            sampleValidationAtCurrentPosition(baseNormalizedIndex, score, progress);
        }

        if (!opModeIsActive()) {
            status = "Stopped";
            setAllLeds(SmartLEDIndicator.IndicatorColor.OFF);
            setStatusIdle("Stopped");
            return false;
        }

        overallTestScore = score.formatOverall();
        frontTestScore = score.formatSensor(score.frontCorrect, score.frontTotal);
        leftTestScore = score.formatSensor(score.leftCorrect, score.leftTotal);
        rightTestScore = score.formatSensor(score.rightCorrect, score.rightTotal);

        boolean passed = score.overallAccuracy() >= passingScore;
        status = "Validation complete";
        setStatusTestingProgress(progress);
        statusActions = "A=tune+test, X=test only";
        statusResult = "Validation " + (passed ? "PASSED" : "FAILED") + " - " + overallTestScore;
        setAllLeds(passed
                ? SmartLEDIndicator.IndicatorColor.GREEN
                : SmartLEDIndicator.IndicatorColor.YELLOW);
        return passed;
    }

    private void sampleValidationAtCurrentPosition(int baseNormalized, ValidationScore score, ValidationRunProgress progress) {
        for (int i = 0; i < Math.max(1, testSamplesPerPosition) && opModeIsActive(); i++) {
            updateIndexerWithJamRecovery(indexer.getTargetIndex());

            int normalized = Math.floorMod(indexer.getNormalizedCurrentIndex(), 3);
            int frontSlotRelativeToStart = Math.floorMod(normalized - baseNormalized, 3);
            KnownContent frontExpected = knownSlots[frontSlotRelativeToStart];
            KnownContent leftExpected = knownSlots[(frontSlotRelativeToStart + 1) % 3];
            KnownContent rightExpected = knownSlots[(frontSlotRelativeToStart + 2) % 3];

            ScoringElementColor frontDetected = frontSensor.getScoringElementColor();
            ScoringElementColor leftDetected = leftSensor.getScoringElementColor();
            ScoringElementColor rightDetected = rightSensor.getScoringElementColor();

            boolean frontOk = matchesExpected(frontDetected, frontExpected);
            boolean leftOk = matchesExpected(leftDetected, leftExpected);
            boolean rightOk = matchesExpected(rightDetected, rightExpected);

            score.addFront(frontOk);
            score.addLeft(leftOk);
            score.addRight(rightOk);
            progress.processed++;
            if (frontOk && leftOk && rightOk) {
                progress.succeeded++;
            } else {
                progress.failed++;
            }
            setStatusTestingProgress(progress);

            safeSetLed(frontLED, frontOk ? colorForKnownContent(frontExpected) : SmartLEDIndicator.IndicatorColor.RED);
            safeSetLed(leftLED, leftOk ? colorForKnownContent(leftExpected) : SmartLEDIndicator.IndicatorColor.RED);
            safeSetLed(rightLED, rightOk ? colorForKnownContent(rightExpected) : SmartLEDIndicator.IndicatorColor.RED);

            if (testSampleDelayMs > 0) {
                sleep(testSampleDelayMs);
            }
        }
    }

    private static boolean matchesExpected(ScoringElementColor detected, KnownContent expected) {
        if (detected == null) {
            detected = ScoringElementColor.NONE;
        }
        switch (expected) {
            case OPEN:
                return detected == ScoringElementColor.NONE;
            case GREEN:
                return detected == ScoringElementColor.GREEN;
            case PURPLE:
            default:
                return detected == ScoringElementColor.PURPLE;
        }
    }

    private ProfileFit fitProfile(SensorSamples samples, SensorRole role) {
        ProfileFit fallback = getCurrentProfile(role);
        ProfileFit fit = fallback.copy();

        List<Double> openSat = samples.open.saturation;
        List<Double> openVal = samples.open.value;
        double greenMinSat = chooseBestThreshold(samples.green.saturation, openSat, fit.greenMinSaturation);
        double greenMinVal = chooseBestThreshold(samples.green.value, openVal, fit.greenMinValue);
        double purpleMinSat = chooseBestThreshold(samples.purple.saturation, openSat, fit.purpleMinSaturation);
        double purpleMinVal = chooseBestThreshold(samples.purple.value, openVal, fit.purpleMinValue);

        // Keep the global gates permissive enough for both classes.
        fit.sensorMinSaturation = clamp01(Math.min(greenMinSat, purpleMinSat));
        fit.sensorMinValue = clamp01(Math.min(greenMinVal, purpleMinVal));

        fit.greenHue = normalizeHue(circularMean(samples.green.hue, fit.greenHue));
        fit.purpleHue = normalizeHue(circularMean(samples.purple.hue, fit.purpleHue));

        double greenSpread = circularDistancePercentile(samples.green.hue, fit.greenHue, hueSpreadPercentile);
        double purpleSpread = circularDistancePercentile(samples.purple.hue, fit.purpleHue, hueSpreadPercentile);
        double requiredTolerance = Math.max(greenSpread, purpleSpread) + hueSafetyMarginDeg;
        double separation = circularDistance(fit.greenHue, fit.purpleHue);
        double maxNonOverlapTolerance = Math.max(minHueToleranceDeg, (separation * 0.5) - hueSafetyMarginDeg);

        if (!samples.green.hue.isEmpty() && !samples.purple.hue.isEmpty()) {
            fit.hueTolerance = clamp(requiredTolerance, minHueToleranceDeg, Math.min(maxHueToleranceDeg, maxNonOverlapTolerance));
        } else {
            fit.hueTolerance = clamp(requiredTolerance, minHueToleranceDeg, maxHueToleranceDeg);
        }

        fit.greenMinSaturation = clamp01(Math.max(fit.sensorMinSaturation, greenMinSat));
        fit.greenMinValue = clamp01(Math.max(fit.sensorMinValue, greenMinVal));
        fit.purpleMinSaturation = clamp01(Math.max(fit.sensorMinSaturation, purpleMinSat));
        fit.purpleMinValue = clamp01(Math.max(fit.sensorMinValue, purpleMinVal));

        return chooseBestLabelAssignment(samples, fit);
    }

    private ProfileFit getCurrentProfile(SensorRole role) {
        ProfileFit fit = new ProfileFit();
        switch (role) {
            case FRONT: {
                fit.hueTolerance = ColorMatchConfig.FRONT_HUE_TOLERANCE;
                fit.sensorMinSaturation = ColorMatchConfig.FRONT_MIN_SATURATION;
                fit.sensorMinValue = ColorMatchConfig.FRONT_MIN_VALUE;

                fit.greenHue = ColorMatchConfig.FRONT_GREEN.targetHue;
                fit.greenMinSaturation = ColorMatchConfig.FRONT_GREEN.minSaturation;
                fit.greenMinValue = ColorMatchConfig.FRONT_GREEN.minValue;

                fit.purpleHue = ColorMatchConfig.FRONT_PURPLE.targetHue;
                fit.purpleMinSaturation = ColorMatchConfig.FRONT_PURPLE.minSaturation;
                fit.purpleMinValue = ColorMatchConfig.FRONT_PURPLE.minValue;
                break;
            }
            case LEFT: {
                fit.hueTolerance = ColorMatchConfig.FRONT_HUE_TOLERANCE;
                fit.sensorMinSaturation = ColorMatchConfig.FRONT_MIN_SATURATION;
                fit.sensorMinValue = ColorMatchConfig.FRONT_MIN_VALUE;

                fit.greenHue = ColorMatchConfig.FRONT_GREEN.targetHue;
                fit.greenMinSaturation = ColorMatchConfig.FRONT_GREEN.minSaturation;
                fit.greenMinValue = ColorMatchConfig.FRONT_GREEN.minValue;

                fit.purpleHue = ColorMatchConfig.FRONT_PURPLE.targetHue;
                fit.purpleMinSaturation = ColorMatchConfig.FRONT_PURPLE.minSaturation;
                fit.purpleMinValue = ColorMatchConfig.FRONT_PURPLE.minValue;
                break;
            }
            case RIGHT: {
                fit.hueTolerance = ColorMatchConfig.FRONT_HUE_TOLERANCE;
                fit.sensorMinSaturation = ColorMatchConfig.FRONT_MIN_SATURATION;
                fit.sensorMinValue = ColorMatchConfig.FRONT_MIN_VALUE;

                fit.greenHue = ColorMatchConfig.FRONT_GREEN.targetHue;
                fit.greenMinSaturation = ColorMatchConfig.FRONT_GREEN.minSaturation;
                fit.greenMinValue = ColorMatchConfig.FRONT_GREEN.minValue;

                fit.purpleHue = ColorMatchConfig.FRONT_PURPLE.targetHue;
                fit.purpleMinSaturation = ColorMatchConfig.FRONT_PURPLE.minSaturation;
                fit.purpleMinValue = ColorMatchConfig.FRONT_PURPLE.minValue;
                break;
            }
        }
        return fit;
    }

    private void applyFit(ProfileFit fit, SensorRole role) {
        switch (role) {
            case FRONT: {
                ColorMatchConfig.FRONT_HUE_TOLERANCE = (float) fit.hueTolerance;
                ColorMatchConfig.FRONT_MIN_SATURATION = (float) fit.sensorMinSaturation;
                ColorMatchConfig.FRONT_MIN_VALUE = (float) fit.sensorMinValue;

                ColorMatchConfig.FRONT_GREEN.targetHue = (float) fit.greenHue;
                ColorMatchConfig.FRONT_GREEN.minSaturation = (float) fit.greenMinSaturation;
                ColorMatchConfig.FRONT_GREEN.minValue = (float) fit.greenMinValue;

                ColorMatchConfig.FRONT_PURPLE.targetHue = (float) fit.purpleHue;
                ColorMatchConfig.FRONT_PURPLE.minSaturation = (float) fit.purpleMinSaturation;
                ColorMatchConfig.FRONT_PURPLE.minValue = (float) fit.purpleMinValue;
                ColorMatchConfig.FRONT_ACTIVE_PRESETS = new ColorMatchConfig.ColorPreset[]{
                        ColorMatchConfig.FRONT_PURPLE,
                        ColorMatchConfig.FRONT_GREEN
                };
                break;
            }
            case LEFT: {
                ColorMatchConfig.FRONT_HUE_TOLERANCE = (float) fit.hueTolerance;
                ColorMatchConfig.FRONT_MIN_SATURATION = (float) fit.sensorMinSaturation;
                ColorMatchConfig.FRONT_MIN_VALUE = (float) fit.sensorMinValue;

                ColorMatchConfig.FRONT_GREEN.targetHue = (float) fit.greenHue;
                ColorMatchConfig.FRONT_GREEN.minSaturation = (float) fit.greenMinSaturation;
                ColorMatchConfig.FRONT_GREEN.minValue = (float) fit.greenMinValue;

                ColorMatchConfig.FRONT_PURPLE.targetHue = (float) fit.purpleHue;
                ColorMatchConfig.FRONT_PURPLE.minSaturation = (float) fit.purpleMinSaturation;
                ColorMatchConfig.FRONT_PURPLE.minValue = (float) fit.purpleMinValue;
                ColorMatchConfig.FRONT_ACTIVE_PRESETS = new ColorMatchConfig.ColorPreset[]{
                        ColorMatchConfig.FRONT_PURPLE,
                        ColorMatchConfig.FRONT_GREEN
                };
                break;
            }
            case RIGHT: {
                ColorMatchConfig.FRONT_HUE_TOLERANCE = (float) fit.hueTolerance;
                ColorMatchConfig.FRONT_MIN_SATURATION = (float) fit.sensorMinSaturation;
                ColorMatchConfig.FRONT_MIN_VALUE = (float) fit.sensorMinValue;

                ColorMatchConfig.FRONT_GREEN.targetHue = (float) fit.greenHue;
                ColorMatchConfig.FRONT_GREEN.minSaturation = (float) fit.greenMinSaturation;
                ColorMatchConfig.FRONT_GREEN.minValue = (float) fit.greenMinValue;

                ColorMatchConfig.FRONT_PURPLE.targetHue = (float) fit.purpleHue;
                ColorMatchConfig.FRONT_PURPLE.minSaturation = (float) fit.purpleMinSaturation;
                ColorMatchConfig.FRONT_PURPLE.minValue = (float) fit.purpleMinValue;
                ColorMatchConfig.FRONT_ACTIVE_PRESETS = new ColorMatchConfig.ColorPreset[]{
                        ColorMatchConfig.FRONT_PURPLE,
                        ColorMatchConfig.FRONT_GREEN
                };
                break;
            }
        }
    }

    private boolean loadKnownSlots() {
        try {
            knownSlots = new KnownContent[]{
                    KnownContent.fromCode(slot0Content),
                    KnownContent.fromCode(slot1Content),
                    KnownContent.fromCode(slot2Content)
            };
        } catch (IllegalArgumentException e) {
            status = "Invalid known slot content";
            errors = e.getMessage();
            return false;
        }

        int openCount = 0;
        int greenCount = 0;
        int purpleCount = 0;
        for (KnownContent content : knownSlots) {
            switch (content) {
                case OPEN:
                    openCount++;
                    break;
                case GREEN:
                    greenCount++;
                    break;
                case PURPLE:
                    purpleCount++;
                    break;
            }
        }

        if (openCount != 1 || greenCount != 1 || purpleCount != 1) {
            status = "Known state must contain 1 OPEN, 1 GREEN, 1 PURPLE";
            errors = "slot0/slot1/slot2 must be a permutation of 0/1/2";
            return false;
        }

        return true;
    }

    private SmartLEDIndicator tryGetLed(String name) {
        try {
            return Hardware.getLEDIndicator(name);
        } catch (Exception e) {
            prettyTelem.warning("LED '" + name + "' unavailable: " + e.getMessage());
            return null;
        }
    }

    private Collector tryGetCollector(String motorName) {
        try {
            return new Collector(Hardware.getMotor(motorName));
        } catch (Exception e) {
            prettyTelem.warning("Collector '" + motorName + "' unavailable: " + e.getMessage());
            return null;
        }
    }

    private boolean promptForCollectorVerification() {
        status = "Prompting layout verify";
        statusPrimary = "Is the indexer in this state?";
        statusDetail = "-";
        statusFront = formatKnown(knownSlots[0]);
        statusRight = formatKnown(knownSlots[2]);
        statusLeft = formatKnown(knownSlots[1]);
        statusActions = "(Yes: A, No: B)";
        statusResult = "Save: " + saveStatus;
        return waitForPromptChoiceAB();
    }

    private boolean promptForSaveConfirmation() {
        status = "Prompting save";
        statusPrimary = "Save tuned results?";
        statusDetail = "Validation passed";
        clearStatusLayout();
        statusActions = "(Yes: A, No: B)";
        return waitForPromptChoiceAB();
    }

    private PostTuneChoice promptForPostTuneChoice() {
        status = "Prompting post-tune action";
        statusPrimary = "Would you like to test the config?";
        statusDetail = String.format(
                Locale.US,
                "Readings complete (%d samples, %s)",
                totalSamples,
                formatElapsed(phaseTimer)
        );
        clearStatusLayout();
        statusActions = "(Yes: A, Save now: X, Cancel: B)";
        return waitForPromptChoiceAXB();
    }

    private boolean waitForPromptButtonsReleased(boolean includeX) {
        while (opModeIsActive()) {
            if (!gamepad1.a && !gamepad1.b && (!includeX || !gamepad1.x)) {
                return true;
            }
            frameworkTick();
            sleep(20);
        }
        return false;
    }

    private boolean waitForPromptChoiceAB() {
        if (!waitForPromptButtonsReleased(false)) {
            return false;
        }

        boolean previousA = false;
        boolean previousB = false;

        while (opModeIsActive()) {
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;

            if (currentA && !previousA) {
                return true;
            }
            if (currentB && !previousB) {
                return false;
            }

            previousA = currentA;
            previousB = currentB;
            frameworkTick();
            sleep(20);
        }
        return false;
    }

    private PostTuneChoice waitForPromptChoiceAXB() {
        if (!waitForPromptButtonsReleased(true)) {
            return PostTuneChoice.CANCEL;
        }

        boolean previousA = false;
        boolean previousB = false;
        boolean previousX = false;

        while (opModeIsActive()) {
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;

            if (currentA && !previousA) {
                return PostTuneChoice.TEST;
            }
            if (currentX && !previousX) {
                return PostTuneChoice.SAVE_NOW;
            }
            if (currentB && !previousB) {
                return PostTuneChoice.CANCEL;
            }

            previousA = currentA;
            previousB = currentB;
            previousX = currentX;
            frameworkTick();
            sleep(20);
        }
        return PostTuneChoice.CANCEL;
    }

    private void setAllLeds(SmartLEDIndicator.IndicatorColor color) {
        safeSetLed(frontLED, color);
        safeSetLed(leftLED, color);
        safeSetLed(rightLED, color);
    }

    private void setExpectedContentLeds(KnownContent front, KnownContent left, KnownContent right) {
        safeSetLed(frontLED, colorForKnownContent(front));
        safeSetLed(leftLED, colorForKnownContent(left));
        safeSetLed(rightLED, colorForKnownContent(right));
    }

    private static SmartLEDIndicator.IndicatorColor colorForKnownContent(KnownContent content) {
        switch (content) {
            case OPEN:
                return SmartLEDIndicator.IndicatorColor.WHITE;
            case GREEN:
                return SmartLEDIndicator.IndicatorColor.GREEN;
            case PURPLE:
            default:
                return SmartLEDIndicator.IndicatorColor.VIOLET;
        }
    }

    private static void safeSetLed(SmartLEDIndicator led, SmartLEDIndicator.IndicatorColor color) {
        if (led == null || color == null) {
            return;
        }
        try {
            led.setColor(color);
        } catch (Exception ignored) {
            // Telemetry already reports missing/broken LEDs; ignore set failures in the hot loop.
        }
    }

    private String knownSlotSummary() {
        return String.format(
                Locale.US,
                "[0:%s, 1:%s, 2:%s]",
                knownSlots[0],
                knownSlots[1],
                knownSlots[2]
        );
    }

    private static void addSample(Distribution distribution, float[] hsv) {
        distribution.hue.add((double) hsv[0]);
        distribution.saturation.add((double) hsv[1]);
        distribution.value.add((double) hsv[2]);
    }

    private static List<Double> concat(List<Double> first, List<Double> second) {
        ArrayList<Double> out = new ArrayList<>(first.size() + second.size());
        out.addAll(first);
        out.addAll(second);
        return out;
    }

    private static double chooseBestThreshold(List<Double> positive, List<Double> negative, double fallback) {
        if (positive.isEmpty() || negative.isEmpty()) {
            return fallback;
        }

        ArrayList<Double> all = new ArrayList<>(positive.size() + negative.size());
        all.addAll(positive);
        all.addAll(negative);
        Collections.sort(all);

        ArrayList<Double> candidates = new ArrayList<>();
        candidates.add(0.0);
        if (!all.isEmpty()) {
            candidates.add(all.get(0));
        }
        for (int i = 0; i < all.size() - 1; i++) {
            double a = all.get(i);
            double b = all.get(i + 1);
            candidates.add(a);
            candidates.add((a + b) * 0.5);
        }
        if (!all.isEmpty()) {
            candidates.add(all.get(all.size() - 1));
        }
        candidates.add(1.0);

        double bestThreshold = fallback;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (double threshold : candidates) {
            int tp = 0;
            int fn = 0;
            int tn = 0;
            int fp = 0;

            for (double value : positive) {
                if (value >= threshold) {
                    tp++;
                } else {
                    fn++;
                }
            }
            for (double value : negative) {
                if (value < threshold) {
                    tn++;
                } else {
                    fp++;
                }
            }

            double tpr = positive.isEmpty() ? 1.0 : (double) tp / positive.size();
            double tnr = negative.isEmpty() ? 1.0 : (double) tn / negative.size();
            double score = tpr + tnr;

            if (score > bestScore) {
                bestScore = score;
                bestThreshold = threshold;
            }
        }

        return bestThreshold;
    }

    private static double circularMean(List<Double> hueValues, double fallback) {
        if (hueValues.isEmpty()) {
            return fallback;
        }

        double sumSin = 0.0;
        double sumCos = 0.0;
        for (double hue : hueValues) {
            double radians = Math.toRadians(hue);
            sumSin += Math.sin(radians);
            sumCos += Math.cos(radians);
        }

        if (Math.abs(sumSin) < 1e-9 && Math.abs(sumCos) < 1e-9) {
            return fallback;
        }

        double angle = Math.toDegrees(Math.atan2(sumSin, sumCos));
        return normalizeHue(angle);
    }

    private static double circularDistancePercentile(List<Double> hueValues, double centerHue, double quantile) {
        if (hueValues.isEmpty()) {
            return 0.0;
        }

        ArrayList<Double> distances = new ArrayList<>(hueValues.size());
        for (double hue : hueValues) {
            distances.add(circularDistance(hue, centerHue));
        }
        return percentile(distances, quantile);
    }

    private static double percentile(List<Double> values, double percentile) {
        if (values.isEmpty()) {
            return 0.0;
        }
        ArrayList<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);

        double clampedPercentile = clamp(percentile, 0.0, 1.0);
        double index = clampedPercentile * (sorted.size() - 1);
        int low = (int) Math.floor(index);
        int high = (int) Math.ceil(index);
        if (low == high) {
            return sorted.get(low);
        }
        double fraction = index - low;
        return sorted.get(low) + (sorted.get(high) - sorted.get(low)) * fraction;
    }

    private static double circularDistance(double firstHue, double secondHue) {
        double diff = Math.abs(firstHue - secondHue) % 360.0;
        return diff > 180.0 ? 360.0 - diff : diff;
    }

    private static double normalizeHue(double hue) {
        double normalized = hue % 360.0;
        if (normalized < 0.0) {
            normalized += 360.0;
        }
        return normalized;
    }

    private static double clamp01(double value) {
        return clamp(value, 0.0, 1.0);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static String formatFit(ProfileFit fit) {
        return String.format(
                Locale.US,
                "%.1f%% swap=%s tol=%.1f",
                fit.trainingScore * 100.0,
                fit.swappedGreenPurple,
                fit.hueTolerance
        );
    }

    private enum SensorRole {
        FRONT,
        LEFT,
        RIGHT
    }

    private enum PostTuneChoice {
        TEST,
        SAVE_NOW,
        CANCEL
    }

    private enum KnownContent {
        OPEN(0),
        GREEN(1),
        PURPLE(2);

        private final int code;

        KnownContent(int code) {
            this.code = code;
        }

        static KnownContent fromCode(int code) {
            for (KnownContent value : values()) {
                if (value.code == code) {
                    return value;
                }
            }
            throw new IllegalArgumentException("Unsupported slot content code: " + code + ". Expected 0, 1, or 2.");
        }
    }

    private static class Distribution {
        final List<Double> hue = new ArrayList<>();
        final List<Double> saturation = new ArrayList<>();
        final List<Double> value = new ArrayList<>();

        void clear() {
            hue.clear();
            saturation.clear();
            value.clear();
        }
    }

    private static class SensorSamples {
        final Distribution open = new Distribution();
        final Distribution green = new Distribution();
        final Distribution purple = new Distribution();

        Distribution get(KnownContent content) {
            switch (content) {
                case OPEN:
                    return open;
                case GREEN:
                    return green;
                case PURPLE:
                default:
                    return purple;
            }
        }

        void clear() {
            open.clear();
            green.clear();
            purple.clear();
        }
    }

    private static class ProfileFit {
        double hueTolerance;
        double sensorMinSaturation;
        double sensorMinValue;
        double trainingScore;
        boolean swappedGreenPurple;

        double greenHue;
        double greenMinSaturation;
        double greenMinValue;

        double purpleHue;
        double purpleMinSaturation;
        double purpleMinValue;

        ProfileFit copy() {
            ProfileFit copy = new ProfileFit();
            copy.hueTolerance = hueTolerance;
            copy.sensorMinSaturation = sensorMinSaturation;
            copy.sensorMinValue = sensorMinValue;
            copy.trainingScore = trainingScore;
            copy.swappedGreenPurple = swappedGreenPurple;
            copy.greenHue = greenHue;
            copy.greenMinSaturation = greenMinSaturation;
            copy.greenMinValue = greenMinValue;
            copy.purpleHue = purpleHue;
            copy.purpleMinSaturation = purpleMinSaturation;
            copy.purpleMinValue = purpleMinValue;
            return copy;
        }
    }

    private static class ValidationScore {
        int frontCorrect;
        int frontTotal;
        int leftCorrect;
        int leftTotal;
        int rightCorrect;
        int rightTotal;

        void addFront(boolean correct) {
            frontTotal++;
            if (correct) {
                frontCorrect++;
            }
        }

        void addLeft(boolean correct) {
            leftTotal++;
            if (correct) {
                leftCorrect++;
            }
        }

        void addRight(boolean correct) {
            rightTotal++;
            if (correct) {
                rightCorrect++;
            }
        }

        double overallAccuracy() {
            int total = frontTotal + leftTotal + rightTotal;
            int correct = frontCorrect + leftCorrect + rightCorrect;
            return total == 0 ? 0.0 : (double) correct / total;
        }

        String formatOverall() {
            int total = frontTotal + leftTotal + rightTotal;
            int correct = frontCorrect + leftCorrect + rightCorrect;
            return String.format(Locale.US, "%.1f%% (%d/%d)", overallAccuracy() * 100.0, correct, total);
        }

        String formatSensor(int correct, int total) {
            double accuracy = total == 0 ? 0.0 : (double) correct / total;
            return String.format(Locale.US, "%.1f%% (%d/%d)", accuracy * 100.0, correct, total);
        }
    }

    private static class ValidationRunProgress {
        int total;
        int processed;
        int succeeded;
        int failed;
    }
}

