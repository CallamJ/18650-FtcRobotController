package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartLimelight3A;

import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.stream.Collectors;

@Configurable
@TeleOp(name = "4 - Limelight Diagnostics")
public class LimelightDiagnosticsTeleOp extends TeleOpCore {
    public static String limelightName = "limelight";
    public static int pipeline = 0;
    public static boolean startOnInit = true;
    public static int maxFiducialDetailRows = 5;

    private SmartLimelight3A limelight;
    private boolean limelightReady = false;
    private boolean limelightRunning = false;

    private double fps = 0;
    private boolean hasRawResult = false;
    private boolean rawResultValid = false;
    private int rawDetectionCount = 0;
    private int knownDetectionCount = 0;
    private String fiducialIds = "[]";
    private String unknownFiducialIds = "[]";
    private String resultSummary = "none";
    private String firstDepot = "none";
    private String firstObelisk = "none";
    private String detections = "none";
    private String rawFiducialDetails = "none";
    private String lastError = "none";

    @Override
    protected void initialize() {
        super.initialize();

        try {
            limelight = Hardware.getLimelight(limelightName);
            limelight.setPipeline(pipeline);
            if (startOnInit) {
                limelight.start();
                limelightRunning = true;
            }
            limelightReady = true;
            prettyTelem.info("Limelight diagnostics ready.");
        } catch (Exception e) {
            limelightReady = false;
            limelightRunning = false;
            lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
            prettyTelem.error("Limelight init failed: " + lastError);
        }

        prettyTelem.addLine("Limelight Diagnostics")
                .addData("Ready", () -> limelightReady)
                .addData("Running", () -> limelightRunning)
                .addData("Pipeline", () -> pipeline)
                .addData("FPS", () -> fps)
                .addData("Raw Result Present", () -> hasRawResult)
                .addData("Raw Result Valid", () -> rawResultValid)
                .addData("Raw Detections", () -> rawDetectionCount)
                .addData("Known Detections", () -> knownDetectionCount)
                .addData("Raw Fiducial IDs", () -> fiducialIds)
                .addData("Unknown IDs", () -> unknownFiducialIds)
                .addData("Result Summary", () -> resultSummary)
                .addData("First Depot", () -> firstDepot)
                .addData("First Obelisk", () -> firstObelisk)
                .addData("Known Detection Details", () -> detections)
                .addData("Raw Fiducial Details", () -> rawFiducialDetails)
                .addData("Last Error", () -> lastError);

        prettyTelem.addLine("Controls")
                .addData("A", () -> "Start/Stop Limelight")
                .addData("Dpad Up", () -> "Pipeline +1")
                .addData("Dpad Down", () -> "Pipeline -1");
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        if (!limelightReady || limelight == null) {
            return;
        }

        if (gamepad1.aPressed()) {
            try {
                if (limelightRunning) {
                    limelight.stop();
                    limelightRunning = false;
                    prettyTelem.info("Limelight stopped.");
                } else {
                    limelight.start();
                    limelightRunning = true;
                    prettyTelem.info("Limelight started.");
                }
            } catch (Exception e) {
                lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
                prettyTelem.error("Failed to toggle Limelight: " + lastError);
            }
        }

        if (gamepad1.dpadUpPressed()) {
            setPipeline(pipeline + 1);
        }

        if (gamepad1.dpadDownPressed()) {
            setPipeline(Math.max(0, pipeline - 1));
        }
    }

    @Override
    public void tick() {
        if (limelightReady && limelight != null && limelightRunning) {
            updateDetectionSnapshot();
        }

        super.tick();
    }

    private void setPipeline(int newPipeline) {
        if (!limelightReady || limelight == null) {
            return;
        }

        try {
            limelight.setPipeline(newPipeline);
            pipeline = newPipeline;
            prettyTelem.info("Pipeline set to " + pipeline);
        } catch (Exception e) {
            lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
            prettyTelem.error("Failed to set pipeline: " + lastError);
        }
    }

    private void updateDetectionSnapshot() {
        try {
            limelight.updateCache();

            fps = limelight.getFps();
            hasRawResult = limelight.hasResult();
            rawResultValid = limelight.isResultValid();
            resultSummary = limelight.getResultSummary();

            List<Integer> ids = limelight.getRawFiducialIds();
            fiducialIds = ids.toString();
            rawDetectionCount = ids.size();

            List<SmartLimelight3A.AprilTag> tags = limelight.getAprilTags();
            knownDetectionCount = tags.size();
            Set<Integer> knownIds = tags.stream()
                    .map(SmartLimelight3A.AprilTag::fiducialId)
                    .collect(Collectors.toSet());
            unknownFiducialIds = ids.stream()
                    .filter(id -> !knownIds.contains(id))
                    .collect(Collectors.toList())
                    .toString();

            SmartLimelight3A.AprilTag depot = tags.stream()
                    .filter(tag -> !tag.isObelisk())
                    .findFirst()
                    .orElse(null);
            SmartLimelight3A.AprilTag obelisk = tags.stream()
                    .filter(SmartLimelight3A.AprilTag::isObelisk)
                    .findFirst()
                    .orElse(null);

            firstDepot = formatTag(depot);
            firstObelisk = formatTag(obelisk);
            detections = tags.isEmpty()
                    ? "none"
                    : tags.stream().map(this::formatTag).collect(Collectors.joining(" | "));
            List<String> detailRows = limelight.getRawFiducialDetails(maxFiducialDetailRows);
            rawFiducialDetails = detailRows.isEmpty() ? "none" : String.join(" || ", detailRows);
            lastError = "none";
        } catch (Exception e) {
            lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
            prettyTelem.error("Failed to read Limelight snapshot: " + lastError);
        }
    }

    private String formatTag(SmartLimelight3A.AprilTag tag) {
        if (tag == null) {
            return "none";
        }

        try {
            return String.format(
                    Locale.US,
                    "%s(id=%d,b=%.1f,d=%.2fm)",
                    tag.type(),
                    tag.fiducialId(),
                    tag.bearingDegToTag(),
                    tag.distanceXYToTagMeters()
            );
        } catch (IllegalStateException e) {
            return String.format(Locale.US, "%s(id=%d,pose=n/a)", tag.type(), tag.fiducialId());
        }
    }
}
