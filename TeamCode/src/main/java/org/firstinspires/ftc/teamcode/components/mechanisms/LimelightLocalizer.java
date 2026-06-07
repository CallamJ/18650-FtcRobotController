package org.firstinspires.ftc.teamcode.components.mechanisms;

import org.firstinspires.ftc.teamcode.hardware.SmartLimelight3A;
import org.firstinspires.ftc.teamcode.utilities.LiveMatchTuning;
import org.firstinspires.ftc.teamcode.utilities.Pose;

public class LimelightLocalizer {
    private final SmartLimelight3A limelight;
    private Pose lastPose = new Pose(0, 0, 0);
    private Pose lastRelPose = new Pose(0, 0, 0);
    private String lastSolveStatus = "No solve yet";

    private static final double INCHES_PER_METER = 39.37007874015748;
    private static final double MIN_TAG_RANGE_INCHES = 1.0;

    public LimelightLocalizer(SmartLimelight3A limelight) {
        this.limelight = limelight;
    }

    public boolean hasDetection() {
        return limelight.getAprilTags().stream().anyMatch(tag -> !tag.isObelisk() && hasUsableRelativeTagPose(tag));
    }

    /**
     * Computes and returns the latest pose of the launcher in the field frame.
     * If no valid AprilTag detections are available, the last known pose is returned.
     *
     * @return the latest computed pose of the launcher in field coordinates.
     */
    public Pose getLatestPose() {
        SmartLimelight3A.AprilTag tag = limelight.getAprilTags()
                .stream()
                .filter(tag1 -> !tag1.isObelisk() && hasUsableRelativeTagPose(tag1))
                .findFirst()
                .orElse(null);

        if (tag == null) {
            lastSolveStatus = "No depot tag with usable pose";
            return lastPose;
        }

        Pose tagRelPose = Pose.from(tag.tagInCameraPose());

        tagRelPose = new Pose(metersToInches(-tagRelPose.x()), metersToInches(tagRelPose.z()), Math.toRadians(tagRelPose.heading() - 180));
        if (tagRelPose.getPosition() == null) {
            lastSolveStatus = "Depot tag pose missing";
            return lastPose;
        }

        lastRelPose = tagRelPose;

        double tagFieldX = LiveMatchTuning.blueDepotX;
        double tagFieldY = LiveMatchTuning.blueDepotY;
        double tagFieldHeading = LiveMatchTuning.blueDepotHeadingDeg;

        if (tag.type() == SmartLimelight3A.AprilTag.Type.RED_DEPOT) {
            tagFieldX = LiveMatchTuning.redDepotX;
            tagFieldY = LiveMatchTuning.redDepotY;
            tagFieldHeading = LiveMatchTuning.redDepotHeadingDeg;
        }

        Pose tagFieldPose = new Pose(tagFieldX, tagFieldY, tagFieldHeading);

        lastPose = solveObserverPose(tagRelPose, tagFieldPose);

        return lastPose;
    }

    public Pose getCameraPose() {
        return new Pose(
                LiveMatchTuning.limelightLocalizerCameraX,
                LiveMatchTuning.limelightLocalizerCameraY,
                LiveMatchTuning.limelightLocalizerCameraZ,
                LiveMatchTuning.limelightLocalizerCameraHeadingDeg,
                0,
                0
        );
    }

    public String getLastSolveStatus() {
        return lastSolveStatus;
    }

    public Pose getLastTagRelPose() {
        return lastRelPose;
    }

    private static boolean hasUsableRelativeTagPose(SmartLimelight3A.AprilTag tag) {
        if (tag == null || tag.tagInCameraPose() == null || tag.tagInCameraPose().getPosition() == null) {
            return false;
        }
        try {
            double forwardInches = metersToInches(tag.getTagZInTurretFrameMeters());
            // double rightInches = metersToInches(tag.getTagXInTurretFrameMeters());
            double rightInches = 0;
            return Double.isFinite(forwardInches)
                    && Double.isFinite(rightInches)
                    && Math.hypot(forwardInches, rightInches) >= MIN_TAG_RANGE_INCHES;
        } catch (IllegalStateException ignored) {
            return false;
        }
    }

    private static double metersToInches(double meters) {
        return meters * INCHES_PER_METER;
    }

    private static double rotateX(double x, double y, double headingRad) {
        return (Math.cos(headingRad) * x) - (Math.sin(headingRad) * y);
    }

    private static double rotateY(double x, double y, double headingRad) {
        return (Math.sin(headingRad) * x) + (Math.cos(headingRad) * y);
    }

    public static Pose solveObserverPose(Pose relativePointPose, Pose absolutePointPose) {
        double relX = relativePointPose.x();
        double relY = relativePointPose.y();
        double relHeadingDeg = relativePointPose.heading();

        double absX = absolutePointPose.x();
        double absY = absolutePointPose.y();
        double absHeadingDeg = absolutePointPose.heading();

        // Observer heading (degrees)
        double observerHeadingDeg = normalizeDegrees(absHeadingDeg - relHeadingDeg);

        // Convert to radians for trig
        double theta = Math.toRadians(observerHeadingDeg);
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        // Rotate relative translation into world frame
        double relWorldX = relX * cos - relY * sin;
        double relWorldY = relX * sin + relY * cos;

        // Solve observer position
        double observerX = absX - relWorldX;
        double observerY = absY - relWorldY;

        return new Pose(observerX, observerY, observerHeadingDeg);
    }

    private static double normalizeDegrees(double angle) {
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }
}
