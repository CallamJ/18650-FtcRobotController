package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.stream.Collectors;

@Configurable

public class SmartLimelight3A extends Device implements Caching, WrappedDevice<Limelight3A> {
    private static final Logger log = LoggerFactory.getLogger(SmartLimelight3A.class);
    private static final long WARN_THROTTLE_MS = 1000;
    private final Limelight3A limelight;

    private final HardwareCache<LLResult> resultCache;
    private final HardwareCache<LLStatus> statusCache;
    private long lastResultWarnMs = 0;
    private long lastStatusWarnMs = 0;
    public static double cameraOnTurretRightOffsetMeters = 0.15;
    public static double cameraOnTurretForwardOffsetMeters = 0;

    public SmartLimelight3A(String configName, Limelight3A limelight) {
        super(configName);
        this.limelight = limelight;
        this.resultCache = new HardwareCache<>(this::safeGetLatestResult);
        this.statusCache = new HardwareCache<>(this::safeGetStatus);
    }

    @Override
    public Limelight3A getRaw() {
        return limelight;
    }

    public void start(){
        limelight.start();
    }

    public void stop(){
        limelight.stop();
    }

    public double getFps(){
        LLStatus status = statusCache.read();
        return status != null ? status.getFps() : 0;
    }

    public void setPipeline(int index){
        limelight.pipelineSwitch(index);
    }

    private LLResult safeGetLatestResult() {
        try {
            return limelight.getLatestResult();
        } catch (Exception e) {
            long now = System.currentTimeMillis();
            if (now - lastResultWarnMs >= WARN_THROTTLE_MS) {
                lastResultWarnMs = now;
                log.warn("Limelight latest result read failed", e);
            }
            return null;
        }
    }

    private LLStatus safeGetStatus() {
        try {
            return limelight.getStatus();
        } catch (Exception e) {
            long now = System.currentTimeMillis();
            if (now - lastStatusWarnMs >= WARN_THROTTLE_MS) {
                lastStatusWarnMs = now;
                log.warn("Limelight status read failed", e);
            }
            return null;
        }
    }

    public List<Integer> getFiducialIds() {
        LLResult result = getValidResult();
        if (result == null || result.getFiducialResults() == null) {
            return List.of();
        }

        return result.getFiducialResults()
                .stream()
                .map(LLResultTypes.FiducialResult::getFiducialId)
                .collect(Collectors.toList());
    }

    public List<Integer> getRawFiducialIds() {
        LLResult result = getAnyResult();
        if (result == null || result.getFiducialResults() == null) {
            return List.of();
        }

        return result.getFiducialResults()
                .stream()
                .map(LLResultTypes.FiducialResult::getFiducialId)
                .collect(Collectors.toList());
    }

    public boolean hasResult() {
        return getAnyResult() != null;
    }

    public boolean isResultValid() {
        LLResult result = getAnyResult();
        return result != null && result.isValid();
    }

    public String getResultSummary() {
        LLResult result = getAnyResult();
        if (result == null) {
            return "none";
        }

        int fidCount = result.getFiducialResults() == null ? 0 : result.getFiducialResults().size();
        Object pipeline = getFirstNonNullGetter(result, "getPipelineIndex", "getPipelineId");
        Object latencyPipeline = getFirstNonNullGetter(result, "getPipelineLatency", "getLatencyPipeline");
        Object latencyCapture = getFirstNonNullGetter(result, "getCaptureLatency", "getLatencyCapture");
        Object latencyParse = getFirstNonNullGetter(result, "getParseLatency", "getLatencyParse");
        Object botPose = getFirstNonNullGetter(result, "getBotpose", "getBotPose");

        return "valid=" + result.isValid() +
                ", fidCount=" + fidCount +
                ", pipeline=" + formatValue(pipeline) +
                ", latencyPipelineMs=" + formatValue(latencyPipeline) +
                ", latencyCaptureMs=" + formatValue(latencyCapture) +
                ", latencyParseMs=" + formatValue(latencyParse) +
                ", botPose=" + formatValue(botPose);
    }

    public List<String> getRawFiducialDetails(int maxCount) {
        LLResult result = getAnyResult();
        if (result == null || result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return List.of();
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        int effectiveMax = maxCount <= 0 ? Integer.MAX_VALUE : maxCount;
        int limit = Math.min(fiducials.size(), effectiveMax);
        List<String> details = new ArrayList<>();

        for (int i = 0; i < limit; i++) {
            LLResultTypes.FiducialResult fid = fiducials.get(i);
            int id = fid.getFiducialId();
            AprilTag.Type mappedType = AprilTag.Type.fromFiducialId(id);
            String mappedTypeLabel = mappedType == null ? "UNKNOWN" : mappedType.name();

            Object tx = getFirstNonNullGetter(fid, "getTx", "getTargetXDegrees");
            Object ty = getFirstNonNullGetter(fid, "getTy", "getTargetYDegrees");
            Object ta = getFirstNonNullGetter(fid, "getTa", "getTargetArea");
            Object skew = getFirstNonNullGetter(fid, "getTs", "getSkew", "getTargetSkew");
            Object ambiguity = getFirstNonNullGetter(fid, "getAmbiguity", "getPoseAmbiguity");
            Object decisionMargin = getFirstNonNullGetter(fid, "getDecisionMargin");
            Object distanceCamera = getFirstNonNullGetter(fid, "getDistToCamera", "getDistanceToCamera");
            Object distanceRobot = getFirstNonNullGetter(fid, "getDistToRobot", "getDistanceToRobot");
            Pose3D cameraPose = fid.getTargetPoseCameraSpace();
            Pose3D robotPose = asPose3D(getFirstNonNullGetter(fid, "getTargetPoseRobotSpace"));
            Pose3D fieldPose = asPose3D(getFirstNonNullGetter(fid, "getTargetPoseFieldSpace"));

            details.add(
                    "#" + i +
                            " id=" + id +
                            " mapped=" + mappedTypeLabel +
                            " tx=" + formatValue(tx) +
                            " ty=" + formatValue(ty) +
                            " ta=" + formatValue(ta) +
                            " skew=" + formatValue(skew) +
                            " ambiguity=" + formatValue(ambiguity) +
                            " decisionMargin=" + formatValue(decisionMargin) +
                            " distCam=" + formatValue(distanceCamera) +
                            " distRobot=" + formatValue(distanceRobot) +
                            " camPose=" + formatPose(cameraPose) +
                            " robotPose=" + formatPose(robotPose) +
                            " fieldPose=" + formatPose(fieldPose)
            );
        }

        if (fiducials.size() > limit) {
            details.add("... +" + (fiducials.size() - limit) + " more fiducials");
        }

        return details;
    }

    public List<String> getRawFiducialDetails() {
        return getRawFiducialDetails(Integer.MAX_VALUE);
    }

    public List<AprilTag> getAprilTags() {
        LLResult result = getValidResult();
        if (result == null || result.getFiducialResults() == null) {
            return List.of();
        }

        return result.getFiducialResults()
                .stream()
                .map(fid -> AprilTag.fromDetection(
                        fid.getFiducialId(),
                        fid.getTargetPoseCameraSpace(),
                        System.nanoTime()
                ))
                .filter(Objects::nonNull)
                .collect(Collectors.toList());
    }

    private LLResult getValidResult() {
        LLResult result = getAnyResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        return result;
    }

    private LLResult getAnyResult() {
        return resultCache.read();
    }

    private static Pose3D asPose3D(Object value) {
        if (value instanceof Pose3D) {
            return (Pose3D) value;
        }
        return null;
    }

    @Nullable
    private static Object getFirstNonNullGetter(Object target, String... methodNames) {
        for (String methodName : methodNames) {
            Object value = tryInvokeNoArg(target, methodName);
            if (value != null) {
                return value;
            }
        }
        return null;
    }

    @Nullable
    private static Object tryInvokeNoArg(Object target, String methodName) {
        if (target == null) {
            return null;
        }
        try {
            Method method = target.getClass().getMethod(methodName);
            return method.invoke(target);
        } catch (Exception ignored) {
            return null;
        }
    }

    private static String formatPose(Pose3D pose) {
        if (pose == null || pose.getPosition() == null) {
            return "n/a";
        }
        return String.format(
                Locale.US,
                "(x=%.3f,y=%.3f,z=%.3f)",
                pose.getPosition().x,
                pose.getPosition().y,
                pose.getPosition().z
        );
    }

    private static String formatValue(Object value) {
        if (value == null) {
            return "n/a";
        }
        if (value instanceof Number) {
            return String.format(Locale.US, "%.3f", ((Number) value).doubleValue());
        }
        return value.toString();
    }

    public AprilTag getFirstObelisk() {
        return getAprilTags()
                .stream()
                .filter(AprilTag::isObelisk)
                .findFirst()
                .orElse(null);
    }

    public AprilTag getFirstDepot() {
        return getAprilTags()
                .stream()
                .filter(tag -> !tag.isObelisk())
                .findFirst()
                .orElse(null);
    }


    @Override
    public void invalidateCache() {
        resultCache.invalidateCache();
        statusCache.invalidateCache();
    }

    @Override
    public void updateCache() {
        resultCache.updateCache();
        statusCache.updateCache();
    }

    @Override
    public void setStrategy(Strategy strategy) {
        resultCache.setStrategy(strategy);
        statusCache.setStrategy(strategy);
    }

    @Override
    public Strategy getStrategy() {
        return resultCache.getStrategy();
    }

    public static final class AprilTag {

        public enum Type {
            BLUE_DEPOT(20, false),
            OBELISK_GPP(21, true),
            OBELISK_PGP(22, true),
            OBELISK_PPG(23, true),
            RED_DEPOT(24, false);

            private final int fiducialId;
            private final boolean isObelisk;

            Type(int fiducialId, boolean isObelisk) {
                this.fiducialId = fiducialId;
                this.isObelisk = isObelisk;
            }

            public int fiducialId() {
                return fiducialId;
            }

            public boolean isObelisk() {
                return isObelisk;
            }

            @Nullable
            public static Type fromFiducialId(int id) {
                for (Type t : values()) {
                    if (t.fiducialId == id) return t;
                }
                return null;
            }
        }

        private final Type type;

        // Pose state (per detection / updated over time)
        private Pose3D tagInCameraPose;     // target pose expressed in camera frame
        private long timestampNanos;

        public AprilTag(Type type) {
            this.type = type;
        }

        public AprilTag(Type type, Pose3D tagInCameraPose, long timestampNanos) {
            this.type = type;
            this.tagInCameraPose = tagInCameraPose;
            this.timestampNanos = timestampNanos;
        }

        @Nullable
        public static AprilTag fromDetection(int fiducialId, Pose3D pose, long timestampNanos) {
            Type type = Type.fromFiducialId(fiducialId);
            if (type == null) {
                return null;
            }
            return new AprilTag(type, pose, timestampNanos);
        }

        // Identity / config
        public Type type() {
            return type;
        }

        public int fiducialId() {
            return type.fiducialId();
        }

        public boolean isObelisk() {
            return type.isObelisk();
        }

        // Pose state
        public Pose3D tagInCameraPose() {
            return tagInCameraPose;
        }

        public long timestampNanos() {
            return timestampNanos;
        }

        public void updatePose(Pose3D tagInCameraPose, long timestampNanos) {
            this.tagInCameraPose = tagInCameraPose;
            this.timestampNanos = timestampNanos;
        }

        // Common helpers for turn to face tag + distance.
        // Camera frame in FTC Limelight pose space: y=right, z=forward.
        // The configurable offsets describe where the camera sits on the turret relative to the
        // turret aim origin (for example the launcher centerline). Positive values mean the camera
        // is physically right/forward of that aim origin, so they are added back to convert the
        // tag pose from camera space into turret-origin space.
        public double bearingDegToTag() {
            if (tagInCameraPose == null) throw new IllegalStateException("Pose not set");
            double y = getTagYInTurretFrameMeters();
            double z = getTagZInTurretFrameMeters();
            return -Math.toDegrees(Math.atan2(y, z));
        }

        public double distanceXYToTagMeters() {
            if (tagInCameraPose == null) throw new IllegalStateException("Pose not set");
            double y = getTagYInTurretFrameMeters();
            double z = getTagZInTurretFrameMeters();
            return Math.hypot(y, z);
        }

        public double getTagYInTurretFrameMeters() {
            if (tagInCameraPose == null) throw new IllegalStateException("Pose not set");
            return tagInCameraPose.getPosition().y + cameraOnTurretRightOffsetMeters;
        }

        public double getTagZInTurretFrameMeters() {
            if (tagInCameraPose == null) throw new IllegalStateException("Pose not set");
            return tagInCameraPose.getPosition().z + cameraOnTurretForwardOffsetMeters;
        }

        @NotNull
        @Override
        public String toString() {
            return "AprilTag{type=" + type +
                    ", id=" + fiducialId() +
                    ", isObelisk=" + isObelisk() +
                    ", timestampNanos=" + timestampNanos +
                    ", pose=" + tagInCameraPose +
                    '}';
        }
    }

}
