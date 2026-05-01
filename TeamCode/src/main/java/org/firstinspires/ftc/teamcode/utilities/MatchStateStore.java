package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.Nullable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.mechanisms.DriveBase;
import org.firstinspires.ftc.teamcode.components.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.components.subsystems.IndexerStorage;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.components.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.hardware.SmartLimelight3A;

public final class MatchStateStore {
    private static final String SNAPSHOT_KEY = "match_state_snapshot_v1";
    private static Snapshot latestLiveSnapshot;
    private static Snapshot latestPersistentSnapshot;
    private static ModeKind latestLiveSnapshotSource = ModeKind.UNKNOWN;

    private MatchStateStore() {}

    public enum AllianceColor {
        BLUE,
        RED;

        public AllianceColor opposite() {
            return this == BLUE ? RED : BLUE;
        }
    }

    public static final class Snapshot {
        public long savedAtUnixMs;
        public String allianceColor;
        public double poseXInches;
        public double poseYInches;
        public double poseHeadingDegrees;
        public String frontContent;
        public String rightContent;
        public String leftContent;
        public long indexerCurrentIndex;
        public long indexerTargetIndex;
        public double turretCurrentAngleDeg;
        public double turretTargetAngleDeg;
        public double turretDesiredTargetAngleDeg;
        public String obeliskType;
        public boolean obeliskVisible;
        public long obeliskSeenAtUnixMs;
    }

    public static void saveSnapshot(
            @Nullable DriveBase driveBase,
            @Nullable IndexerStorage indexerStorage,
            @Nullable Indexer indexer,
            @Nullable Turret turret,
            AllianceColor allianceColor,
            @Nullable SmartLimelight3A.AprilTag obeliskTag
    ) {
        Snapshot snapshot = new Snapshot();
        Snapshot previous = getLatestSnapshotForCarryForward();
        snapshot.savedAtUnixMs = System.currentTimeMillis();
        snapshot.allianceColor = allianceColor.name();

        if (driveBase != null) {
            Pose pose = driveBase.getPoseSimple();
            snapshot.poseXInches = pose.x();
            snapshot.poseYInches = pose.y();
            snapshot.poseHeadingDegrees = pose.heading();
        }

        if (indexerStorage != null) {
            snapshot.frontContent = indexerStorage.getFrontContent().name();
            snapshot.rightContent = indexerStorage.getRightContent().name();
            snapshot.leftContent = indexerStorage.getLeftContent().name();
        }

        if (indexer != null) {
            snapshot.indexerCurrentIndex = indexer.getCurrentIndex();
            snapshot.indexerTargetIndex = indexer.getTargetIndex();
        }
        if (turret != null) {
            snapshot.turretCurrentAngleDeg = turret.getCurrentPosition();
            snapshot.turretTargetAngleDeg = turret.getTargetPosition();
            snapshot.turretDesiredTargetAngleDeg = turret.getDesiredTarget();
        }

        if (previous != null) {
            snapshot.obeliskType = previous.obeliskType;
            snapshot.obeliskSeenAtUnixMs = previous.obeliskSeenAtUnixMs;
        }
        snapshot.obeliskVisible = false;
        if (obeliskTag != null && obeliskTag.type() != null) {
            snapshot.obeliskType = obeliskTag.type().name();
            snapshot.obeliskVisible = true;
            snapshot.obeliskSeenAtUnixMs = snapshot.savedAtUnixMs;
        }

        ModeKind currentModeKind = getCurrentModeKind();
        latestLiveSnapshot = snapshot;
        latestLiveSnapshotSource = currentModeKind;
        if (shouldPersistForMode(currentModeKind)) {
            savePersistentSnapshot(snapshot);
        }
    }

    @Nullable
    public static Snapshot getLatestSnapshot() {
        if (shouldUseLiveSnapshotForCurrentOpMode()) {
            if (latestLiveSnapshot != null && latestLiveSnapshotSource == ModeKind.AUTONOMOUS) {
                return latestLiveSnapshot;
            }
            if (latestPersistentSnapshot != null) {
                return latestPersistentSnapshot;
            }
        }
        return getStoredSnapshot();
    }

    @Nullable
    public static Snapshot getFreshSnapshot(long maxAgeMs) {
        Snapshot snapshot = getLatestSnapshot();
        if (snapshot == null) {
            return null;
        }
        if (snapshot.savedAtUnixMs <= 0) {
            return null;
        }
        if (System.currentTimeMillis() - snapshot.savedAtUnixMs > maxAgeMs) {
            return null;
        }
        return snapshot;
    }

    public static AllianceColor parseAllianceColor(@Nullable String value, AllianceColor fallback) {
        if (value == null) {
            return fallback;
        }
        try {
            return AllianceColor.valueOf(value);
        } catch (IllegalArgumentException ignored) {
            return fallback;
        }
    }

    public static IndexerStorage.SlotContent parseSlotContent(@Nullable String value, IndexerStorage.SlotContent fallback) {
        if (value == null) {
            return fallback;
        }
        try {
            return IndexerStorage.SlotContent.valueOf(value);
        } catch (IllegalArgumentException ignored) {
            return fallback;
        }
    }

    public static SmartLimelight3A.AprilTag.Type parseObeliskType(@Nullable String value, SmartLimelight3A.AprilTag.Type fallback) {
        if (value == null) {
            return fallback;
        }
        try {
            return SmartLimelight3A.AprilTag.Type.valueOf(value);
        } catch (IllegalArgumentException ignored) {
            return fallback;
        }
    }

    public static void clear() {
        latestLiveSnapshot = null;
        latestPersistentSnapshot = null;
        latestLiveSnapshotSource = ModeKind.UNKNOWN;
        if (PersistentStorage.isInitialized()) {
            PersistentStorage.remove(SNAPSHOT_KEY);
        }
    }

    @Nullable
    private static Snapshot getStoredSnapshot() {
        if (!PersistentStorage.isInitialized()) {
            return null;
        }
        latestPersistentSnapshot = PersistentStorage.getObject(SNAPSHOT_KEY, Snapshot.class);
        return latestPersistentSnapshot;
    }

    private static void savePersistentSnapshot(Snapshot snapshot) {
        latestPersistentSnapshot = snapshot;
        if (PersistentStorage.isInitialized()) {
            PersistentStorage.saveObject(SNAPSHOT_KEY, snapshot);
        }
    }

    private static boolean shouldPersistForMode(ModeKind modeKind) {
        return modeKind == ModeKind.AUTONOMOUS;
    }

    private static boolean shouldUseLiveSnapshotForCurrentOpMode() {
        return getCurrentModeKind() == ModeKind.TELEOP;
    }

    private static ModeKind getCurrentModeKind() {
        OpModeCore opMode = OpModeCore.getInstance();
        if (opMode == null) {
            return ModeKind.UNKNOWN;
        }

        ModeKind annotatedMode = getAnnotatedModeKind(opMode.getClass());
        if (annotatedMode != ModeKind.UNKNOWN) {
            return annotatedMode;
        }
        return ModeKind.UNKNOWN;
    }

    @Nullable
    private static Snapshot getLatestSnapshotForCarryForward() {
        if (latestLiveSnapshot != null) {
            return latestLiveSnapshot;
        }
        if (latestPersistentSnapshot != null) {
            return latestPersistentSnapshot;
        }
        return getStoredSnapshot();
    }

    private static ModeKind getAnnotatedModeKind(@Nullable Class<?> opModeClass) {
        Class<?> current = opModeClass;
        while (current != null) {
            if (current.isAnnotationPresent(TeleOp.class)) {
                return ModeKind.TELEOP;
            }
            if (current.isAnnotationPresent(Autonomous.class)) {
                return ModeKind.AUTONOMOUS;
            }
            current = current.getSuperclass();
        }
        return ModeKind.UNKNOWN;
    }

    private enum ModeKind {
        TELEOP,
        AUTONOMOUS,
        UNKNOWN
    }
}
