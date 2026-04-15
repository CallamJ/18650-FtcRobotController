package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.Nullable;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.components.Indexer;
import org.firstinspires.ftc.teamcode.components.StorageController;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.hardware.SmartLimelight3A;

public final class MatchStateStore {
    private static final String SNAPSHOT_KEY = "match_state_snapshot_v1";

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
            @Nullable StorageController storageController,
            @Nullable Indexer indexer,
            @Nullable Turret turret,
            AllianceColor allianceColor,
            @Nullable SmartLimelight3A.AprilTag obeliskTag
    ) {
        if (!PersistentStorage.isInitialized()) {
            return;
        }

        Snapshot snapshot = new Snapshot();
        Snapshot previous = getLatestSnapshot();
        snapshot.savedAtUnixMs = System.currentTimeMillis();
        snapshot.allianceColor = allianceColor.name();

        if (driveBase != null) {
            Pose pose = driveBase.getPoseSimple();
            snapshot.poseXInches = pose.x();
            snapshot.poseYInches = pose.y();
            snapshot.poseHeadingDegrees = pose.heading();
        }

        if (storageController != null) {
            snapshot.frontContent = storageController.getFrontContent().name();
            snapshot.rightContent = storageController.getRightContent().name();
            snapshot.leftContent = storageController.getLeftContent().name();
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

        PersistentStorage.saveObject(SNAPSHOT_KEY, snapshot);
    }

    @Nullable
    public static Snapshot getLatestSnapshot() {
        if (!PersistentStorage.isInitialized()) {
            return null;
        }
        return PersistentStorage.getObject(SNAPSHOT_KEY, Snapshot.class);
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

    public static StorageController.SlotContent parseSlotContent(@Nullable String value, StorageController.SlotContent fallback) {
        if (value == null) {
            return fallback;
        }
        try {
            return StorageController.SlotContent.valueOf(value);
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
        if (!PersistentStorage.isInitialized()) {
            return;
        }
        PersistentStorage.remove(SNAPSHOT_KEY);
    }
}
