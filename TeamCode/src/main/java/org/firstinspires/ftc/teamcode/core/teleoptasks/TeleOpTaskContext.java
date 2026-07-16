package org.firstinspires.ftc.teamcode.core.teleoptasks;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.components.mechanisms.DriveBase;
import org.firstinspires.ftc.teamcode.components.subsystems.FireControlSystem;
import org.firstinspires.ftc.teamcode.components.subsystems.IndexerStorage;
import org.firstinspires.ftc.teamcode.components.subsystems.VolleyFireStorageManager;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class TeleOpTaskContext {
    private final Supplier<DriveBase> driveBaseSupplier;
    private final Supplier<IndexerStorage> indexerStorageSupplier;
    private final Supplier<VolleyFireStorageManager> volleyStorageSupplier;
    private final Supplier<FireControlSystem> simpleFcsSupplier;
    private final Supplier<MatchStateStore.AllianceColor> allianceSupplier;
    private final Supplier<Double> runtimeSecSupplier;
    private final Consumer<String> warningLogger;

    public TeleOpTaskContext(
            Supplier<DriveBase> driveBaseSupplier,
            Supplier<IndexerStorage> indexerStorageSupplier,
            Supplier<VolleyFireStorageManager> volleyStorageSupplier,
            Supplier<FireControlSystem> simpleFcsSupplier,
            Supplier<MatchStateStore.AllianceColor> allianceSupplier,
            Supplier<Double> runtimeSecSupplier,
            Consumer<String> warningLogger
    ) {
        this.driveBaseSupplier = driveBaseSupplier;
        this.indexerStorageSupplier = indexerStorageSupplier;
        this.volleyStorageSupplier = volleyStorageSupplier;
        this.simpleFcsSupplier = simpleFcsSupplier;
        this.allianceSupplier = allianceSupplier;
        this.runtimeSecSupplier = runtimeSecSupplier;
        this.warningLogger = warningLogger;
    }

    public DriveBase driveBase() {
        return driveBaseSupplier.get();
    }

    public IndexerStorage indexerStorage() {
        return indexerStorageSupplier.get();
    }

    public VolleyFireStorageManager volleyStorage() {
        return volleyStorageSupplier.get();
    }

    public FireControlSystem simpleFcs() {
        return simpleFcsSupplier.get();
    }

    public MatchStateStore.AllianceColor allianceColor() {
        return allianceSupplier.get();
    }

    public double runtimeSec() {
        return runtimeSecSupplier.get();
    }

    public void warn(String message) {
        if (warningLogger != null) {
            warningLogger.accept(message);
        }
    }

    public Follower follower() {
        DriveBase driveBase = driveBase();
        return driveBase == null ? null : driveBase.getFollower();
    }
}
