package org.firstinspires.ftc.teamcode.core.teleoptasks;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.components.SimpleFCS;
import org.firstinspires.ftc.teamcode.components.StorageController;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class TeleOpTaskContext {
    private final Supplier<DriveBase> driveBaseSupplier;
    private final Supplier<StorageController> storageControllerSupplier;
    private final Supplier<SimpleFCS> simpleFcsSupplier;
    private final Supplier<MatchStateStore.AllianceColor> allianceSupplier;
    private final Supplier<Double> runtimeSecSupplier;
    private final Consumer<String> warningLogger;

    public TeleOpTaskContext(
            Supplier<DriveBase> driveBaseSupplier,
            Supplier<StorageController> storageControllerSupplier,
            Supplier<SimpleFCS> simpleFcsSupplier,
            Supplier<MatchStateStore.AllianceColor> allianceSupplier,
            Supplier<Double> runtimeSecSupplier,
            Consumer<String> warningLogger
    ) {
        this.driveBaseSupplier = driveBaseSupplier;
        this.storageControllerSupplier = storageControllerSupplier;
        this.simpleFcsSupplier = simpleFcsSupplier;
        this.allianceSupplier = allianceSupplier;
        this.runtimeSecSupplier = runtimeSecSupplier;
        this.warningLogger = warningLogger;
    }

    public DriveBase driveBase() {
        return driveBaseSupplier.get();
    }

    public StorageController storageController() {
        return storageControllerSupplier.get();
    }

    public SimpleFCS simpleFcs() {
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
