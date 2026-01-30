package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.components.*;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Direction;

import java.nio.file.Path;

public abstract class AutoOpBase extends OpModeCore {

    protected static DriveBase driveBase;
    protected static Feeder feeder;
    protected static Collector collector;
    protected static Indexer indexer;
    protected static Launcher launcher;
    protected static StorageController storageController;

    protected void initialize() {
        super.initialize();
        DriveBaseMotorConfig.DriveBaseMotorConfigBuilder configBuilder = new DriveBaseMotorConfig.DriveBaseMotorConfigBuilder();
        configBuilder.leftFront("LFront", Direction.FORWARD);
        configBuilder.leftRear("LRear", Direction.FORWARD);
        configBuilder.rightFront("RFront", Direction.REVERSE);
        configBuilder.rightRear("RRear", Direction.FORWARD);

        try {
            driveBase = new DriveBase(hardwareMap, configBuilder.build(), true);
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
            storageController = new StorageController(
                    feeder,
                    indexer,
                    collector,
                    Hardware.getColorSensor("frontColorSensor")
            );
        } catch (Exception e) {
            prettyTelem.error("Feeder failed to initialize, skipping: " + e.getMessage());
        }

        try {
            launcher = new Launcher(Hardware.getMotor("launcherMotor"));
        } catch (Exception e) {
            prettyTelem.error("Launcher failed to initialize, skipping: " + e.getMessage());
        }

        prettyTelem.addLine("Localization")
                .addData("X", () -> driveBase.getPoseSimple().x())
                .addData("Y", () -> driveBase.getPoseSimple().y())
                .addData("Heading", () -> driveBase.getPoseSimple().heading());
    }

    @Override
    protected void run() {
        super.run();
        Follower follower = driveBase.getFollower();

        follower.followPath(buildPath(follower.pathBuilder()));
    }

    @Override
    public void tick() {
        super.tick();

        if(storageController != null){
            storageController.tick();
        }

        if(launcher != null){
            launcher.tick();
        }

        if(driveBase != null){
            driveBase.getFollower().update();
        }
    }

    public abstract PathChain buildPath(PathBuilder pathBuilder);
}
