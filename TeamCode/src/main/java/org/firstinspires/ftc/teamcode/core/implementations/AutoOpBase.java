package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.components.*;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Direction;

import java.util.List;

public abstract class AutoOpBase extends OpModeCore {

    protected static DriveBase driveBase;
    protected static Feeder feeder;
    protected static Collector collector;
    protected static Indexer indexer;
    protected static Launcher launcher;
    protected static StorageController storageController;

    protected static Limelight3A limelight;

    protected int fiducialId = -1;

    private List<Integer> validFidIds = List.of(21, 22, 23);


    protected void initialize() {
        super.initialize();
        DriveBaseMotorConfig.DriveBaseMotorConfigBuilder configBuilder = new DriveBaseMotorConfig.DriveBaseMotorConfigBuilder();
        configBuilder.leftFront("LFront", Direction.FORWARD);
        configBuilder.leftRear("LRear", Direction.FORWARD);
        configBuilder.rightFront("RFront", Direction.REVERSE);
        configBuilder.rightRear("RRear", Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        prettyTelem.addLine("Limelight")
                .addData("FPS", ()-> limelight.getStatus().getFps())
                .addData("Fiducial ID", () -> {
                    List<LLResultTypes.FiducialResult> fiducialResults = limelight.getLatestResult().getFiducialResults();
                    for (LLResultTypes.FiducialResult result : fiducialResults) {
                        if(fiducialId == -1 && validFidIds.contains(result.getFiducialId())) {
                            this.fiducialId = result.getFiducialId();
                        }
                    }
                    return fiducialId;
                })
                .addData("Is Result Valid?", () -> limelight.getLatestResult().isValid());

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
                    Hardware.getColorSensor("frontColorSensor"),
                    hardwareMap.get(Servo.class, "leftLED"),
                    hardwareMap.get(Servo.class, "rightLED"),
                    hardwareMap.get(Servo.class, "frontLED")
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

    public abstract PathChain buildPath(PathBuilder pathBuilder, Follower follower);
}
