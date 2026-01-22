package org.firstinspires.ftc.teamcode.core.implementations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.components.*;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Config
@TeleOp(name = "1 - Main TeleOp")
public class MainTeleOp extends TeleOpCore {
    protected static DriveBase driveBase;
    protected static Feeder feeder;
    protected static Collector collector;
    protected static Indexer indexer;
    protected static Launcher launcher;
    protected static StorageController storageController;

    @Override
    protected void initialize(){
        //noinspection DuplicatedCode

        super.initialize();

        DriveBaseMotorConfig.DriveBaseMotorConfigBuilder configBuilder = new DriveBaseMotorConfig.DriveBaseMotorConfigBuilder();
        configBuilder.leftFront("LFront", Direction.FORWARD);
        configBuilder.leftRear("LRear", Direction.FORWARD);
        configBuilder.rightFront("RFront", Direction.REVERSE);
        configBuilder.rightRear("RRear", Direction.FORWARD);

        try {
            driveBase = new DriveBase(hardwareMap, configBuilder.build());
        } catch (Exception e) {
            prettyTelem.error("Drive base failed to initialize, skipping: " + e.getMessage());
        }

        try {
            feeder = new Feeder(
                    hardwareMap.get(CRServo.class, "feederServo"),
                    Hardware.getPotentiometer("feederPotentiometer", 270, 3.3)
            );
            indexer = new Indexer(Hardware.getMotor("indexerMotor"));
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
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {

        if(driveBase != null){
            driveBase.moveUsingPower(gamepad1.leftStickX, gamepad1.leftStickY, gamepad1.rightStickX);
        }

        if(feeder != null){
            if(gamepad1.yPressed()){
                feeder.trigger();
            }
        }

        if(storageController != null){
            if(gamepad1.leftBumperPressed()){
                indexer.advanceIndexCounterclockwise();
            }
            if(gamepad1.rightBumperPressed()){
                indexer.advanceIndexClockwise();
            }
        }

        if(launcher != null){
            if(gamepad1.xPressed()){
                double launcherVelocity = 25000;
                if(launcher.getTargetVelocity() == launcherVelocity){
                    launcher.stop();
                } else {
                    launcher.setTargetVelocity(launcherVelocity);
                }
            }
        }

        //noinspection DuplicatedCode
        if(collector != null){
            if(gamepad1.aPressed()){
                double forwardPower = 1;
                if(collector.getPower() == forwardPower){
                    collector.stop();
                } else {
                    collector.setPower(forwardPower);
                }
            }

            if(gamepad1.bPressed()){
                double reversePower = -0.5;
                if(collector.getPower() == reversePower){
                    collector.stop();
                } else {
                    collector.setPower(reversePower);
                }
            }
        }
    }

    @Override
    public void tick(){
        super.tick();

        if(storageController != null){
            storageController.tick();
        }

        if(launcher != null){
            launcher.tick();
        }
    }
}
