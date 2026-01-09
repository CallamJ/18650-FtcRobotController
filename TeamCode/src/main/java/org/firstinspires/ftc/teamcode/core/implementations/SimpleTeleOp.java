package org.firstinspires.ftc.teamcode.core.implementations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.components.Collector;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.components.Indexer;
import org.firstinspires.ftc.teamcode.components.Feeder;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Config
@TeleOp(name = "2 - Simple TeleOp")
public class SimpleTeleOp extends TeleOpCore {
    protected static DriveBase driveBase;
    protected static Feeder feeder;
    protected static Collector collector;
    protected static Indexer indexer;

    @Override
    protected void initialize(){
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
                    Hardware.getServo("feederServo"),
                    Hardware.getPotentiometer("feederPotentiometer", 270, 3.3)
            );
        } catch (Exception e) {
            prettyTelem.error("Feeder failed to initialize, skipping: " + e.getMessage());
        }

        try {
            indexer = new Indexer(Hardware.getMotor("indexerMotor", true));
        } catch (Exception e) {
            prettyTelem.error("Indexer failed to initialize, skipping: " + e.getMessage());
        }

        try {
            collector = new Collector(Hardware.getMotor("collectorMotor"));
        } catch (Exception e) {
            prettyTelem.error("Collector failed to initialize, skipping: " + e.getMessage());
        }
    }

    @Override
    protected void checkGamepads(Gamepad gamepad1, Gamepad gamepad2, Gamepad lastGamepad1, Gamepad lastGamepad2) {
        if(driveBase != null){
            driveBase.moveUsingPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        if(feeder != null){
            if(gamepad1.x == lastGamepad1.x){
                feeder.trigger();
            }
        }

        if(indexer != null){
            if(gamepad1.left_bumper && !lastGamepad1.left_bumper){
                indexer.advanceIndexCounterclockwise();
            }
            if(gamepad1.right_bumper && !lastGamepad1.right_bumper){
                indexer.advanceIndexClockwise();
            }
        }

        if(collector != null){
            if(gamepad1.a && !lastGamepad1.a){
                double forwardPower = 1;
                if(collector.getPower() == forwardPower){
                    collector.stop();
                } else {
                    collector.setPower(forwardPower);
                }
            }

            if(gamepad1.b && !lastGamepad1.b){
                double reversePower = 0.5;
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
        if(feeder != null){
            feeder.tick();
        }
        if(indexer != null){
            indexer.tick();
        }
    }
}
