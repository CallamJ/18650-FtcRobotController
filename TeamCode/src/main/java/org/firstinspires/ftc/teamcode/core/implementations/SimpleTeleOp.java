package org.firstinspires.ftc.teamcode.core.implementations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.components.Collector;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.components.Indexer;
import org.firstinspires.ftc.teamcode.components.Loader;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Config
@TeleOp(name = "2 - Simple TeleOp")
public class SimpleTeleOp extends TeleOpCore {
    protected static DriveBase driveBase;
    protected static Loader loader;
    protected static Collector collector;
    protected static Indexer indexer;

    @Override
    protected void initialize(){
        super.initialize();

        DriveBaseMotorConfig.DriveBaseMotorConfigBuilder configBuilder = new DriveBaseMotorConfig.DriveBaseMotorConfigBuilder();
        configBuilder.leftFront("LFront", Direction.FORWARD);
        configBuilder.leftRear("LRear", Direction.REVERSE);
        configBuilder.rightFront("RFront", Direction.FORWARD);
        configBuilder.rightRear("RRear", Direction.REVERSE);

        driveBase = new DriveBase(hardwareMap, configBuilder.build());

        loader = new Loader(Hardware.getServo("loaderServo"));

        indexer = new Indexer(Hardware.getMotor("indexerMotor", true));

        collector = new Collector(Hardware.getMotor("collectorMotor"));
    }

    @Override
    protected void checkGamepads(Gamepad gamepad1, Gamepad gamepad2, Gamepad lastGamepad1, Gamepad lastGamepad2) {
        driveBase.moveUsingRR(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if(gamepad1.leftBumperWasPressed()){
            indexer.advanceIndexCounterclockwise();
        }
        if(gamepad1.rightBumperWasPressed()){
            indexer.advanceIndexClockwise();
        }

        if(gamepad1.aWasPressed()){
            if(collector.isPowered()){
                collector.stop();
            } else {
                collector.setPower(0.5);
            }
        }
    }
}
