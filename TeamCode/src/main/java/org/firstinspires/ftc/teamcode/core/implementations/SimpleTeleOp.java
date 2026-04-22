package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.components.mechanisms.*;
import org.firstinspires.ftc.teamcode.components.subsystems.FeedSystem;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Configurable
@TeleOp(name = "2 - Simple TeleOp")
public class SimpleTeleOp extends TeleOpCore {
    protected static DriveBase driveBase;
    protected static FeedWheels feedWheels;
    protected static FeedRamp feedRamp;
    protected static FeedSystem feeder;
    protected static Collector collector;
    protected static Indexer indexer;
    protected static Launcher launcher;
    protected static Turret turret;

    @Override
    protected void onInitialize() {
        //noinspection DuplicatedCode

        DriveBaseMotorConfig.DriveBaseMotorConfigBuilder configBuilder = new DriveBaseMotorConfig.DriveBaseMotorConfigBuilder();
        configBuilder.leftFront("LFront", Direction.FORWARD);
        configBuilder.leftRear("LRear", Direction.FORWARD);
        configBuilder.rightFront("RFront", Direction.REVERSE);
        configBuilder.rightRear("RRear", Direction.FORWARD);

        try {
            driveBase = new DriveBase(hardwareMap, configBuilder.build(), false);
        } catch (Exception e) {
            prettyTelem.error("Drive base failed to initialize, skipping: " + e.getMessage());
        }

        try {
            feedWheels = new FeedWheels(
                    hardwareMap.get(CRServo.class, "leftFeedServo"),
                    hardwareMap.get(CRServo.class, "rightFeedServo")
            );
            feedRamp = new FeedRamp(
                    hardware.getServo("leftFeedRampServo"),
                    hardware.getServo("rightFeedRampServo")
            );

            feeder = new FeedSystem(feedWheels, feedRamp);
            feeder.stopFeeding();
        } catch (Exception e) {
            prettyTelem.error("Feeder failed to initialize, skipping: " + e.getMessage());
        }

        try {
            indexer = new Indexer(hardware.getMotor("indexerMotor", true));
        } catch (Exception e) {
            prettyTelem.error("Indexer failed to initialize, skipping: " + e.getMessage());
        }

        try {
            launcher = new Launcher(hardware, hardware.getMotor("launcherMotor"));
        } catch (Exception e) {
            prettyTelem.error("Launcher failed to initialize, skipping: " + e.getMessage());
        }

        try {
            collector = new Collector(hardware.getMotor("collectorMotor"));
        } catch (Exception e) {
            prettyTelem.error("Collector failed to initialize, skipping: " + e.getMessage());
        }

        try {
            turret = new Turret(hardware.getMotor("turretMotor"), hardware.getMotor("turretMotor").getEncoder());
        } catch (Exception e) {
            prettyTelem.error("Turret failed to initialize, skipping: " + e.getMessage());
        }
    }

    boolean runFeeders = false;

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        //noinspection DuplicatedCode

        if (driveBase != null) {
            driveBase.moveUsingPower(gamepad1.leftStickX, gamepad1.leftStickY, gamepad1.rightStickX);
        }

        if (feeder != null) {
            if (gamepad1.yPressed()) {
                feeder.toggleFeeding();
            }
        }

        if (indexer != null) {
            if (gamepad1.leftBumperPressed()) {
                indexer.advanceIndexCounterclockwise();
            }
            if (gamepad1.rightBumperPressed()) {
                indexer.advanceIndexClockwise();
            }
        }

        if (launcher != null) {
            if (gamepad1.xPressed()) {
                double launcherVelocity = 15000;
                if (launcher.getTargetVelocity() == launcherVelocity) {
                    launcher.stop();
                } else {
                    launcher.setTargetVelocity(launcherVelocity);
                }
            }
        }

        if (collector != null) {
            if (gamepad1.aPressed()) {
                double forwardPower = 1;
                if (collector.getPower() == forwardPower) {
                    collector.stop();
                } else {
                    collector.setPower(forwardPower);
                }
            }

            if (gamepad1.bPressed()) {
                double reversePower = -0.5;
                if (collector.getPower() == reversePower) {
                    collector.stop();
                } else {
                    collector.setPower(reversePower);
                }
            }
        }

        if (turret != null) {
            turret.setTargetPosition(turret.getCurrentPosition() + (gamepad1.rightTrigger - gamepad1.leftTrigger) * 5);
        }
    }

    @Override
    protected void onTick() {
        if (indexer != null) {
            indexer.tick();
        }
        if (launcher != null) {
            launcher.tick();
        }
    }
}
