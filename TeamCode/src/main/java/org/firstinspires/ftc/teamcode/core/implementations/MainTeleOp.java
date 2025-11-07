package org.firstinspires.ftc.teamcode.core.implementations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;

@Config
@TeleOp(name = "1 - Main TeleOp")
public class MainTeleOp extends TeleOpCore {
    private boolean isHighPower = false;
    protected static DriveBase driveBase;

    //<editor-fold desc="Config">
    public static float LOW_POWER_MODIFIER = 0.25f;
    public static float HIGH_POWER_MODIFIER = 0.75f;
    public static float MAX_INCHES_PER_SECOND = 12f;
    //</editor-fold>


    @Override
    protected void initialize(){
        super.initialize();

        driveBase = new DriveBase(hardwareMap);

        telemetry.log();

        prettyTelem.addData("Power Factor", driveBase::getPoseSimple);
    }

    @Override
    protected void checkGamepads(Gamepad gamepad1, Gamepad gamepad2, Gamepad lastGamepad1, Gamepad lastGamepad2) {
        driveBase.moveUsingRR(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    protected void configureTelemetry(){}
}
