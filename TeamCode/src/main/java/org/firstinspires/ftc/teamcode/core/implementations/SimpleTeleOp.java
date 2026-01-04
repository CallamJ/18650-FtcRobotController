package org.firstinspires.ftc.teamcode.core.implementations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;

@Config
@TeleOp(name = "2 - Simple TeleOp")
public class SimpleTeleOp extends TeleOpCore {
    protected static DriveBase driveBase;

    @Override
    protected void initialize(){
        super.initialize();

        driveBase = new DriveBase(hardwareMap);
    }

    @Override
    protected void checkGamepads(Gamepad gamepad1, Gamepad gamepad2, Gamepad lastGamepad1, Gamepad lastGamepad2) {
        driveBase.moveUsingRR(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
