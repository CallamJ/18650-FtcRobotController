package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.components.*;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Configurable
@TeleOp(name = "4 - Servo Tester")
public class ServoTester extends TeleOpCore {
    private SmartServo servo;

    @Override
    protected void onInitialize(){
        //noinspection DuplicatedCode

        servo = Hardware.getServo("hoodServo");
        telemetry.addData("Target", servo::getPosition);
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        servo.setPosition(gamepad1.leftTrigger);
    }

}
