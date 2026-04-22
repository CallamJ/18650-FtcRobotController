package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;

@TeleOp(name = "7 - ContinuousServoTest")
public class ContinuousServoTest extends TeleOpCore {
    SmartServo left, right;

    @Override
    protected void onInitialize() {
        left = hardware.getServo("leftFeedRampServo");
        right = hardware.getServo("rightFeedRampServo");
        prettyTelem.addData("left", left::getPosition);
        prettyTelem.addData("right", right::getPosition);
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        left.setPosition(gamepad1.leftStickX);
        right.setPosition(gamepad1.rightStickX);
    }
}
