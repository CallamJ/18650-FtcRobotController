package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;

@TeleOp(name = "3 - Motor Identifier")
public class MotorIdentifier extends TeleOpCore {
    public SmartMotor leftFront;
    public SmartMotor leftRear;
    public SmartMotor rightFront;
    public SmartMotor rightRear;

    @Override
    protected void initialize() {
        super.initialize();
        this.leftFront = Hardware.getMotor("LFront");
        this.leftRear = Hardware.getMotor("LRear");
        this.rightFront = Hardware.getMotor("RFront");
        this.rightRear = Hardware.getMotor("RRear");
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        if(gamepad1.a) {
            this.leftFront.setPower(0.5);
        } else {
            this.leftFront.setPower(0);
        }

        if(gamepad1.b) {
            this.leftRear.setPower(0.5);
        } else {
            this.leftRear.setPower(0);
        }

        if (gamepad1.x) {
            this.rightFront.setPower(0.5);
        } else {
            this.rightFront.setPower(0);
        }

        if(gamepad1.y) {
            this.rightRear.setPower(0.5);
        } else {
            this.rightRear.setPower(0);
        }
    }

}
