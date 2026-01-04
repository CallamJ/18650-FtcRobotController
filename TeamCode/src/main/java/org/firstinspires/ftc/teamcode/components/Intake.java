package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;

public class Intake {
    private SmartMotor motor;

    public Intake(SmartMotor motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0);
    }
}
