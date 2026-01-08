package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;

public class Collector {
    private final SmartMotor motor;

    public Collector(SmartMotor motor) {
        this.motor = motor;
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0);
    }

    public boolean isPowered(){
        return motor.getPower() > 0;
    }
}
