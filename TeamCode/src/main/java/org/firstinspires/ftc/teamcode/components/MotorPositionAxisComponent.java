package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.PositionControlAlgorithm;

/**
 * Position-controlled axis backed by a {@link SmartMotor}.
 */
public abstract class MotorPositionAxisComponent extends PositionAxisComponent {
    protected final SmartMotor motor;

    protected MotorPositionAxisComponent(SmartMotor motor, PositionControlAlgorithm controller) {
        super(controller);
        this.motor = motor;
    }

    @Override
    protected void applyOutput(double output, double target, double current) {
        motor.setPower(shapeMotorPower(output, target, current));
    }

    protected double shapeMotorPower(double output, double target, double current) {
        return output;
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
