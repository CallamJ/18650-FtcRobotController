package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.VelocityControlAlgorithm;

/**
 * Velocity-controlled axis backed by a {@link SmartMotor}.
 */
public abstract class MotorVelocityAxisComponent extends VelocityAxisComponent {
    protected final SmartMotor motor;

    protected MotorVelocityAxisComponent(SmartMotor motor, VelocityControlAlgorithm controller) {
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
        super.stop();
        motor.setPower(0);
    }
}
