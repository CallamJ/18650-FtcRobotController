package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.ControlAlgorithm;

/**
 * Shared base for closed-loop axes backed by a {@link SmartMotor}.
 */
public abstract class MotorAxisComponent extends AxisComponent {
    protected final SmartMotor motor;

    protected MotorAxisComponent(SmartMotor motor, ControlAlgorithm controller) {
        super(controller);
        this.motor = motor;
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }
}
