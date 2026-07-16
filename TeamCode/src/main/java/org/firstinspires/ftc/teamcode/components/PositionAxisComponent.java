package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.controllers.PositionControlAlgorithm;

/**
 * Closed-loop axis whose controlled value is position.
 */
public abstract class PositionAxisComponent extends AxisComponent {
    private double targetPosition;

    protected PositionAxisComponent(PositionControlAlgorithm controller) {
        super(controller);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = normalizeTargetPosition(targetPosition);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public abstract double getCurrentPosition();

    public double getPositionError() {
        return getTargetPosition() - getCurrentPosition();
    }

    @Override
    protected final double readTargetValue() {
        return getTargetPosition();
    }

    @Override
    protected final double readCurrentValue() {
        return getCurrentPosition();
    }

    @Override
    protected double calculateOutput(double target, double current) {
        return ((PositionControlAlgorithm) controller).calcPosition(target, current);
    }

    protected double normalizeTargetPosition(double targetPosition) {
        return targetPosition;
    }
}
