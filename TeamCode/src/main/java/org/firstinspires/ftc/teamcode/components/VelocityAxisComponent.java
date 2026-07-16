package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.controllers.VelocityControlAlgorithm;

/**
 * Closed-loop axis whose controlled value is velocity.
 */
public abstract class VelocityAxisComponent extends AxisComponent {
    private double targetVelocity;

    protected VelocityAxisComponent(VelocityControlAlgorithm controller) {
        super(controller);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = normalizeTargetVelocity(targetVelocity);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public abstract double getCurrentVelocity();

    public double getVelocityError() {
        return getTargetVelocity() - getCurrentVelocity();
    }

    public boolean isStopped() {
        return getTargetVelocity() == 0;
    }

    @Override
    public void stop() {
        setTargetVelocity(0);
    }

    @Override
    protected final double readTargetValue() {
        return getTargetVelocity();
    }

    @Override
    protected final double readCurrentValue() {
        return getCurrentVelocity();
    }

    @Override
    protected double calculateOutput(double target, double current) {
        return ((VelocityControlAlgorithm) controller).calcVelocity(target, current);
    }

    protected double normalizeTargetVelocity(double targetVelocity) {
        return targetVelocity;
    }
}
