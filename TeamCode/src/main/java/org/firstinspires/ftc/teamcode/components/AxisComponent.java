package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.controllers.ControlAlgorithm;
import org.firstinspires.ftc.teamcode.utilities.Notifier;

/**
 * Generic closed-loop actuator axis driven by a {@link ControlAlgorithm}.
 */
public abstract class AxisComponent extends ActuatorComponent {
    protected final ControlAlgorithm controller;
    public final Notifier noLongerBusyNotifier;

    protected AxisComponent(ControlAlgorithm controller) {
        this.controller = controller;
        this.noLongerBusyNotifier = controller.getNoLongerBusyNotifier();
    }

    @Override
    public final void tick() {
        double target = getTargetValue();
        double current = getCurrentValue();
        double output = calculateOutput(target, current);
        applyOutput(output, target, current);
    }

    public final double getTargetValue() {
        return readTargetValue();
    }

    public final double getCurrentValue() {
        return readCurrentValue();
    }

    protected abstract double calculateOutput(double target, double current);

    protected abstract double readTargetValue();

    protected abstract double readCurrentValue();

    protected abstract void applyOutput(double output, double target, double current);

    public final double getValueError() {
        return getTargetValue() - getCurrentValue();
    }

    public boolean isBusy() {
        return Math.abs(getValueError()) >= getBusyTolerance();
    }

    protected double getBusyTolerance() {
        return controller.getTolerance();
    }

    @Override
    public double getPower() {
        return controller.result();
    }
}
