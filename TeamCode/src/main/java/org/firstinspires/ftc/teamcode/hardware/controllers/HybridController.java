package org.firstinspires.ftc.teamcode.hardware.controllers;

import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.Notifier;

public class HybridController implements ControlAlgorithm {
    private ControlAlgorithm holdController;
    private ControlAlgorithm moveController;
    private double tolerance;

    private double result;
    private double error;
    private boolean waitForMoveNoLongerBusy;

    public HybridController(ControlAlgorithm holdController, ControlAlgorithm moveController, double tolerance, boolean waitForMoveNoLongerBusy) {
        this.holdController = holdController;
        this.moveController = moveController;
        this.tolerance = tolerance;
        this.waitForMoveNoLongerBusy = waitForMoveNoLongerBusy;
    }

    @Override
    public Notifier getNoLongerBusyNotifier() {
        return null;
    }

    @Override
    public boolean isBusy() {
        return isHolding()
                ? holdController.isBusy()
                : moveController.isBusy();
    }

    @Override
    public double calc(double target, double actual) {
        this.error = Math.abs(target - actual);

        if (isHolding()) {
            result = holdController.calc(target, actual);
        } else {
            result = moveController.calc(target, actual);
        }

        return result;
    }

    public boolean isHolding() {
        return error <= tolerance && !moveController.isBusy();
    }

    public boolean isMoving() {
        return !isHolding();
    }

    @Override
    public double result() {
        return result;
    }

    @Override
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    @Override
    public double getTolerance() {
        return tolerance;
    }

    @Override
    public void setDirection(Direction direction) {
        holdController.setDirection(direction);
        moveController.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return holdController.getDirection();
    }

    public ControlAlgorithm getHoldController() {
        return holdController;
    }

    public ControlAlgorithm getMoveController() {
        return moveController;
    }
}