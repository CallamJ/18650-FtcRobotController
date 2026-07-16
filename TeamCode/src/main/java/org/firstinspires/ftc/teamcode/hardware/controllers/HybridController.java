package org.firstinspires.ftc.teamcode.hardware.controllers;

import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.Notifier;

public class HybridController implements PositionControlAlgorithm {
    private PositionControlAlgorithm holdController;
    private PositionControlAlgorithm moveController;
    private double tolerance;

    private double result;
    private double error;
    private boolean waitForMoveNoLongerBusy;

    public HybridController(PositionControlAlgorithm holdController, PositionControlAlgorithm moveController, double tolerance, boolean waitForMoveNoLongerBusy) {
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
    public double calcPosition(double target, double actual) {
        this.error = Math.abs(target - actual);

        if (isHolding()) {
            result = holdController.calcPosition(target, actual);
        } else {
            result = moveController.calcPosition(target, actual);
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

    public PositionControlAlgorithm getHoldController() {
        return holdController;
    }

    public PositionControlAlgorithm getMoveController() {
        return moveController;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private PositionControlAlgorithm holdController;
        private PositionControlAlgorithm moveController;
        private double tolerance;
        private boolean waitForMoveNoLongerBusy;

        public Builder holdController(PositionControlAlgorithm controller) {
            this.holdController = controller;
            return this;
        }

        public Builder moveController(PositionControlAlgorithm controller) {
            this.moveController = controller;
            return this;
        }

        public Builder tolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public Builder waitForMoveNoLongerBusy(boolean value) {
            this.waitForMoveNoLongerBusy = value;
            return this;
        }

        public HybridController build() {
            return new HybridController(holdController, moveController, tolerance, waitForMoveNoLongerBusy);
        }
    }
}
