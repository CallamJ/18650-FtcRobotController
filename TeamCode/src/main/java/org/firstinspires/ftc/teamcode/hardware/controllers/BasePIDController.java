package org.firstinspires.ftc.teamcode.hardware.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.Notifier;

/**
 * Shared PID state and bookkeeping for concrete PID-style controllers.
 */
public abstract class BasePIDController implements ControlAlgorithm {
    protected double integral, lastError, tolerance, minimum, result;
    protected double pResult, iResult, dResult, fResult;
    protected final ElapsedTime timer = new ElapsedTime();
    protected boolean isBusy = true;
    protected Direction direction;
    protected final Notifier noLongerBusyNotifier = new Notifier();

    protected BasePIDController(double tolerance) {
        this.tolerance = tolerance;
    }

    @Override
    public Notifier getNoLongerBusyNotifier() {
        return noLongerBusyNotifier;
    }

    @Override
    public boolean isBusy() {
        return isBusy;
    }

    @Override
    public double result() {
        return result;
    }

    public double pResult() {
        return pResult;
    }

    public double iResult() {
        return iResult;
    }

    public double dResult() {
        return dResult;
    }

    public double fResult() {
        return fResult;
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
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    protected void notifyIfNoLongerBusy(boolean wasBusy) {
        if (wasBusy && !isBusy) {
            noLongerBusyNotifier.notifyWaitingThreads();
        }
    }
}
