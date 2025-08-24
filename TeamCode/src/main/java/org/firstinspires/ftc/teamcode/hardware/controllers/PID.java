package org.firstinspires.ftc.teamcode.hardware.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.Notifier;

import java.util.function.DoubleSupplier;

/**
 * A PID controller that implements a Proportional-Integral-Derivative (PID) control loop.
 * This class is designed to be used for controlling systems like motors, servos, or any other
 * system where you need to apply feedback control to reach a target value.
 * <p>
 * The controller uses PIDF (Proportional, Integral, Derivative, and Feedforward) terms to calculate
 * the output based on the error between the target and actual values.
 */
public class PID implements ControlAlgorithm {
    protected double integral, lastError, tolerance, minimum, result;
    protected final ElapsedTime timer = new ElapsedTime();
    private boolean isBusy = true;
    private Direction direction;
    Notifier noLongerBusyNotifier = new Notifier();

    protected final DoubleSupplier kP, kI, kD, kF;

    protected PID(DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD, DoubleSupplier kF, double tolerance) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
    }

    /**
     * Sets the direction of the PID controller. This will determine whether the output
     * should be reversed or not.
     *
     * @param direction The direction to set for the controller. It can be either {@link Direction#FORWARD} or {@link Direction#REVERSE}.
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Gets the current direction of the PID controller.
     *
     * @return The current direction of the controller.
     */
    public Direction getDirection() {
        return direction;
    }

    @Override
    public Notifier getNoLongerBusyNotifier() {
        return noLongerBusyNotifier;
    }

    public double calc(double target, double actual) {
        return calc(target, actual, kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble(), kF.getAsDouble());
    }

    protected double calc(double target, double actual, double kP, double kI, double kD, double kF) {
        double currentError = target - actual;

        boolean lastIsBusy = isBusy;

        if (Math.abs(currentError) > tolerance) {
            double timeChange = timer.milliseconds();
            timer.reset();

            double p = kP * currentError;

            integral += kI * currentError * timeChange;

            double d = kD * (currentError - lastError);

            lastError = currentError;

            double output = p + integral + d + kF;

            if (direction == Direction.REVERSE) {
                output = -output;
            }

            result = output;
            isBusy = true;
        } else {
            result = kF * (Math.abs(currentError) / tolerance);
            isBusy = false;
        }

        if(lastIsBusy && !isBusy)
            noLongerBusyNotifier.notifyWaitingThreads();

        // Return the last calculated output.
        return result;
    }

    /**
     * Gets the most recent result from the PID controller.
     * This is the output of the last call to {@link #calc(double, double)}.
     *
     * @return The last calculated output of the PID controller.
     */
    public double result() {
        return result;
    }

    /**
     * Sets the tolerance for the PID controller.
     * If the error is within this tolerance, the controller will stop adjusting the output.
     *
     * @param tolerance The tolerance value to set.
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getTolerance(){
        return tolerance;
    }


    public static class Builder {
        private DoubleSupplier kP = () -> 0, kI = () -> 0, kD = () -> 0, kF = () -> 0;
        private double tolerance;

        public Builder setKP(double kP) { this.kP = () -> kP; return this; }
        public Builder setKI(double kI) { this.kI = () -> kI; return this; }
        public Builder setKD(double kD) { this.kD = () -> kD; return this; }
        public Builder setKF(double kF) { this.kF = () -> kF; return this; }
        public Builder setKP(DoubleSupplier kP) { this.kP = kP; return this; }
        public Builder setKI(DoubleSupplier kI) { this.kI = kI; return this; }
        public Builder setKD(DoubleSupplier kD) { this.kD = kD; return this; }
        public Builder setKF(DoubleSupplier kF) { this.kF = kF; return this; }
        public Builder setTolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public PID build() { return new PID(kP, kI, kD, kF, tolerance); }
    }
}
