package org.firstinspires.ftc.teamcode.hardware.controllers;

import org.firstinspires.ftc.teamcode.utilities.Direction;

import java.util.function.DoubleSupplier;

/**
 * A PID controller that implements a Proportional-Integral-Derivative (PID) control loop.
 * This class is designed to be used for controlling systems like motors, servos, or any other
 * system where you need to apply feedback control to reach a target value.
 * <p>
 * The controller uses PIDF (Proportional, Integral, Derivative, and Feedforward) terms to calculate
 * the output based on the error between the target and actual values.
 */
public class PID extends BasePIDController implements PositionControlAlgorithm {
    protected boolean directionalKF = true;

    protected final DoubleSupplier kP, kI, kD, kF;

    protected PID(DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD, DoubleSupplier kF, double tolerance, boolean directionalKF) {
        super(tolerance);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.directionalKF = directionalKF;
    }

    @Override
    public double calcPosition(double target, double actual) {
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

            double output;
            if(directionalKF) {
                output = p + integral + d + kF * (Math.signum(currentError));
                fResult = kF * Math.signum(currentError);
            } else {
                output = p + integral + d + kF;
                fResult = kF;
            }

            if (direction == Direction.REVERSE) {
                output = -output;
            }

            pResult = p;
            iResult = integral;
            dResult = d;



            result = output;
            isBusy = true;
        } else {
            result = kF * (Math.abs(currentError) / tolerance);
            pResult = 0;
            iResult = 0;
            dResult = 0;
            fResult = result;

            isBusy = false;
        }

        notifyIfNoLongerBusy(lastIsBusy);

        // Return the last calculated output.
        return result;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder extends CoefficientBuilderBase<Builder> {
        private boolean directionalKF = true;

        @Override
        protected Builder self() {
            return this;
        }

        public Builder setDirectionalKF(boolean directionalKF) {
            this.directionalKF = directionalKF;
            return this;
        }

        public PID build() { return new PID(kP, kI, kD, kF, tolerance,  directionalKF); }
    }
}
