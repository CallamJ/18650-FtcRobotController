package org.firstinspires.ftc.teamcode.hardware.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utilities.Direction;

import java.util.function.DoubleSupplier;

/**
 * A PID controller specifically designed for velocity control.
 * This extends the base PID class but modifies the feedforward term to work with velocity.
 * <p>
 * The key difference is that kF is used as a feedforward coefficient that gets multiplied
 * by the target velocity, rather than being a constant term.
 */
public class VelocityPID extends PID {
    private double lastPosition = 0;
    private double currentVelocity = 0;
    private boolean firstCalculation = true;
    private final ElapsedTime velocityTimer = new ElapsedTime();

    protected VelocityPID(DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD, DoubleSupplier kF, double tolerance) {
        super(kP, kI, kD, kF, tolerance);
    }

    /**
     * Calculates the PID output for velocity control.
     * This method automatically calculates velocity from position measurements.
     *
     * @param targetVelocity The desired velocity
     * @param currentPosition The current position (used to calculate actual velocity)
     * @return The calculated motor power
     */
    public double calc(double targetVelocity, double currentPosition) {
        // Calculate velocity from position change
        if (firstCalculation) {
            lastPosition = currentPosition;
            velocityTimer.reset();
            firstCalculation = false;
            currentVelocity = 0;
        } else {
            double dt = velocityTimer.seconds();
            if (dt > 0) {
                currentVelocity = (currentPosition - lastPosition) / dt;
                lastPosition = currentPosition;
                velocityTimer.reset();
            }
        }

        return calcWithVelocity(targetVelocity, currentVelocity);
    }

    /**
     * Calculates the PID output for velocity control when you already have the velocity measured.
     *
     * @param targetVelocity The desired velocity
     * @param actualVelocity The actual measured velocity
     * @return The calculated motor power
     */
    public double calcWithVelocity(double targetVelocity, double actualVelocity) {
        return calc(targetVelocity, actualVelocity, kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble(), kF.getAsDouble());
    }

    @Override
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

            // KEY DIFFERENCE: kF is multiplied by target velocity for feedforward
            double f = kF * target;

            double output = p + integral + d + f;

            if (direction == Direction.REVERSE) {
                output = -output;
            }

            pResult = p;
            iResult = integral;
            dResult = d;
            fResult = f;

            result = output;
            isBusy = true;
        } else {
            // When within tolerance, use proportional feedforward
            double f = kF * target;

            result = f;
            pResult = 0;
            iResult = 0;
            dResult = 0;
            fResult = f;

            isBusy = false;
        }

        if (lastIsBusy && !isBusy)
            noLongerBusyNotifier.notifyWaitingThreads();

        return result;
    }

    /**
     * Gets the most recently calculated velocity.
     * Only valid if using the calc(targetVelocity, currentPosition) method.
     *
     * @return The current velocity in units/second
     */
    public double getCurrentVelocity() {
        return currentVelocity;
    }

    /**
     * Resets the velocity calculation state.
     * Call this when you want to restart velocity measurements from scratch.
     */
    public void resetVelocity() {
        firstCalculation = true;
        currentVelocity = 0;
        lastPosition = 0;
        velocityTimer.reset();
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

        public VelocityPID build() {
            return new VelocityPID(kP, kI, kD, kF, tolerance);
        }
    }
}