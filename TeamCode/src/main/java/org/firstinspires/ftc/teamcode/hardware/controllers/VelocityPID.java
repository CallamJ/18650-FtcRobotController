package org.firstinspires.ftc.teamcode.hardware.controllers;

import java.util.function.DoubleSupplier;

/**
 * A PID controller specifically designed for velocity control.
 * This extends the base PID class but modifies the feedforward term to work with velocity.
 * <p>
 * The key difference is that kF is used as a feedforward coefficient that gets multiplied
 * by the target velocity, rather than being a constant term.
 */
public class VelocityPID extends BasePIDController implements VelocityControlAlgorithm {
    protected final DoubleSupplier kP, kI, kD, kF;
    private double lastPosition = 0;
    private double currentVelocity = 0;
    private boolean firstCalculation = true;
    private final com.qualcomm.robotcore.util.ElapsedTime velocityTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    protected VelocityPID(DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD, DoubleSupplier kF, double tolerance) {
        super(tolerance);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
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

        return calcVelocity(targetVelocity, currentVelocity);
    }

    /**
     * Calculates the PID output for velocity control when you already have the velocity measured.
     *
     * @param targetVelocity The desired velocity
     * @param actualVelocity The actual measured velocity
     * @return The calculated motor power
     */
    @Override
    public double calcVelocity(double targetVelocity, double actualVelocity) {
        return calc(targetVelocity, actualVelocity, kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble(), kF.getAsDouble());
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

            // KEY DIFFERENCE: kF is multiplied by target velocity for feedforward
            double f = kF * target;

            double output = p + integral + d + f;

            if (direction == org.firstinspires.ftc.teamcode.utilities.Direction.REVERSE) {
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

        notifyIfNoLongerBusy(lastIsBusy);

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

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder extends CoefficientBuilderBase<Builder> {
        @Override
        protected Builder self() {
            return this;
        }

        public VelocityPID build() {
            return new VelocityPID(kP, kI, kD, kF, tolerance);
        }
    }
}
