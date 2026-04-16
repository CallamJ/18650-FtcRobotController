package org.firstinspires.ftc.teamcode.hardware.controllers;

import java.util.function.DoubleSupplier;

/**
 * Shared builder state for controllers that use a single set of PIDF coefficients.
 */
public abstract class CoefficientBuilderBase<T extends CoefficientBuilderBase<T>> {
    protected DoubleSupplier kP = () -> 0;
    protected DoubleSupplier kI = () -> 0;
    protected DoubleSupplier kD = () -> 0;
    protected DoubleSupplier kF = () -> 0;
    protected double tolerance;

    protected abstract T self();

    public T setKP(double kP) { this.kP = () -> kP; return self(); }
    public T setKI(double kI) { this.kI = () -> kI; return self(); }
    public T setKD(double kD) { this.kD = () -> kD; return self(); }
    public T setKF(double kF) { this.kF = () -> kF; return self(); }
    public T setKP(DoubleSupplier kP) { this.kP = kP; return self(); }
    public T setKI(DoubleSupplier kI) { this.kI = kI; return self(); }
    public T setKD(DoubleSupplier kD) { this.kD = kD; return self(); }
    public T setKF(DoubleSupplier kF) { this.kF = kF; return self(); }

    public T setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return self();
    }
}
