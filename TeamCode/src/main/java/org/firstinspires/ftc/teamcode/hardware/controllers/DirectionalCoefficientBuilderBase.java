package org.firstinspires.ftc.teamcode.hardware.controllers;

import java.util.function.DoubleSupplier;

/**
 * Shared builder state for controllers that use separate forward and reverse PIDF coefficients.
 */
public abstract class DirectionalCoefficientBuilderBase<T extends DirectionalCoefficientBuilderBase<T>> {
    protected DoubleSupplier kPForward = () -> 0, kIForward = () -> 0, kDForward = () -> 0, kFForward = () -> 0;
    protected DoubleSupplier kPReverse = () -> 0, kIReverse = () -> 0, kDReverse = () -> 0, kFReverse = () -> 0;
    protected double tolerance;

    protected abstract T self();

    public T forwardKP(double kP){ this.kPForward = () -> kP; return self(); }
    public T forwardKI(double kI){ this.kIForward = () -> kI; return self(); }
    public T forwardKD(double kD){ this.kDForward = () -> kD; return self(); }
    public T forwardKF(double kF){ this.kFForward = () -> kF; return self(); }
    public T reverseKP(double kP){ this.kPReverse = () -> kP; return self(); }
    public T reverseKI(double kI){ this.kIReverse = () -> kI; return self(); }
    public T reverseKD(double kD){ this.kDReverse = () -> kD; return self(); }
    public T reverseKF(double kF){ this.kFReverse = () -> kF; return self(); }
    public T forwardKP(DoubleSupplier kP){ this.kPForward = kP; return self(); }
    public T forwardKI(DoubleSupplier kI){ this.kIForward = kI; return self(); }
    public T forwardKD(DoubleSupplier kD){ this.kDForward = kD; return self(); }
    public T forwardKF(DoubleSupplier kF){ this.kFForward = kF; return self(); }
    public T reverseKP(DoubleSupplier kP){ this.kPReverse = kP; return self(); }
    public T reverseKI(DoubleSupplier kI){ this.kIReverse = kI; return self(); }
    public T reverseKD(DoubleSupplier kD){ this.kDReverse = kD; return self(); }
    public T reverseKF(DoubleSupplier kF){ this.kFReverse = kF; return self(); }

    public T tolerance(double tolerance){
        this.tolerance = tolerance;
        return self();
    }
}
