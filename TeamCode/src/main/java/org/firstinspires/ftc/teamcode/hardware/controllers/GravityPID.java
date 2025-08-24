package org.firstinspires.ftc.teamcode.hardware.controllers;

import java.util.function.DoubleSupplier;

public class GravityPID extends DirectionalPID implements ControlAlgorithm {
    private final GravityFunction gravityFunc;
    private final DoubleSupplier g;

    protected GravityPID(
            DoubleSupplier kPForward, DoubleSupplier kIForward, DoubleSupplier kDForward, DoubleSupplier kFForward,
            DoubleSupplier kPReverse, DoubleSupplier kIReverse, DoubleSupplier kDReverse, DoubleSupplier kFReverse,
            GravityFunction gravityFunc, DoubleSupplier g, double tolerance
    ) {
        super(kPForward, kIForward, kDForward, kFForward, kPReverse, kIReverse, kDReverse, kFReverse, tolerance);
        this.gravityFunc = gravityFunc;
        this.g = g;
    }

    @Override
    public double calc(double target, double actual) {
        double basePID = super.calc(target, actual);
        double gravityEffect = gravityFunc.apply(g.getAsDouble(), actual) * g.getAsDouble();
        result = basePID + gravityEffect;
        return result;
    }

    public static class Builder {
        private DoubleSupplier kPForward = () -> 0, kIForward = () -> 0, kDForward = () -> 0, kFForward = () -> 0;
        private DoubleSupplier kPReverse = () -> 0, kIReverse = () -> 0, kDReverse = () -> 0, kFReverse = () -> 0;
        private GravityFunction gravityFunc = (t, a) -> 0;
        private DoubleSupplier g = () -> 0;
        private double tolerance;

        public Builder forwardKP(double kP) { this.kPForward = () -> kP; return this; }
        public Builder forwardKI(double kI) { this.kIForward = () -> kI; return this; }
        public Builder forwardKD(double kD) { this.kDForward = () -> kD; return this; }
        public Builder forwardKF(double kF) { this.kFForward = () -> kF; return this; }

        public Builder reverseKP(double kP) { this.kPReverse = () -> kP; return this; }
        public Builder reverseKI(double kI) { this.kIReverse = () -> kI; return this; }
        public Builder reverseKD(double kD) { this.kDReverse = () -> kD; return this; }
        public Builder reverseKF(double kF) { this.kFReverse = () -> kF; return this; }

        public Builder forwardKP(DoubleSupplier kP) { this.kPForward = kP; return this; }
        public Builder forwardKI(DoubleSupplier kI) { this.kIForward = kI; return this; }
        public Builder forwardKD(DoubleSupplier kD) { this.kDForward = kD; return this; }
        public Builder forwardKF(DoubleSupplier kF) { this.kFForward = kF; return this; }

        public Builder reverseKP(DoubleSupplier kP) { this.kPReverse = kP; return this; }
        public Builder reverseKI(DoubleSupplier kI) { this.kIReverse = kI; return this; }
        public Builder reverseKD(DoubleSupplier kD) { this.kDReverse = kD; return this; }
        public Builder reverseKF(DoubleSupplier kF) { this.kFReverse = kF; return this; }

        public Builder setGravityFunction(GravityFunction func) { this.gravityFunc = func; return this; }
        public Builder g(double g) { this.g = () -> g; return this; }
        public Builder g(DoubleSupplier g) { this.g = g; return this; }
        public Builder tolerance(double tolerance) { this.tolerance = tolerance; return this; }

        public GravityPID build() {
            return new GravityPID(
                    kPForward, kIForward, kDForward, kFForward,
                    kPReverse, kIReverse, kDReverse, kFReverse,
                    gravityFunc, g, tolerance
            );
        }
    }

    @FunctionalInterface
    public interface GravityFunction {
        double apply(double kG, double actual);
    }
}
