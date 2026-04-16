package org.firstinspires.ftc.teamcode.hardware.controllers;

import java.util.function.DoubleSupplier;

public class GravityPID extends DirectionalPID implements PositionControlAlgorithm {
    private final GravityFunction gravityFunc;
    private final DoubleSupplier g;
    protected double gResult;

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
    public double calcPosition(double target, double actual) {
        double basePID = super.calcPosition(target, actual);
        double gravityEffect = gravityFunc.apply(g.getAsDouble(), actual) * g.getAsDouble();
        result = basePID + gravityEffect;
        gResult = gravityEffect;
        return result;
    }

    public double gResult(){
        return gResult;
    }

    public static Builder gravityBuilder() {
        return new Builder();
    }

    public static class Builder extends DirectionalCoefficientBuilderBase<Builder> {
        private GravityFunction gravityFunc = (t, a) -> 0;
        private DoubleSupplier g = () -> 0;

        @Override
        protected Builder self() {
            return this;
        }

        public Builder setGravityFunction(GravityFunction func) { this.gravityFunc = func; return this; }
        public Builder g(double g) { this.g = () -> g; return this; }
        public Builder g(DoubleSupplier g) { this.g = g; return this; }

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
