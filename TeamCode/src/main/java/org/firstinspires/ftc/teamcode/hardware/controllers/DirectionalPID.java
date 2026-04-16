package org.firstinspires.ftc.teamcode.hardware.controllers;

import java.util.function.DoubleSupplier;

public class DirectionalPID extends PID implements PositionControlAlgorithm {
	private final DoubleSupplier kPForward, kIForward, kDForward, kFForward;
	private final DoubleSupplier kPReverse, kIReverse, kDReverse, kFReverse;

	protected double rPResult;
	protected double rIResult;
	protected double rDResult;
	protected double rFResult;

	protected DirectionalPID(
			DoubleSupplier kPForward, DoubleSupplier kIForward, DoubleSupplier kDForward, DoubleSupplier kFForward,
			DoubleSupplier kPReverse, DoubleSupplier kIReverse, DoubleSupplier kDReverse, DoubleSupplier kFReverse,
			double tolerance
	){
		super(() -> 0, () -> 0, () -> 0, () -> 0, tolerance, false); // Placeholder values
		this.kPForward = kPForward;
		this.kIForward = kIForward;
		this.kDForward = kDForward;
		this.kFForward = kFForward;
		this.kPReverse = kPReverse;
		this.kIReverse = kIReverse;
		this.kDReverse = kDReverse;
		this.kFReverse = kFReverse;
	}

    @Override
	public double calcPosition(double target, double actual){
		double error = target - actual;
		if(error >= 0){
			return calc(target, actual, kPForward.getAsDouble(), kIForward.getAsDouble(), kDForward.getAsDouble(), kFForward.getAsDouble());
		} else {
			return calc(target, actual, kPReverse.getAsDouble(), kIReverse.getAsDouble(), kDReverse.getAsDouble(), -kFReverse.getAsDouble());
		}
	}

    public static Builder directionalBuilder() {
        return new Builder();
    }

	public static class Builder extends DirectionalCoefficientBuilderBase<Builder> {
        @Override
        protected Builder self() {
            return this;
        }

		public DirectionalPID build(){
			return new DirectionalPID(kPForward, kIForward, kDForward, kFForward, kPReverse, kIReverse, kDReverse, kFReverse, tolerance);
		}
	}
}
