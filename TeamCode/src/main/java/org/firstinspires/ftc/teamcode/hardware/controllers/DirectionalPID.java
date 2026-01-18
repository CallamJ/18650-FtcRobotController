package org.firstinspires.ftc.teamcode.hardware.controllers;

import java.util.function.DoubleSupplier;

public class DirectionalPID extends PID implements ControlAlgorithm {
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
		super(() -> 0, () -> 0, () -> 0, () -> 0, tolerance); // Placeholder values
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
	public double calc(double target, double actual){
		double error = target - actual;
		if(error >= 0){
			return calc(target, actual, kPForward.getAsDouble(), kIForward.getAsDouble(), kDForward.getAsDouble(), kFForward.getAsDouble());
		} else {
			return calc(target, actual, kPReverse.getAsDouble(), kIReverse.getAsDouble(), kDReverse.getAsDouble(), -kFReverse.getAsDouble());
		}
	}

	public static class Builder {
		private DoubleSupplier kPForward = () -> 0, kIForward = () -> 0, kDForward = () -> 0, kFForward = () -> 0;
		private DoubleSupplier kPReverse = () -> 0, kIReverse = () -> 0, kDReverse = () -> 0, kFReverse = () -> 0;
		private double tolerance;

		public Builder forwardKP(double kP){
			this.kPForward = () -> kP;
			return this;
		}

		public Builder forwardKI(double kI){
			this.kIForward = () -> kI;
			return this;
		}

		public Builder forwardKD(double kD){
			this.kDForward = () -> kD;
			return this;
		}

		public Builder forwardKF(double kF){
			this.kFForward = () -> kF;
			return this;
		}

		public Builder reverseKP(double kP){
			this.kPReverse = () -> kP;
			return this;
		}

		public Builder reverseKI(double kI){
			this.kIReverse = () -> kI;
			return this;
		}

		public Builder reverseKD(double kD){
			this.kDReverse = () -> kD;
			return this;
		}

		public Builder reverseKF(double kF){
			this.kFReverse = () -> kF;
			return this;
		}

		public Builder forwardKP(DoubleSupplier kP){
			this.kPForward = kP;
			return this;
		}

        public Builder forwardKI(DoubleSupplier kI){
            this.kIForward = kI;
            return this;
        }

        public Builder forwardKD(DoubleSupplier kD){
            this.kDForward = kD;
            return this;
        }

        public Builder forwardKF(DoubleSupplier kF){
            this.kFForward = kF;
            return this;
        }

        public Builder reverseKP(DoubleSupplier kP){
            this.kPReverse = kP;
            return this;
        }

        public Builder reverseKI(DoubleSupplier kI){
            this.kIReverse = kI;
            return this;
        }

        public Builder reverseKD(DoubleSupplier kD){
            this.kDReverse = kD;
            return this;
        }

        public Builder reverseKF(DoubleSupplier kF){
            this.kFReverse = kF;
            return this;
        }

		public Builder tolerance(double tolerance){
			this.tolerance = tolerance;
			return this;
		}

		public DirectionalPID build(){
			return new DirectionalPID(kPForward, kIForward, kDForward, kFForward, kPReverse, kIReverse, kDReverse, kFReverse, tolerance);
		}
	}
}
