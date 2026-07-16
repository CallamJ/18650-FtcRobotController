package org.firstinspires.ftc.teamcode.hardware.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.Notifier;

import java.util.function.Supplier;

public class BangBangController implements PositionControlAlgorithm {
    private final Notifier noLongerBusyNotifier = new Notifier();
    private boolean isBusy = false, lastIsBusy = false;
    private final Supplier<Double> tolerance;
    private final Supplier<Double> maxDecel;
    private final Supplier<Double> maxPower;
    private final Supplier<Double> velocityTolerance, filterFraction, brakeMargin;
    private double result;
    private boolean initialized = false;
    private double lastActual;
    private Direction direction;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastVelocity;

    @Override
    public Notifier getNoLongerBusyNotifier() {
        return noLongerBusyNotifier;
    }

    @Override
    public boolean isBusy() {
        return isBusy;
    }

    @Override
    public double calcPosition(double target, double actual) {
        double error = target - actual;
        if (!initialized) {
            initialized = true;
            lastActual = actual;
            timer.reset();
            result = 0;
            return result;
        }

        double dt = timer.seconds();
        timer.reset();

        if (dt <= 1e-6) {
            return result;
        }

        double fraction = filterFraction.get();

        double rawVelocity = (actual - lastActual) / dt;
        double velocity = fraction * lastVelocity + (1 - fraction) * rawVelocity;
        double stoppingDistance = (Math.pow(velocity, 2) / (2 * maxDecel.get())) + brakeMargin.get();
        double distance = Math.abs(error);

        if(distance < tolerance.get() && Math.abs(velocity) < velocityTolerance.get()){
            isBusy = false;
            if(lastIsBusy){
                noLongerBusyNotifier.notifyWaitingThreads();
            }
            result = 0;
        } else {
            isBusy = true;
            if (velocity * error <= 0) {
                // moving away from target or stopped
                result = Math.signum(error) * maxPower.get();
            } else if (distance > stoppingDistance) {
                // still safe to keep accelerating toward target
                result = Math.signum(error) * maxPower.get();
            } else {
                // must brake
                result = -Math.signum(velocity) * maxPower.get();
            }
        }

        lastActual = actual;
        lastIsBusy = isBusy;
        lastVelocity = velocity;

        timer.reset();

        return result;
    }

    @Override
    public double result() {
        return result;
    }

    @Override
    public void setTolerance(double tolerance) {

    }

    @Override
    public double getTolerance() {
        return tolerance.get();
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    public double getLastVelocity() {
        return lastVelocity;
    }

    public BangBangController(Supplier<Double> maxPower, Supplier<Double> maxDecel, Supplier<Double> tolerance, Supplier<Double> velocityTolerance, Supplier<Double> brakeMargin, Supplier<Double> filterFraction){
        this.maxPower = maxPower;
        this.maxDecel = maxDecel;
        this.tolerance = tolerance;
        this.velocityTolerance = velocityTolerance;
        this.brakeMargin = brakeMargin;
        this.filterFraction = filterFraction;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private Supplier<Double> maxPower = () -> 0.0;
        private Supplier<Double> maxDecel = () -> 0.0;
        private Supplier<Double> tolerance = () -> 0.0;
        private Supplier<Double> velocityTolerance = () -> 0.0;
        private Supplier<Double> brakeMargin = () -> 0.0;
        private Supplier<Double> filterFraction = () -> 0.0;

        public Builder maxPower(double value) { this.maxPower = () -> value; return this; }
        public Builder maxDecel(double value) { this.maxDecel = () -> value; return this; }
        public Builder tolerance(double value) { this.tolerance = () -> value; return this; }
        public Builder velocityTolerance(double value) { this.velocityTolerance = () -> value; return this; }
        public Builder brakeMargin(double value) { this.brakeMargin = () -> value; return this; }
        public Builder filterFraction(double value) { this.filterFraction = () -> value; return this; }
        public Builder maxPower(Supplier<Double> value) { this.maxPower = value; return this; }
        public Builder maxDecel(Supplier<Double> value) { this.maxDecel = value; return this; }
        public Builder tolerance(Supplier<Double> value) { this.tolerance = value; return this; }
        public Builder velocityTolerance(Supplier<Double> value) { this.velocityTolerance = value; return this; }
        public Builder brakeMargin(Supplier<Double> value) { this.brakeMargin = value; return this; }
        public Builder filterFraction(Supplier<Double> value) { this.filterFraction = value; return this; }

        public BangBangController build() {
            return new BangBangController(maxPower, maxDecel, tolerance, velocityTolerance, brakeMargin, filterFraction);
        }
    }
}
