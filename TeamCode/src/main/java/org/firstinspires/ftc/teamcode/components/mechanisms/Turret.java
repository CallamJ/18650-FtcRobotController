package org.firstinspires.ftc.teamcode.components.mechanisms;

import org.firstinspires.ftc.teamcode.components.MotorPositionAxisComponent;
import org.firstinspires.ftc.teamcode.hardware.SmartEncoder;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.LiveMatchTuning;

public class Turret extends MotorPositionAxisComponent {
    private final SmartEncoder encoder;
    private double desiredTarget;

    public Turret(SmartMotor motor, SmartEncoder encoder) {
        super(
                motor,
                PID.builder()
                        .setKP(() -> LiveMatchTuning.turretKp)
                        .setKI(() -> LiveMatchTuning.turretKi)
                        .setKD(() -> LiveMatchTuning.turretKd)
                        .setKF(() -> LiveMatchTuning.turretKf)
                        .setTolerance(LiveMatchTuning.turretToleranceDeg)
                        .setDirectionalKF(true)
                        .build()
        );
        this.encoder = encoder;
        this.encoder.setDirection(Direction.REVERSE);
        this.encoder.reset();
        this.desiredTarget = 0;
    }

    @Override
    protected double shapeMotorPower(double output, double target, double current) {
        return target > current ? Math.abs(output) : -Math.abs(output);
    }

    @Override
    public double getCurrentPosition() {
        return encoder.getPosition() / LiveMatchTuning.turretTicksPerDegree;
    }

    @Override
    public void setTargetPosition(double position) {
        desiredTarget = position;
        super.setTargetPosition(position);
    }

    @Override
    protected double normalizeTargetPosition(double targetPosition) {
        return clamp(targetPosition, LiveMatchTuning.turretMinAngleDeg, LiveMatchTuning.turretMaxAngleDeg);
    }

    public double getDesiredTarget(){
        return desiredTarget;
    }

    public void bumpZero(int bumpTicks) {
        encoder.addOffset(bumpTicks);
    }

    public void setCurrentAsZero() {
        encoder.resetAs(0);
    }

    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}
