package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.components.MotorPositionAxisComponent;
import org.firstinspires.ftc.teamcode.hardware.SmartEncoder;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Configurable
public class Turret extends MotorPositionAxisComponent {
    private final SmartEncoder encoder;
    public static double kP = 0.02, kI = 0, kD = 0.015, kF = 0, tolerance = 1;
    public static float ticksPerDegree = (8192f / 360f) * (88f / 20f);
    public static double minAngle = -90;
    public static double maxAngle = 90;
    private double desiredTarget;

    public Turret(SmartMotor motor, SmartEncoder encoder) {
        super(
                motor,
                PID.builder()
                        .setKP(() -> kP)
                        .setKI(() -> kI)
                        .setKD(() -> kD)
                        .setKF(() -> kF)
                        .setTolerance(tolerance)
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
        return encoder.getPosition() / ticksPerDegree;
    }

    @Override
    public void setTargetPosition(double position) {
        desiredTarget = position;
        super.setTargetPosition(position);
    }

    @Override
    protected double normalizeTargetPosition(double targetPosition) {
        return clamp(targetPosition, minAngle, maxAngle);
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
