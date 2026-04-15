package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.hardware.SmartEncoder;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Configurable
public class Turret extends AxisComponent {
    private final SmartEncoder encoder;
    private final SmartMotor motor;
    public static double kP = 0.02, kI = 0, kD = 0.015, kF = 0, tolerance = 1;
    public static float ticksPerDegree = (8192f / 360f) * (88f / 20f);
    public static double minAngle = -720;
    public static double maxAngle = 720;
    private double desiredTarget;

    public Turret(SmartMotor motor, SmartEncoder encoder) {
        super(
                new PID.Builder()
                        .setKP(() -> kP)
                        .setKI(() -> kI)
                        .setKD(() -> kD)
                        .setKF(() -> kF)
                        .setTolerance(tolerance)
                        .setDirectionalKF(true)
                        .build()
        );
        this.motor = motor;
        this.encoder = encoder;
        this.encoder.setDirection(Direction.REVERSE);
        this.encoder.reset();
        this.desiredTarget = 0;
    }

    @Override
    public void tick() {
        tickPIDF();
    }

    @Override
    protected void tickPIDF() {
        controller.calc(getTargetPosition(), getCurrentPosition());

        if(getTargetPosition() > getCurrentPosition()){
            motor.setPower(Math.abs(controller.result()));
        } else {
            motor.setPower(Math.abs(controller.result()) * -1);
        }

    }

    public double getDesiredTarget(){
        return desiredTarget;
    }

    @Override
    public void setTargetPosition(double position) {
        super.setTargetPosition(clamp(position, minAngle, maxAngle));
        desiredTarget = position;
    }

    @Override
    public double getCurrentPosition() {
        return encoder.getPosition() / ticksPerDegree;
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
