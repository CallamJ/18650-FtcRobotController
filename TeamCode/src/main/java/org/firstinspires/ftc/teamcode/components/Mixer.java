package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;

@Config
public class Mixer extends AxisComponent {

    public static double kP = 0.01, kI = 0, kD = 0.005, kF = 0.02;
    public static float ticksPerDegree = 8192f/360f;

    private final SmartMotor motor;


    public Mixer(SmartMotor motor) {
        super(
                new PID.Builder()
                .setKP(() -> kP)
                .setKI(() -> kI)
                .setKD(() -> kD)
                .setKF(() -> kF)
                .build()
        );
        this.motor = motor;
    }

    @Override
    protected void tickPIDF() {
        controller.calc(getTargetPosition(), getCurrentPosition());
        motor.setPower(controller.result());
    }

    @Override
    public double getCurrentPosition() {
        return motor.getCurrentPosition() / ticksPerDegree;
    }
}
