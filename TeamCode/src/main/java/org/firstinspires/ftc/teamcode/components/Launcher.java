package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.ControlAlgorithm;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;

@Config
public class Launcher {
    private final SmartMotor motor;

    private final ControlAlgorithm controller;

    public static double kP = 0.001, kI = 0, kD = 0.005, kF = 0.02, tolerance = 10;

    public static float ticksPerDegree = (288f/360f);

    private double targetVelocity;

    public Launcher(SmartMotor motor) {
        this.motor = motor;
        this.controller = new PID.Builder()
                .setKP(() -> kP)
                .setKI(() -> kI)
                .setKD(() -> kD)
                .setKF(() -> kF)
                .setTolerance(tolerance)
                .build();
    }

    public void tick(){
        motor.setPower(controller.calc(targetVelocity, getVelocity()));
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocity() {
        return motor.getVelocity() / ticksPerDegree;
    }

    public double getPower() {
        return motor.getPower();
    }

    public void stop() {
        setTargetVelocity(0);
    }

    public boolean isStopped() {
        return targetVelocity == 0;
    }

}
