package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.VelocityPID;
import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;

@Configurable
public class Launcher {
    private final SmartMotor motor;

    private final VelocityPID controller;

    public static double kP = 0.001, kI = 0, kD = 0, kF = 0.00035, kV = 0.03, tolerance = 30;
    public static double maxVoltage = 14;
    private final DataFilter voltageFilter = new RollingAverage(100);

    public static float ticksPerDegree = (288f/360f);

    private double targetVelocity;

    public Launcher(SmartMotor motor) {
        this.motor = motor;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.controller = new VelocityPID.Builder()
                .setKP(() -> kP)
                .setKI(() -> kI)
                .setKD(() -> kD)
                .setKF(() -> kF)
                .setTolerance(tolerance)
                .build();
    }

    public void tick(){
        double result = controller.calcWithVelocity(targetVelocity, getVelocity());
        double voltageCompensation = targetVelocity != 0 ? (maxVoltage - voltageFilter.compute(Hardware.getControlHub().getInputVoltage(VoltageUnit.VOLTS))) * kV : 0;
        motor.setPower(result + voltageCompensation);
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

    public double getPidResult() {
        return controller.result();
    }

    public void stop() {
        setTargetVelocity(0);
    }

    public boolean isStopped() {
        return targetVelocity == 0;
    }

}
