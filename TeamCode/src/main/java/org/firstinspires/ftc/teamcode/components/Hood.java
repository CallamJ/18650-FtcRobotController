package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.controllers.ControlAlgorithm;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;

@Config
public class Hood {
    public static double kP = 0.01, kI = 0, kD = 0.005, kF = 0.02, tolerance = 1;

    private final CRServo lServo, rServo;
    private final SmartPotentiometer potentiometer;

    private double targetLaunchAngle;

    private final ControlAlgorithm controller;

    public Hood(CRServo lServo, CRServo rServo, SmartPotentiometer potentiometer) {
        this.lServo = lServo;
        this.rServo = rServo;
        this.potentiometer = potentiometer;
        this.controller = new PID.Builder()
                .setKP(() -> kP)
                .setKI(() -> kI)
                .setKD(() -> kD)
                .setKF(() -> kF)
                .setTolerance(tolerance)
                .build();
    }

    public boolean isBusy(){
        return Math.abs(angleToLaunchAngle(potentiometer.getAngle()) -  targetLaunchAngle) > controller.getTolerance();
    }

    public void setTargetLaunchAngle(double targetLaunchAngle) {
        this.targetLaunchAngle = targetLaunchAngle;
    }

    public double getCurrentLaunchAngle() {
        return angleToLaunchAngle(potentiometer.getAngle());
    }

    public double getTargetLaunchAngle() {
        return targetLaunchAngle;
    }

    public void tick(){
        controller.calc(targetLaunchAngle, getCurrentLaunchAngle());
        lServo.setPower(controller.result());
        rServo.setPower(controller.result());
    }

    private double angleToLaunchAngle(double position) {
        throw new UnsupportedOperationException("This method is not yet implemented!"); //TODO: need to implement Hood.angleToLaunchAngle
    }
}
