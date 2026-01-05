package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;

@Config
public class Indexer extends AxisComponent {

    public static double kP = 0.01, kI = 0, kD = 0.005, kF = 0.02;
    public static float ticksPerDegree = 8192f/360f;

    private final SmartMotor motor;


    public Indexer(SmartMotor motor) {
        super(
                new PID.Builder()
                .setKP(() -> kP)
                .setKI(() -> kI)
                .setKD(() -> kD)
                .setKF(() -> kF)
                .build()
        );
        this.motor = motor;

        motor.getEncoder().reset();
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

    public long getCurrentIndex(){
        return degreesToIndex(getCurrentPosition());
    }

    public long getTargetIndex(){
        return degreesToIndex(getTargetPosition());
    }

    public void setTargetIndex(long index){
        setTargetPosition(index * 120);
    }

    public void advanceIndexClockwise(){
        advanceIndexClockwise(1);
    }

    public void advanceIndexClockwise(int count){
        setTargetIndex(getTargetIndex() + count);
    }

    public void advanceIndexCounterclockwise(){
        advanceIndexCounterclockwise(1);
    }

    public void advanceIndexCounterclockwise(int count){
        setTargetIndex(getTargetIndex() - count);
    }

    private long degreesToIndex(double degrees){
        return Math.round(degrees / 120);
    }
}
