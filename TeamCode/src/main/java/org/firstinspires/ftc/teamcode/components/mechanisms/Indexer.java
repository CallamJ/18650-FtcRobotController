package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.components.MotorPositionAxisComponent;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Configurable
public class Indexer extends MotorPositionAxisComponent {

    public static double kP = 0.007, kI = 0, kD = 0.0075, kF = 0.0001, tolerance = 1, busyTolerance = 2.5;
    public static float ticksPerDegree = 8192f/360f;

    public Indexer(SmartMotor motor) {
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

        motor.getEncoder().setDirection(Direction.REVERSE);

        motor.getEncoder().reset();
    }

    @Override
    public boolean isBusy() {
        return Math.abs(getCurrentPosition() - getTargetPosition()) > busyTolerance;
    }

    public double getVelocity(){
        return motor.getVelocity();
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

    public short getNormalizedCurrentIndex() {
        return (short) Math.floorMod(getCurrentIndex(), 3);
    }

    public void bumpZero(int bumpVal){
        motor.getEncoder().addOffset(bumpVal);
    }

    private long degreesToIndex(double degrees){
        return Math.round(degrees / 120);
    }
}
