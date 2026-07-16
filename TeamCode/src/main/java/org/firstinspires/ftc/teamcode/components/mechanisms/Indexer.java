package org.firstinspires.ftc.teamcode.components.mechanisms;

import org.firstinspires.ftc.teamcode.components.MotorPositionAxisComponent;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.LiveMatchTuning;

public class Indexer extends MotorPositionAxisComponent {

    private boolean poweredApproachActive = false;
    private double poweredApproachDirection = 0;
    private double poweredApproachPower = LiveMatchTuning.indexerPoweredMovePower;

    public Indexer(SmartMotor motor) {
        super(
                motor,
                PID.builder()
                .setKP(() -> LiveMatchTuning.indexerKp)
                .setKI(() -> LiveMatchTuning.indexerKi)
                .setKD(() -> LiveMatchTuning.indexerKd)
                .setKF(() -> LiveMatchTuning.indexerKf)
                .setTolerance(LiveMatchTuning.indexerToleranceDeg)
                .setDirectionalKF(true)
                .build()
        );

        motor.getEncoder().setDirection(Direction.REVERSE);

        motor.getEncoder().reset();
    }

    @Override
    public boolean isBusy() {
        return Math.abs(getCurrentPosition() - getTargetPosition()) > LiveMatchTuning.indexerBusyToleranceDeg;
    }

    @Override
    public void setTargetPosition(double targetPosition) {
        poweredApproachActive = false;
        poweredApproachDirection = 0;
        super.setTargetPosition(targetPosition);
    }

    public double getVelocity(){
        return motor.getVelocity();
    }

    @Override
    public double getCurrentPosition() {
        return motor.getCurrentPosition() / LiveMatchTuning.indexerTicksPerDegree;
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

    public void setTargetPositionWithPower(double targetPosition, double power) {
        double currentPosition = getCurrentPosition();
        double error = targetPosition - currentPosition;

        super.setTargetPosition(targetPosition);

        poweredApproachDirection = Math.signum(error);
        poweredApproachPower = Math.min(1.0, Math.max(0.0, Math.abs(power)));
        poweredApproachActive = poweredApproachPower > 0 && poweredApproachDirection != 0;
    }

    public void setTargetIndexWithPower(long index, double power) {
        setTargetPositionWithPower(index * 120, power);
    }

    public boolean isRunningPoweredMove() {
        return poweredApproachActive;
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

    public void advanceIndexClockwiseWithPower() {
        advanceIndexClockwiseWithPower(1, LiveMatchTuning.indexerPoweredMovePower);
    }

    public void advanceIndexClockwiseWithPower(double power) {
        advanceIndexClockwiseWithPower(1, power);
    }

    public void advanceIndexClockwiseWithPower(int count, double power) {
        setTargetIndexWithPower(getTargetIndex() + count, power);
    }

    public void advanceIndexCounterclockwiseWithPower() {
        advanceIndexCounterclockwiseWithPower(1, LiveMatchTuning.indexerPoweredMovePower);
    }

    public void advanceIndexCounterclockwiseWithPower(double power) {
        advanceIndexCounterclockwiseWithPower(1, power);
    }

    public void advanceIndexCounterclockwiseWithPower(int count, double power) {
        setTargetIndexWithPower(getTargetIndex() - count, power);
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

    @Override
    protected double calculateOutput(double target, double current) {
        if (poweredApproachActive) {
            double error = target - current;
            if (poweredApproachDirection != 0 && Math.signum(error) == poweredApproachDirection) {
                return poweredApproachDirection * poweredApproachPower;
            }
            poweredApproachActive = false;
            poweredApproachDirection = 0;
        }
        return super.calculateOutput(target, current);
    }
}
