package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.controllers.GravityPID;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;

@Configurable
public class Feeder {
    private final CRServo servo;
    private final SmartPotentiometer potentiometer;
    public static double restAngle = 0, triggerAngle = 90, tolerance = 1;
    public static boolean reverseServo;
    private State state = State.RESTING;
    private ChainedFuture<?> triggerFuture;
    private double targetPosition = 0;

    public Feeder(CRServo servo, SmartPotentiometer potentiometer) {
        potentiometer.reset();
        setTargetPosition(restAngle);

        this.servo = servo;
        this.potentiometer = potentiometer;

        OpModeCore.getTelemetry().addLine("Feeder")
                .addData("Current Angle", this::getCurrentPosition)
                .addData("Target Angle", this::getTargetPosition);
    }

    /**
     * @return a future that completes with null when the feeder has returned to resting.
     */
    @SuppressWarnings("UnusedReturnValue")
    public ChainedFuture<?> trigger() {
        this.state = State.TRIGGERED;
        setTargetPosition(triggerAngle);
        return triggerFuture = new ChainedFuture<>();
    }

    public void tick() {
        switch (state) {
            case RESTING : {
                setTargetPosition(restAngle);
                break;
            }
            case TRIGGERED: {
                if(getCurrentPosition() >= triggerAngle - tolerance) {
                    setTargetPosition(restAngle);
                    state = State.RETURNING_TO_REST;
                } else {
                    setTargetPosition(triggerAngle);
                }
                break;
            }
            case RETURNING_TO_REST: {
                if(getCurrentPosition() <= restAngle + tolerance) {
                    state = State.RESTING;
                    triggerFuture.complete(null);
                }
                break;
            }
        }
        tickPIDF();
    }

    protected void tickPIDF() {
        double power = 0;
        if(state == State.RESTING){
            power = 0;
        } else {
            if(getCurrentPosition() > getTargetPosition()){
                power = -0.5;
            } else if (getCurrentPosition() < getTargetPosition()) {
                power = 0.5;
            }
        }

        this.servo.setPower(reverseServo ? -power : power);
    }

    public double getCurrentPosition() {
        return potentiometer.getAngle();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    private boolean withinTolerance(double target, double actual, double tolerance) {
        return Math.abs(target - actual) < tolerance;
    }

    public enum State {
        RESTING,
        TRIGGERED,
        RETURNING_TO_REST
    }
}
