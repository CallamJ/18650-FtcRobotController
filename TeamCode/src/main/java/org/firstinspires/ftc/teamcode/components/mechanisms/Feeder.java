package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;

@Configurable
public class Feeder {
    private final CRServo servo;
    private final SmartPotentiometer potentiometer;
    public static double restAngle = 0, triggerAngle = 90, tolerance = 1;
    public static boolean reverseServo;
    private State state = State.RESTING;
    private Double lastTriggerDurationMs;
    private double targetPosition = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public Feeder(CRServo servo, SmartPotentiometer potentiometer) {
        potentiometer.reset();
        setTargetPosition(restAngle);

        this.servo = servo;
        this.potentiometer = potentiometer;
    }

    /**
     * Starts a feeder trigger cycle.
     */
    public void trigger() {
        this.state = State.TRIGGERED;
        setTargetPosition(triggerAngle);
        timer.reset();
        lastTriggerDurationMs = null;
    }

    public void tick() {
        switch (state) {
            case RESTING : {
                setTargetPosition(restAngle);
                break;
            }
            case TRIGGERED: {
                if(
                        getCurrentPosition() >= triggerAngle - tolerance ||
                        timer.milliseconds() > 2000 // timeout
                ) {
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
                    lastTriggerDurationMs = timer.milliseconds();
                }
                break;
            }
        }
        tickPIDF();
    }

    protected void tickPIDF() {
        double power = 0;
        if (state == State.RESTING) {
            power = 0;
        } else {
            if (getCurrentPosition() > getTargetPosition()) {
                power = -1;
            } else if (getCurrentPosition() < getTargetPosition()) {
                power = 1;
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

    public State getState() {
        return state;
    }

    public Double getLastTriggerDurationMs() {
        return lastTriggerDurationMs;
    }

    public enum State {
        RESTING,
        TRIGGERED,
        RETURNING_TO_REST
    }
}
