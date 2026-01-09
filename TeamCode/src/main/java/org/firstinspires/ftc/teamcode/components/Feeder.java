package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;

@Config
public class Feeder {
    private final SmartServo servo;
    private final SmartPotentiometer potentiometer;
    public static double restAngle = 0, triggerAngle = 180;
    public static double restPosition = 0, triggerPosition = 1;
    public static double angleTolerance = 2;
    public static boolean reverseServo;
    private ChainedFuture<Object> triggerFuture;
    private State state = State.RESTING;

    public Feeder(SmartServo servo, SmartPotentiometer potentiometer) {
        this.servo = servo;
        this.potentiometer = potentiometer;
    }

    public ChainedFuture<Object> trigger() {
        this.state = State.TRIGGERED;
        triggerFuture = new ChainedFuture<>();
        return triggerFuture;
    }

    public boolean isBusy(){
        return state != State.RESTING;
    }

    public void tick() {
        switch (state) {
            case RESTING : {
                break;
            }
            case TRIGGERED: {
                if(withinTolerance(potentiometer.getAngle(), triggerAngle, angleTolerance)){
                    this.servo.setPosition(reverseServo ? 1 : 0);
                    state = State.RETURNING_TO_REST;
                } else {
                    this.servo.setPosition(reverseServo ? 0 : 1);
                }
                break;
            }
            case RETURNING_TO_REST: {
                if(withinTolerance(potentiometer.getAngle(), restAngle, angleTolerance)){
                    this.servo.setPosition(restPosition);
                    state = State.RESTING;
                    triggerFuture.complete(null);
                }
                break;
            }
        }
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
