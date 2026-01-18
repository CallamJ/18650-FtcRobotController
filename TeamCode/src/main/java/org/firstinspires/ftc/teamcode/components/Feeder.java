package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.controllers.GravityPID;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;

@Config
public class Feeder extends AxisComponent {
    public static double kP = 0.005, kI = 0, kD = 0, kF = 0.05;
    public static double rKP = 0.005, rKI = 0, rKD = 0, rKF = 0.1;
    public static double gravity = 0.05, tolerance = 1;

    private final CRServo servo;
    private final SmartPotentiometer potentiometer;
    public static double restAngle = 0, triggerAngle = 90;
    public static boolean reverseServo;
    private State state = State.RESTING;
    private ChainedFuture<?> triggerFuture;

    public Feeder(CRServo servo, SmartPotentiometer potentiometer) {
        super(
                new GravityPID.Builder()
                        .forwardKP(() -> kP)
                        .forwardKI(() -> kI)
                        .forwardKD(() -> kD)
                        .forwardKF(() -> kF)
                        .reverseKP(() -> rKP)
                        .reverseKI(() -> rKI)
                        .reverseKD(() -> rKD)
                        .reverseKF(() -> rKF)
                        .g(() -> gravity)
                        .tolerance(tolerance)
                        .build()
        );
        potentiometer.reset();
        setTargetPosition(restAngle);

        this.servo = servo;
        this.potentiometer = potentiometer;

        GravityPID pid = (GravityPID) controller;

        OpModeCore.getTelemetry().addLine("Feeder")
                .addData("Current Angle", this::getCurrentPosition)
                .addData("Target Angle", this::getTargetPosition)
                .addData("State", () -> state )
                .addData("P Result", pid::pResult)
                .addData("I Result", pid::iResult)
                .addData("D Result", pid::dResult)
                .addData("F Result", pid::fResult)
                .addData("G Result", pid::gResult)
                .addData("Total Result", pid::result);

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
        super.tick();
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
    }

    @Override
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

    @Override
    public double getCurrentPosition() {
        return potentiometer.getAngle();
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
