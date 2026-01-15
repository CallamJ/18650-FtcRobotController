package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;
import org.firstinspires.ftc.teamcode.utilities.Notifier;

@Config
public class Feeder extends AxisComponent {
    public static double kP = 0.005, kI = 0, kD = 0, kF = 0.025, tolerance = 1;

    private final CRServo servo;
    private final SmartPotentiometer potentiometer;
    public static double restAngle = 0, triggerAngle = 90;
    public static double angleTolerance = 2;
    public static boolean reverseServo;
    private State state = State.RESTING;

    public Feeder(CRServo servo, SmartPotentiometer potentiometer) {
        super(
                new PID.Builder()
                        .setKP(() -> kP)
                        .setKI(() -> kI)
                        .setKD(() -> kD)
                        .setKF(() -> kF)
                        .setTolerance(tolerance)
                        .build()
        );
        potentiometer.reset();

        this.servo = servo;
        this.potentiometer = potentiometer;

        OpModeCore.getTelemetry().addLine("Feeder")
                .addData("Current Angle", this::getCurrentPosition)
                .addData("Target Angle", this::getTargetPosition)
                .addData("State", () -> state );
    }

    @SuppressWarnings("UnusedReturnValue")
    public void trigger() {
        this.state = State.TRIGGERED;
    }

    public void tick() {
        super.tick();
        switch (state) {
            case RESTING : {
                break;
            }
            case TRIGGERED: {
                if(withinTolerance(triggerAngle, getCurrentPosition(), angleTolerance)){
                    setTargetPosition(restAngle);
                    state = State.RETURNING_TO_REST;
                } else {
                    setTargetPosition(triggerAngle);
                }
                break;
            }
            case RETURNING_TO_REST: {
                if(withinTolerance(restAngle, getCurrentPosition(), angleTolerance)){
                    state = State.RESTING;
                }
                break;
            }
        }
    }

    @Override
    protected void tickPIDF() {
        controller.calc(getTargetPosition(), getCurrentPosition());
        this.servo.setPower(controller.result());
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
