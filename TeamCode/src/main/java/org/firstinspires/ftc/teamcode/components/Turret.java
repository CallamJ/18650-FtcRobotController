package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.hardware.SmartEncoder;
import org.firstinspires.ftc.teamcode.hardware.controllers.PID;

public class Turret extends AxisComponent {

    private final SmartEncoder encoder;
    private final CRServo servo;

    public static double kP = 0.01, kI = 0, kD = 0.005, kF = 0.02, tolerance = 1;
    public static float ticksPerDegree = (8192f/360f) * 5; //todo: put actual gear ratio here

    public Turret(CRServo servo, SmartEncoder encoder) {
        super(
                new PID.Builder()
                        .setKP(() -> kP)
                        .setKI(() -> kI)
                        .setKD(() -> kD)
                        .setKF(() -> kF)
                        .setTolerance(tolerance)
                        .build()
        );
        this.servo = servo;
        this.encoder = encoder;
    }


    @Override
    protected void tickPIDF() {
        controller.calc(getTargetPosition(), getCurrentPosition());
    }

    @Override
    public double getCurrentPosition() {
        return encoder.getPosition() / ticksPerDegree;
    }
}
