package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.components.MotorVelocityAxisComponent;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.VelocityPID;
import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;

@Configurable
public class Launcher extends MotorVelocityAxisComponent {

    public static double kP = 0.001, kI = 0, kD = 0, kF = 0.00035, kV = 0.03, tolerance = 30;
    public static double maxVoltage = 14;
    private final DataFilter voltageFilter = new RollingAverage(100);

    public static float ticksPerDegree = (288f/360f);

    public Launcher(SmartMotor motor) {
        super(
                motor,
                VelocityPID.builder()
                        .setKP(() -> kP)
                        .setKI(() -> kI)
                        .setKD(() -> kD)
                        .setKF(() -> kF)
                        .setTolerance(tolerance)
                        .build()
        );
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    protected double shapeMotorPower(double output, double target, double current) {
        double voltageCompensation = target != 0
                ? (maxVoltage - voltageFilter.compute(Hardware.getControlHub().getInputVoltage(VoltageUnit.VOLTS))) * kV
                : 0;
        return output + voltageCompensation;
    }

    @Override
    public double getCurrentVelocity() {
        return motor.getVelocity() / ticksPerDegree;
    }

    public double getVelocity() {
        return getCurrentVelocity();
    }

    public double getPidResult() {
        return controller.result();
    }
}
