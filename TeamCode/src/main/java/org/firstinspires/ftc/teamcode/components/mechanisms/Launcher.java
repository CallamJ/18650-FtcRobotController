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
    private final Hardware hardware;

    public static double kP = 0.002, kI = 0, kD = 0, kF = 0.00025, kV = 0.03, tolerance = 30;
    public static double maxVoltage = 14;
    private final DataFilter voltageFilter = new RollingAverage(100);

    public static float ticksPerDegree = (112f/360f);

    private boolean runMaxPower = false;

    public Launcher(Hardware hardware, SmartMotor motor) {
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
        this.hardware = hardware;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    protected double shapeMotorPower(double output, double target, double current) {
        if(runMaxPower && (target + tolerance * 2) - current > tolerance ) {
            return 1;
        }
        double voltageCompensation = target != 0
                ? (maxVoltage - voltageFilter.compute(hardware.getControlHub().getInputVoltage(VoltageUnit.VOLTS))) * kV
                : 0;
        return output + voltageCompensation;
    }

    public void setRunMaxPower(boolean runMaxPower) {
        this.runMaxPower = runMaxPower;
    }

    @Override
    public void stop() {
        super.stop();
        setRunMaxPower(false);
    }

    public boolean isRunningMaxPower() {
        return runMaxPower;
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
