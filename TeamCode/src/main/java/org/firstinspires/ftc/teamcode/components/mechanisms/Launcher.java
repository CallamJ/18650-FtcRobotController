package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.components.MotorVelocityAxisComponent;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.VelocityPID;
import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;
import org.firstinspires.ftc.teamcode.utilities.LiveMatchTuning;

public class Launcher extends MotorVelocityAxisComponent {
    private final Hardware hardware;

    private final DataFilter voltageFilter = new RollingAverage(100);

    private boolean runMaxPower = false;

    public Launcher(Hardware hardware, SmartMotor motor) {
        super(
                motor,
                VelocityPID.builder()
                        .setKP(() -> LiveMatchTuning.launcherKp)
                        .setKI(() -> LiveMatchTuning.launcherKi)
                        .setKD(() -> LiveMatchTuning.launcherKd)
                        .setKF(() -> LiveMatchTuning.launcherKf)
                        .setTolerance(LiveMatchTuning.launcherTolerance)
                        .build()
        );
        this.hardware = hardware;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    protected double shapeMotorPower(double output, double target, double current) {
        if(runMaxPower && (target + LiveMatchTuning.launcherTolerance * 2) - current > LiveMatchTuning.launcherTolerance ) {
            return 1;
        }
        double voltageCompensation = target != 0
                ? (LiveMatchTuning.launcherMaxVoltage - voltageFilter.compute(hardware.getControlHub().getInputVoltage(VoltageUnit.VOLTS))) * LiveMatchTuning.launcherKv
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
        return motor.getVelocity() / LiveMatchTuning.launcherTicksPerDegree;
    }

    public double getVelocity() {
        return getCurrentVelocity();
    }

    public double getPidResult() {
        return controller.result();
    }
}
