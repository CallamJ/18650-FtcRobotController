package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Configurable
public class Collector {
    private final SmartMotor motor;
    public static Direction direction = Direction.REVERSE;

    public Collector(SmartMotor motor) {
        this.motor = motor;
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        motor.setPower(power);
        motor.setDirection(direction.toMotorDirection());
    }

    public double getPower() {
        return motor.getPower();
    }

    public void stop() {
        motor.setPower(0);
    }

    public boolean isPowered(){
        return motor.getPower() != 0;
    }
}
