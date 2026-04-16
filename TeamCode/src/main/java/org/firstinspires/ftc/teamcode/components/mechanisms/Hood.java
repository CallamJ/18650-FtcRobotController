package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;

@Configurable
public class Hood {
    private final SmartServo servo;

    //arc angle in degrees 40-50

    public Hood(SmartServo servo) {
        this.servo = servo;
    }

    public void setTargetPosition(double targetPosition) {
        this.servo.setPosition(targetPosition);
    }

    public double getTargetPosition() {
        return servo.getPosition();
    }

}
