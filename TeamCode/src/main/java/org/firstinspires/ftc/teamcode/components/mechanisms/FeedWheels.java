package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;

@Configurable
public class FeedWheels {
    private final CRServo leftServo, rightServo;
    public static double power = 1;

    public FeedWheels(CRServo leftServo, CRServo rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }

    public void start(){
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    public void stop(){
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    public boolean isRunning(){
        return leftServo.getPower() != 0 || rightServo.getPower() != 0;
    }
}
