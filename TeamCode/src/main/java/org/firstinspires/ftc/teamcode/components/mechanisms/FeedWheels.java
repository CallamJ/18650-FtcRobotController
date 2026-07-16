package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.utilities.LiveMatchTuning;

public class FeedWheels {
    private final CRServo leftServo, rightServo;

    public FeedWheels(CRServo leftServo, CRServo rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }

    public void start(){
        leftServo.setPower(LiveMatchTuning.feedWheelPower);
        rightServo.setPower(LiveMatchTuning.feedWheelPower);
    }

    public void stop(){
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    public boolean isRunning(){
        return leftServo.getPower() != 0 || rightServo.getPower() != 0;
    }
}
