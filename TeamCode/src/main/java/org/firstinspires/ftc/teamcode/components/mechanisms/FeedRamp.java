package org.firstinspires.ftc.teamcode.components.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;

@Configurable
public class FeedRamp {
    SmartServo leftFeedServo, rightFeedServo;

    private double targetPosition;

    public static double min = 0;
    public static double max = 1;

    public static double engagedPosition = 0;
    public static double disengagedPosition = 1;



    public FeedRamp(SmartServo leftServo, SmartServo rightServo) {
        leftFeedServo = leftServo;
        rightFeedServo = rightServo;
        setTargetPosition(disengagedPosition);
    }

    public void setTargetPosition(double position) {
        this.targetPosition = clamp(position);
        leftFeedServo.setPosition(1 - targetPosition);
        rightFeedServo.setPosition(targetPosition);
    }

    public void bumpTargetPosition(double value){
        this.targetPosition += value;
        this.targetPosition = clamp(targetPosition);
        leftFeedServo.setPosition(1 - targetPosition);
        rightFeedServo.setPosition(targetPosition);
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    public void engage(){
        setTargetPosition(engagedPosition);
    }

    public void disengage(){
        setTargetPosition(disengagedPosition);
    }

    public boolean isEngaged(){
        return targetPosition == engagedPosition;
    }

    private double clamp(double value){
        return clamp(value, min, max);
    }

    private double clamp(double value, double min, double max){
        if(value < min) return min;
        return Math.min(value, max);
    }
}
