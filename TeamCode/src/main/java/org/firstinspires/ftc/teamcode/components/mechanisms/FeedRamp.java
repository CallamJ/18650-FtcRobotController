package org.firstinspires.ftc.teamcode.components.mechanisms;

import org.firstinspires.ftc.teamcode.hardware.SmartServo;
import org.firstinspires.ftc.teamcode.utilities.LiveMatchTuning;

public class FeedRamp {
    SmartServo leftFeedServo, rightFeedServo;

    private double targetPosition;

    public FeedRamp(SmartServo leftServo, SmartServo rightServo) {
        leftFeedServo = leftServo;
        rightFeedServo = rightServo;
        setTargetPosition(LiveMatchTuning.feedRampDisengagedPosition);
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
        setTargetPosition(LiveMatchTuning.feedRampEngagedPosition);
    }

    public void disengage(){
        setTargetPosition(LiveMatchTuning.feedRampDisengagedPosition);
    }

    public boolean isEngaged(){
        return targetPosition == LiveMatchTuning.feedRampEngagedPosition;
    }

    private double clamp(double value){
        return clamp(value, LiveMatchTuning.feedRampMin, LiveMatchTuning.feedRampMax);
    }

    private double clamp(double value, double min, double max){
        if(value < min) return min;
        return Math.min(value, max);
    }
}
