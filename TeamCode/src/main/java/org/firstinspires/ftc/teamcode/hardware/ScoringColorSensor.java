package org.firstinspires.ftc.teamcode.hardware;

public interface ScoringColorSensor {
    float[] getHSV();
    ScoringElementColor getScoringElementColor();
    SmartColorSensor.ColorMatchResult getColorMatchResult();

    default String getClosestColorMatchName() {
        return "N/A";
    }
}
