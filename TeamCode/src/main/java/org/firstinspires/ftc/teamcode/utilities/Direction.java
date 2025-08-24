package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.drive.roadrunner.util.Encoder;

/**
 * An enumeration representing the two possible directions: FORWARD and REVERSE.
 * This class provides methods for converting between custom `Direction` values and the corresponding `Encoder.Direction` values.
 */
public enum Direction {
    FORWARD, REVERSE;

    /**
     * Converts a RoadRunner `Encoder.Direction` to a custom `Direction` value.
     *
     * @param theirs The `Encoder.Direction` from RoadRunner to be converted.
     * @return The corresponding `Direction` value (either `FORWARD` or `REVERSE`).
     */
    public static Direction of(Encoder.Direction theirs){
        return theirs == Encoder.Direction.FORWARD ? FORWARD : REVERSE;
    }

    /**
     * Converts the custom `Direction` to the corresponding RoadRunner `Encoder.Direction` value.
     *
     * @return The corresponding `Encoder.Direction` value (either `FORWARD` or `REVERSE`).
     */
    public Encoder.Direction toRR(){
        return this == FORWARD ? Encoder.Direction.FORWARD : Encoder.Direction.REVERSE;
    }
}
