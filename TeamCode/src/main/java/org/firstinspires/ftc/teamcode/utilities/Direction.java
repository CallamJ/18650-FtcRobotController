package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * An enumeration representing the two possible directions: FORWARD and REVERSE.
 * This class provides methods for converting between custom `Direction` values and the corresponding `Encoder.Direction` values.
 */
public enum Direction {
    FORWARD, REVERSE;


    public DcMotorSimple.Direction toMotorDirection(){
        return this == FORWARD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
    }
}
