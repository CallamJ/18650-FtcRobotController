package org.firstinspires.ftc.teamcode.hardware.controllers;

/**
 * Control algorithm that operates directly on measured position values.
 */
public interface PositionControlAlgorithm extends ControlAlgorithm {
    double calcPosition(double targetPosition, double actualPosition);
}
