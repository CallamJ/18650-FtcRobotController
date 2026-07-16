package org.firstinspires.ftc.teamcode.hardware.controllers;

/**
 * Control algorithm that operates directly on measured velocity values.
 */
public interface VelocityControlAlgorithm extends ControlAlgorithm {
    double calcVelocity(double targetVelocity, double actualVelocity);
}
