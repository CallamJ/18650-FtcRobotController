package org.firstinspires.ftc.teamcode.hardware.controllers;

import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.Notifier;

public interface ControlAlgorithm {
    Notifier getNoLongerBusyNotifier();

    boolean isBusy();

    double result();

    void setTolerance(double tolerance);

    double getTolerance();

    void setDirection(Direction direction);

    Direction getDirection();
}
