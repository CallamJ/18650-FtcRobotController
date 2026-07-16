package org.firstinspires.ftc.teamcode.components.subsystems;

import org.firstinspires.ftc.teamcode.components.mechanisms.FeedRamp;
import org.firstinspires.ftc.teamcode.components.mechanisms.FeedWheels;

public class FeedSystem {
    private final FeedWheels wheels;
    private final FeedRamp ramp;

    public FeedSystem(FeedWheels wheels, FeedRamp ramp) {
        this.wheels = wheels;
        this.ramp = ramp;
    }

    public void startFeeding() {
        wheels.start();
        ramp.engage();
    }

    public void stopFeeding() {
        wheels.stop();
        ramp.disengage();
    }

    public boolean isFeeding() {
        return wheels.isRunning() && ramp.isEngaged();
    }

    public void toggleFeeding() {
        if (isFeeding()) {
            stopFeeding();
        } else {
            startFeeding();
        }
    }
}
