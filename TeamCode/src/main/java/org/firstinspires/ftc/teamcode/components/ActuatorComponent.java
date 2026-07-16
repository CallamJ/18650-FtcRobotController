package org.firstinspires.ftc.teamcode.components;

/**
 * Base type for components that need periodic updates to drive hardware toward a commanded state.
 */
public abstract class ActuatorComponent {
    public abstract void tick();

    public void stop() {
    }

    public double getPower() {
        return 0;
    }
}
