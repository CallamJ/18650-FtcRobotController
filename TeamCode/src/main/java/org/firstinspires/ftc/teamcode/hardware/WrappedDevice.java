package org.firstinspires.ftc.teamcode.hardware;

/**
 * Marker for smart wrappers that expose the underlying FTC SDK device.
 *
 * @param <T> wrapped device type
 */
public interface WrappedDevice<T> {
    T getRaw();
}
