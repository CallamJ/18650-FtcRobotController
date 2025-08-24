package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class SmartServo extends Device implements Servo, Caching {
    private final Servo baseServo;

    HardwareCache<Double> positionCache;

    SmartServo(Servo baseServo, String configName){
        super(configName);
        this.baseServo = baseServo;
        positionCache = new HardwareCache<>(baseServo::getPosition);
    }

    /**
     * Returns the underlying servo controller on which this servo is situated.
     *
     * @return the underlying servo controller on which this servo is situated.
     * @see #getPortNumber()
     */
    @Override
    public ServoController getController() {
        return baseServo.getController();
    }

    /**
     * Returns the port number on the underlying servo controller on which this motor is situated.
     *
     * @return the port number on the underlying servo controller on which this motor is situated.
     * @see #getController()
     */
    @Override
    public int getPortNumber() {
        return baseServo.getPortNumber();
    }

    /**
     * Sets the logical direction in which this servo operates.
     *
     * @param direction the direction to set for this servo
     * @see #getDirection()
     * @see Direction
     */
    @Override
    public void setDirection(Direction direction) {
        baseServo.setDirection(direction);
    }

    /**
     * Returns the current logical direction in which this servo is set as operating.
     *
     * @return the current logical direction in which this servo is set as operating.
     * @see #setDirection(Direction)
     */
    @Override
    public Direction getDirection() {
        return baseServo.getDirection();
    }

    /**
     * Sets the current position of the servo, expressed as a fraction of its available
     * range. If PWM power is enabled for the servo, the servo will attempt to move to
     * the indicated position.
     *
     * @param position the position to which the servo should move, a value in the range [0.0, 1.0]
     * @see ServoController#pwmEnable()
     * @see #getPosition()
     */
    @Override
    public void setPosition(double position) {
        baseServo.setPosition(position);
    }

    /**
     * Returns the position to which the servo was last commanded to move. Note that this method
     * does NOT read a position from the servo through any electrical means, as no such electrical
     * mechanism is, generally, available.
     *
     * @return the position to which the servo was last commanded to move, or Double.NaN
     * if no such position is known
     * @see #setPosition(double)
     * @see Double#NaN
     * @see Double#isNaN()
     */
    @Override
    public double getPosition() {
        return positionCache.read();
    }

    /**
     * Scales the available movement range of the servo to be a subset of its maximum range. Subsequent
     * positioning calls will operate within that subset range. This is useful if your servo has
     * only a limited useful range of movement due to the physical hardware that it is manipulating
     * (as is often the case) but you don't want to have to manually scale and adjust the input
     * to {@link #setPosition(double) setPosition()} each time.
     *
     * <p>For example, if scaleRange(0.2, 0.8) is set; then servo positions will be
     * scaled to fit in that range:<br>
     * setPosition(0.0) scales to 0.2<br>
     * setPosition(1.0) scales to 0.8<br>
     * setPosition(0.5) scales to 0.5<br>
     * setPosition(0.25) scales to 0.35<br>
     * setPosition(0.75) scales to 0.65<br>
     * </p>
     *
     * <p>Note the parameters passed here are relative to the underlying full range of motion of
     * the servo, not its currently scaled range, if any. Thus, scaleRange(0.0, 1.0) will reset
     * the servo to its full range of movement.</p>
     *
     * @param min the lower limit of the servo movement range, a value in the interval [0.0, 1.0]
     * @param max the upper limit of the servo movement range, a value in the interval [0.0, 1.0]
     * @see #setPosition(double)
     */
    @Override
    public void scaleRange(double min, double max) {
        baseServo.scaleRange(min, max);
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return baseServo.getManufacturer();
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return baseServo.getDeviceName();
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return baseServo.getConnectionInfo();
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion() {
        return baseServo.getVersion();
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        baseServo.resetDeviceConfigurationForOpMode();
    }

    /**
     * Closes this device
     */
    @Override
    public void close() {
        baseServo.close();
    }

    /**
     *
     */
    @Override
    public void invalidateCache() {
        positionCache.invalidateCache();
    }

    /**
     *
     */
    @Override
    public void updateCache() {
        positionCache.updateCache();
    }

    /**
     * @param strategy
     */
    @Override
    public void setStrategy(Strategy strategy) {
        positionCache.setStrategy(strategy);
    }

    /**
     * @return
     */
    @Override
    public Strategy getStrategy() {
        return positionCache.getStrategy();
    }
}
