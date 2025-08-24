package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class SmartTouchSensor extends Device implements TouchSensor, Caching {

    private final TouchSensor touchSensor;

    private final HardwareCache<Boolean> pressingCache;
    private final HardwareCache<Double> valueCache;

    SmartTouchSensor(TouchSensor touchSensor, String configName){
        super(configName);
        this.touchSensor = touchSensor;
        pressingCache = new HardwareCache<>(touchSensor::isPressed);
        valueCache = new HardwareCache<>(touchSensor::getValue);
    }

    /**
     * Represents how much force is applied to the touch sensor; for some touch sensors
     * this value will only ever be 0 or 1.
     *
     * @return a number between 0 and 1
     */
    @Override
    public double getValue() {
        return valueCache.read();
    }

    /**
     * Return true if the touch sensor is being pressed
     *
     * @return true if the touch sensor is being pressed
     */
    @Override
    public boolean isPressed() {
        return pressingCache.read();
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return touchSensor.getManufacturer();
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
        return touchSensor.getDeviceName();
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return touchSensor.getConnectionInfo();
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion() {
        return touchSensor.getVersion();
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        touchSensor.resetDeviceConfigurationForOpMode();
    }

    /**
     * Closes this device
     */
    @Override
    public void close() {
        touchSensor.close();
    }

    /**
     *
     */
    @Override
    public void invalidateCache() {
        pressingCache.invalidateCache();
        valueCache.invalidateCache();
    }

    /**
     *
     */
    @Override
    public void updateCache() {
        pressingCache.updateCache();
        valueCache.updateCache();
    }

    @Override
    public void setStrategy(Strategy strategy) {
        pressingCache.setStrategy(strategy);
        valueCache.setStrategy(strategy);
    }

    @Override
    public Strategy getStrategy() {
        return pressingCache.getStrategy();
    }
}
