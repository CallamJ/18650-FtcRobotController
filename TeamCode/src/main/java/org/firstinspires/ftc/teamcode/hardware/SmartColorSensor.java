package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class SmartColorSensor extends Device implements NormalizedColorSensor, Caching {
    //configuration
    public static float HUE_THRESHOLD = 20.0f;
    public static float RED_HUE = 25.0f;
    public static float YELLOW_HUE = 75.0f;
    public static float BLUE_HUE = 215.0f;
    public static float GREEN_HUE = 155.0f;
    public static float PURPLE_HUE = 220.0f;
    public static float MIN_VALUE = 0.15f;
    public static float MIN_SATURATION = 0.2f;

    public static int GAIN = 150;

    NormalizedColorSensor colorSensor;

    HardwareCache<NormalizedRGBA> colorCache;

    SmartColorSensor(NormalizedColorSensor colorSensor, String configName) {
        super(configName);
        this.colorSensor = colorSensor;
        colorCache = new HardwareCache<>(colorSensor::getNormalizedColors);
        colorSensor.setGain(GAIN);
    }

    /**
     * Reads the current color and returns it as HSV values.
     * @return array containing the Hue, Saturation and Value floats in that order.
     */
    public float[] getHSV() {
        float[] hsv = new float[3];
        Color.colorToHSV(getNormalizedColors().toColor(), hsv);
        return hsv;
    }

    /**
     * @param distanceUnit The unit the distance should be returned in
     * @return distance to the closest obstruction directly in front of the color sensor
     * @throws IllegalStateException if this color sensor does not support distance sensing. You can check this programmatically by calling hasDistanceSensing().
     */
    public double getDistance(DistanceUnit distanceUnit){
        if (colorSensor instanceof DistanceSensor) {
            return ((DistanceSensor) colorSensor).getDistance(distanceUnit);
        }else {
            throw new IllegalStateException("Color sensor is not an instance of DistanceSensor, this color sensor likely doesn't support distance sensing.");
        }
    }

    /**
     * @implNote if this method returns false on a color sensor object, then calling getDistance() on that object will throw an IllegalStateException
     * @return whether this color sensor supports distance sensing.
     */
    public boolean hasDistanceSensing(){
        return colorSensor instanceof DistanceSensor;
    }

    /**
     * Reads the currently detected color and returns a scoring element color or null if no scoring element color was detected.
     *
     * @return the approximate color detected by the sensor. If no scoring element color is detected returns ScoringElementColor.NONE.
     */
    public @NonNull ScoringElementColor getScoringElementColor() {
        float[] hsv = getHSV();

        float hue = hsv[0];
        final float saturation = hsv[1];
        final float value = hsv[2];

        // Ensure valid saturation and value
        if (saturation < MIN_SATURATION || value < MIN_VALUE) {
            return ScoringElementColor.NONE; // Very low saturation or brightness, return None
        }

        // Normalize hue
        hue = hue % 360;
        if (hue < 0) hue += 360;

        // Check closeness to each color
        if (isWithinThreshold(hue, PURPLE_HUE)) {
            return ScoringElementColor.PURPLE;
        } else if (isWithinThreshold(hue, GREEN_HUE)) {
            return ScoringElementColor.GREEN;
        } else {
            return ScoringElementColor.NONE;
        }
    }

    private static boolean isWithinThreshold(float hue, float targetHue) {
        return Math.abs(hue - targetHue) <= HUE_THRESHOLD;
    }


    public void setGain(float gain){
        if(gain <= 0)
            throw new IllegalArgumentException("Gain must be positive");

        colorSensor.setGain(gain);
    }

    /**
     * Reads the colors from the sensor
     *
     * @return the current set of colors from the sensor
     */
    @Override
    public NormalizedRGBA getNormalizedColors() {
        return colorCache.read();
    }

    public float getGain(){
        return colorSensor.getGain();
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return colorSensor.getManufacturer();
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
        return colorSensor.getDeviceName();
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return colorSensor.getConnectionInfo();
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion() {
        return colorSensor.getVersion();
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        colorSensor.resetDeviceConfigurationForOpMode();
    }

    /**
     * Closes this device
     */
    @Override
    public void close() {
        colorSensor.close();
    }

    /**
     *
     */
    @Override
    public void invalidateCache() {
        colorCache.invalidateCache();
    }

    /**
     *
     */
    @Override
    public void updateCache() {
        colorCache.updateCache();
    }

    /**
     * @param strategy
     */
    @Override
    public void setStrategy(Strategy strategy) {
        colorCache.setStrategy(strategy);
    }

    /**
     * @return
     */
    @Override
    public Strategy getStrategy() {
        return colorCache.getStrategy();
    }
}