package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;
import androidx.annotation.NonNull;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;

@Configurable
public class SmartColorSensor extends Device implements NormalizedColorSensor, Caching {

    NormalizedColorSensor colorSensor;
    HardwareCache<NormalizedRGBA> colorCache;

    SmartColorSensor(NormalizedColorSensor colorSensor, String configName) {
        super(configName);
        this.colorSensor = colorSensor;
        colorCache = new HardwareCache<>(colorSensor::getNormalizedColors);
        colorSensor.setGain(ColorMatchConfig.GAIN);
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
     * Uses the configurable color matching system from ColorMatchConfig.
     *
     * @return the approximate color detected by the sensor. If no scoring element color is detected returns ScoringElementColor.NONE.
     */
    public @NonNull ScoringElementColor getScoringElementColor() {
        float[] hsv = getHSV();
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // Ensure minimum valid saturation and value (not black/white/gray)
        if (saturation < ColorMatchConfig.MIN_SATURATION || value < ColorMatchConfig.MIN_VALUE) {
            return ScoringElementColor.NONE;
        }

        // Normalize hue to 0-360 range
        hue = hue % 360;
        if (hue < 0) hue += 360;

        // Find the best matching preset color
        ColorMatchConfig.ColorPreset bestMatch = null;
        float bestConfidence = 0.0f;

        for (ColorMatchConfig.ColorPreset preset : ColorMatchConfig.ACTIVE_PRESETS) {
            if (preset.matches(hue, saturation, value)) {
                float confidence = preset.getMatchConfidence(hue, saturation, value);
                if (confidence > bestConfidence) {
                    bestConfidence = confidence;
                    bestMatch = preset;
                }
            }
        }

        return bestMatch != null ? bestMatch.color : ScoringElementColor.NONE;
    }

    /**
     * Gets detailed color matching information for debugging and tuning.
     *
     * @return ColorMatchResult containing the detected color, HSV values, and match confidence
     */
    public ColorMatchResult getColorMatchResult() {
        float[] hsv = getHSV();
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // Normalize hue
        hue = hue % 360;
        if (hue < 0) hue += 360;

        // Find the best matching preset color
        ColorMatchConfig.ColorPreset bestMatch = null;
        float bestConfidence = 0.0f;

        for (ColorMatchConfig.ColorPreset preset : ColorMatchConfig.ACTIVE_PRESETS) {
            float confidence = preset.getMatchConfidence(hue, saturation, value);
            if (confidence > bestConfidence) {
                bestConfidence = confidence;
                bestMatch = preset;
            }
        }

        ScoringElementColor detectedColor = (bestMatch != null && bestConfidence > 0)
                ? bestMatch.color
                : ScoringElementColor.NONE;

        return new ColorMatchResult(detectedColor, hue, saturation, value, bestConfidence);
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

    /**
     * Data class containing detailed color matching results.
     */
    public static class ColorMatchResult {
        public final ScoringElementColor detectedColor;
        public final float hue;
        public final float saturation;
        public final float value;
        public final float confidence;

        public ColorMatchResult(ScoringElementColor detectedColor, float hue, float saturation, float value, float confidence) {
            this.detectedColor = detectedColor;
            this.hue = hue;
            this.saturation = saturation;
            this.value = value;
            this.confidence = confidence;
        }

        @NotNull
        @NonNull
        @Override
        public String toString() {
            return String.format("Color: %s, HSV: (%.1f, %.2f, %.2f), Confidence: %.2f",
                    detectedColor, hue, saturation, value, confidence);
        }
    }
}