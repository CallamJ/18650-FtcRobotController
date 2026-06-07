package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;
import org.firstinspires.ftc.teamcode.utilities.LiveMatchTuning;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;

public class SmartColorSensor extends Device implements NormalizedColorSensor, Caching, ScoringColorSensor, WrappedDevice<NormalizedColorSensor> {
    private final NormalizedColorSensor colorSensor;
    private final HardwareCache<NormalizedRGBA> colorCache;
    private final ColorMatchConfig.ColorMatchProfile colorProfile;
    private DataFilter hueFilter = DataFilter.NONE;
    private DataFilter saturationFilter = DataFilter.NONE;
    private DataFilter valueFilter = DataFilter.NONE;
    private int appliedHueFilterWindow = Integer.MIN_VALUE;
    private int appliedSaturationFilterWindow = Integer.MIN_VALUE;
    private int appliedValueFilterWindow = Integer.MIN_VALUE;
    private int appliedGain = Integer.MIN_VALUE;

    SmartColorSensor(NormalizedColorSensor colorSensor, String configName) {
        this(colorSensor, configName, ColorMatchConfig.frontProfile());
    }

    SmartColorSensor(
            NormalizedColorSensor colorSensor,
            String configName,
            ColorMatchConfig.ColorMatchProfile colorProfile
    ) {
        super(configName);
        this.colorSensor = colorSensor;
        this.colorProfile = colorProfile == null ? ColorMatchConfig.frontProfile() : colorProfile;
        this.colorCache = new HardwareCache<>(colorSensor::getNormalizedColors);
        syncConfiguredGain();
    }

    @Override
    public NormalizedColorSensor getRaw() {
        return colorSensor;
    }

    /**
     * Reads the current color and returns it as HSV values.
     * @return array containing the Hue, Saturation and Value floats in that order.
     */
    public float[] getHSV() {
        syncConfiguredGain();
        syncConfiguredFilters();
        float[] hsv = new float[3];
        Color.colorToHSV(getNormalizedColors().toColor(), hsv);
        hsv[0] = normalizeHue((float) hueFilter.compute(hsv[0]));
        hsv[1] = clamp01((float) saturationFilter.compute(hsv[1]));
        hsv[2] = clamp01((float) valueFilter.compute(hsv[2]));
        return hsv;
    }

    public void setHsvFilters(DataFilter hueFilter, DataFilter saturationFilter, DataFilter valueFilter) {
        this.hueFilter = hueFilter == null ? DataFilter.NONE : hueFilter;
        this.saturationFilter = saturationFilter == null ? DataFilter.NONE : saturationFilter;
        this.valueFilter = valueFilter == null ? DataFilter.NONE : valueFilter;
        this.appliedHueFilterWindow = Integer.MIN_VALUE;
        this.appliedSaturationFilterWindow = Integer.MIN_VALUE;
        this.appliedValueFilterWindow = Integer.MIN_VALUE;
    }

    public void setHsvRollingAverageWindow(int window) {
        if (window <= 0) {
            setHsvFilters(DataFilter.NONE, DataFilter.NONE, DataFilter.NONE);
            return;
        }
        setHsvFilters(
                new RollingAverage(window),
                new RollingAverage(window),
                new RollingAverage(window)
        );
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
        return matchScoringElementColor(colorProfile, hsv[0], hsv[1], hsv[2]);
    }

    /**
     * Gets detailed color matching information for debugging and tuning.
     *
     * @return ColorMatchResult containing the detected color, HSV values, and match confidence
     */
    @Override
    public ColorMatchResult getColorMatchResult() {
        float[] hsv = getHSV();
        return getColorMatchResult(colorProfile, hsv[0], hsv[1], hsv[2]);
    }

    /**
     * Runs configured scoring-element color matching for arbitrary HSV data.
     *
     * @param profile DI-injected profile used for matching
     * @param hue hue in 0-360 range (values outside range are normalized)
     * @param saturation saturation in 0-1 range
     * @param value value/brightness in 0-1 range
     * @return matched scoring element color or NONE when thresholds/matching fail
     */
    public static @NonNull ScoringElementColor matchScoringElementColor(
            ColorMatchConfig.ColorMatchProfile profile,
            float hue,
            float saturation,
            float value
    ) {
        return getColorMatchResult(profile, hue, saturation, value).detectedColor;
    }

    /**
     * Returns detailed matching result for arbitrary HSV input.
     *
     * @param profile DI-injected profile used for matching
     * @param hue hue in 0-360 range (values outside range are normalized)
     * @param saturation saturation in 0-1 range
     * @param value value/brightness in 0-1 range
     * @return scoring match details and confidence
     */
    public static ColorMatchResult getColorMatchResult(
            ColorMatchConfig.ColorMatchProfile profile,
            float hue,
            float saturation,
            float value
    ) {
        ColorMatchConfig.ColorMatchProfile safeProfile = profile == null
                ? ColorMatchConfig.frontProfile()
                : profile;
        float minSaturationThreshold = safeProfile.minSaturation();
        float minValueThreshold = safeProfile.minValue();
        float hueTolerance = safeProfile.hueTolerance();
        ColorMatchConfig.ColorPreset[] activePresets = safeProfile.activePresets();

        hue = normalizeHue(hue);

        if (saturation < minSaturationThreshold || value < minValueThreshold) {
            return new ColorMatchResult(ScoringElementColor.NONE, hue, saturation, value, 0.0f);
        }

        ColorMatchConfig.ColorPreset bestMatch = null;
        float bestConfidence = 0.0f;

        for (ColorMatchConfig.ColorPreset preset : activePresets) {
            if (preset.matches(hue, saturation, value, hueTolerance)) {
                float confidence = preset.getMatchConfidence(hue, saturation, value, hueTolerance);
                if (confidence > bestConfidence) {
                    bestConfidence = confidence;
                    bestMatch = preset;
                }
            }
        }

        ScoringElementColor detectedColor = bestMatch != null ? bestMatch.color : ScoringElementColor.NONE;
        return new ColorMatchResult(detectedColor, hue, saturation, value, bestConfidence);
    }

    public void setGain(float gain){
        if(gain <= 0)
            throw new IllegalArgumentException("Gain must be positive");

        colorSensor.setGain(gain);
        appliedGain = (int) gain;
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

    private void syncConfiguredGain() {
        int configuredGain = Math.max(1, colorProfile.gain());
        if (configuredGain != appliedGain) {
            colorSensor.setGain(configuredGain);
            appliedGain = configuredGain;
        }
    }

    private void syncConfiguredFilters() {
        int hueWindow = Math.max(0, LiveMatchTuning.colorSensorHueFilterWindow);
        int saturationWindow = Math.max(0, LiveMatchTuning.colorSensorSaturationFilterWindow);
        int valueWindow = Math.max(0, LiveMatchTuning.colorSensorValueFilterWindow);

        if (hueWindow != appliedHueFilterWindow) {
            hueFilter = hueWindow == 0 ? DataFilter.NONE : new RollingAverage(hueWindow);
            appliedHueFilterWindow = hueWindow;
        }
        if (saturationWindow != appliedSaturationFilterWindow) {
            saturationFilter = saturationWindow == 0 ? DataFilter.NONE : new RollingAverage(saturationWindow);
            appliedSaturationFilterWindow = saturationWindow;
        }
        if (valueWindow != appliedValueFilterWindow) {
            valueFilter = valueWindow == 0 ? DataFilter.NONE : new RollingAverage(valueWindow);
            appliedValueFilterWindow = valueWindow;
        }
    }

    private static float normalizeHue(float hue) {
        float normalized = hue % 360f;
        if (normalized < 0f) {
            normalized += 360f;
        }
        return normalized;
    }

    private static float clamp01(float value) {
        return Math.max(0f, Math.min(1f, value));
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
