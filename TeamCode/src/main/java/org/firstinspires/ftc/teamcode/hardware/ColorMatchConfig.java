package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration class for color matching.
 * Allows defining preset colors with their target HSV values and matching tolerance.
 */
@Configurable
public class ColorMatchConfig {
    // Default tolerance for color matching
    public static float HUE_TOLERANCE = 20.0f;
    public static float SATURATION_TOLERANCE = 0.3f;
    public static float VALUE_TOLERANCE = 0.3f;

    // Minimum thresholds to consider a color valid (not black/white/gray)
    public static float MIN_VALUE = 0.15f;
    public static float MIN_SATURATION = 0.2f;

    // Sensor gain
    public static int GAIN = 150;

    // Preset color definitions (H, S, V)
    public static final ColorPreset PURPLE = new ColorPreset(
            ScoringElementColor.PURPLE,
            180.0f,  // Hue
            0.3f,    // Min Saturation
            0.18f     // Min Value
    );

    public static final ColorPreset GREEN = new ColorPreset(
            ScoringElementColor.GREEN,
            155.0f,  // Hue
            0.5f,    // Min Saturation
            0.18f     // Min Value
    );

    public static final ColorPreset RED = new ColorPreset(
            ScoringElementColor.RED,
            25.0f,   // Hue
            0.5f,    // Min Saturation
            0.3f     // Min Value
    );

    public static final ColorPreset YELLOW = new ColorPreset(
            ScoringElementColor.YELLOW,
            75.0f,   // Hue
            0.5f,    // Min Saturation
            0.3f     // Min Value
    );

    public static final ColorPreset BLUE = new ColorPreset(
            ScoringElementColor.BLUE,
            215.0f,  // Hue
            0.5f,    // Min Saturation
            0.3f     // Min Value
    );

    // Array of all presets to match against (customize which colors to detect)
    public static ColorPreset[] ACTIVE_PRESETS = {
            PURPLE,
            GREEN
    };

    /**
     * Represents a preset color with target HSV values.
     */
    public static class ColorPreset {
        public final ScoringElementColor color;
        public final float targetHue;
        public final float minSaturation;
        public final float minValue;

        public ColorPreset(ScoringElementColor color, float targetHue, float minSaturation, float minValue) {
            this.color = color;
            this.targetHue = targetHue;
            this.minSaturation = minSaturation;
            this.minValue = minValue;
        }

        /**
         * Checks if the given HSV values match this preset color.
         *
         * @param hue Hue value (0-360)
         * @param saturation Saturation value (0-1)
         * @param value Value/brightness (0-1)
         * @return true if the color matches this preset within tolerance
         */
        public boolean matches(float hue, float saturation, float value) {
            // Check minimum saturation and value thresholds
            if (saturation < minSaturation || value < minValue) {
                return false;
            }

            // Check hue match with tolerance
            return isHueWithinTolerance(hue, targetHue, HUE_TOLERANCE);
        }

        /**
         * Calculates match confidence (0-1) for the given HSV values.
         * Higher values indicate better matches.
         *
         * @param hue Hue value (0-360)
         * @param saturation Saturation value (0-1)
         * @param value Value/brightness (0-1)
         * @return confidence score from 0 to 1
         */
        public float getMatchConfidence(float hue, float saturation, float value) {
            if (!matches(hue, saturation, value)) {
                return 0.0f;
            }

            // Calculate hue difference (accounting for circular nature)
            float hueDiff = Math.abs(normalizeHueDifference(hue - targetHue));
            float hueScore = 1.0f - (hueDiff / HUE_TOLERANCE);

            // Calculate saturation score
            float satScore = Math.min(saturation / minSaturation, 1.0f);

            // Calculate value score
            float valScore = Math.min(value / minValue, 1.0f);

            // Weighted average (hue is most important)
            return (hueScore * 0.7f) + (satScore * 0.15f) + (valScore * 0.15f);
        }

        private static boolean isHueWithinTolerance(float hue, float targetHue, float tolerance) {
            float diff = normalizeHueDifference(hue - targetHue);
            return Math.abs(diff) <= tolerance;
        }

        private static float normalizeHueDifference(float diff) {
            // Normalize difference to -180 to 180 range (accounting for circular hue)
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;
            return diff;
        }
    }
}