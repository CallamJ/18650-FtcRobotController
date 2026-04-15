package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;

/**
 * Front color sensor profile used by all scoring color matching.
 */
@Configurable
public class ColorMatchConfig {
    private static final String PERSISTENCE_KEY = "color_match_profiles_front_v2";
    private static boolean persistedConfigLoaded = false;
    public static boolean ENABLE_PERSISTED_CONFIG = false;

    // FTC hardware config name for the active front sensor profile.
    public static String FRONT_SENSOR_NAME = "frontColorSensorLeft";

    // Shared profile values.
    public static float FRONT_HUE_TOLERANCE = 40.0f;
    public static float FRONT_MIN_VALUE = 0.15f;
    public static float FRONT_MIN_SATURATION = 0f;
    public static int FRONT_GAIN = 150;

    // Shared presets.
    public static ColorPreset FRONT_PURPLE = new ColorPreset(ScoringElementColor.PURPLE, 295.0f, 0f, 0.3f);
    public static ColorPreset FRONT_GREEN = new ColorPreset(ScoringElementColor.GREEN, 145.0f, 0f, 0.15f);
    public static ColorPreset[] FRONT_ACTIVE_PRESETS = {FRONT_PURPLE, FRONT_GREEN};

    // Backward-compatible defaults for older callers.
    public static float DEFAULT_HUE_TOLERANCE = FRONT_HUE_TOLERANCE;
    public static float DEFAULT_MIN_VALUE = FRONT_MIN_VALUE;
    public static float DEFAULT_MIN_SATURATION = FRONT_MIN_SATURATION;
    public static int DEFAULT_GAIN = FRONT_GAIN;
    public static ColorPreset DEFAULT_PURPLE = FRONT_PURPLE;
    public static ColorPreset DEFAULT_GREEN = FRONT_GREEN;
    public static ColorPreset[] DEFAULT_ACTIVE_PRESETS = FRONT_ACTIVE_PRESETS;
    private static final ColorMatchProfile FRONT_PROFILE = new ColorMatchProfile() {
        @Override
        public int gain() {
            ensureLoadedFromPersistentStorage();
            return FRONT_GAIN;
        }

        @Override
        public float hueTolerance() {
            ensureLoadedFromPersistentStorage();
            return FRONT_HUE_TOLERANCE;
        }

        @Override
        public float minSaturation() {
            ensureLoadedFromPersistentStorage();
            return FRONT_MIN_SATURATION;
        }

        @Override
        public float minValue() {
            ensureLoadedFromPersistentStorage();
            return FRONT_MIN_VALUE;
        }

        @Override
        public ColorPreset[] activePresets() {
            ensureLoadedFromPersistentStorage();
            if (FRONT_ACTIVE_PRESETS == null || FRONT_ACTIVE_PRESETS.length == 0) {
                return new ColorPreset[]{FRONT_PURPLE, FRONT_GREEN};
            }
            return FRONT_ACTIVE_PRESETS;
        }
    };

    public static ColorMatchProfile frontProfile() {
        return FRONT_PROFILE;
    }

    /**
     * Loads persisted color configuration once per process when storage is ready.
     */
    public static synchronized void ensureLoadedFromPersistentStorage() {
        if (!ENABLE_PERSISTED_CONFIG || persistedConfigLoaded || !PersistentStorage.isInitialized()) {
            return;
        }

        try {
            PersistedColorConfig persisted = PersistentStorage.getObject(PERSISTENCE_KEY, PersistedColorConfig.class);
            if (persisted != null) {
                applyPersistedConfig(persisted);
            }
        } catch (Exception ignored) {
            // Keep in-memory defaults if persistent storage fails.
        }

        persistedConfigLoaded = true;
    }

    /**
     * Persists the current in-memory color configuration so it becomes the default on future runs.
     */
    public static synchronized boolean saveToPersistentStorage() {
        if (!PersistentStorage.isInitialized()) {
            return false;
        }
        PersistentStorage.saveObject(PERSISTENCE_KEY, snapshotPersistedConfig());
        persistedConfigLoaded = true;
        return true;
    }

    private static PersistedColorConfig snapshotPersistedConfig() {
        PersistedColorConfig config = new PersistedColorConfig();
        config.front = snapshotSensorProfile(
                FRONT_GAIN,
                FRONT_HUE_TOLERANCE,
                FRONT_MIN_SATURATION,
                FRONT_MIN_VALUE,
                FRONT_GREEN,
                FRONT_PURPLE
        );
        return config;
    }

    private static PersistedSensorProfile snapshotSensorProfile(
            int gain,
            float hueTolerance,
            float minSaturation,
            float minValue,
            ColorPreset green,
            ColorPreset purple
    ) {
        PersistedSensorProfile profile = new PersistedSensorProfile();
        profile.gain = gain;
        profile.hueTolerance = hueTolerance;
        profile.minSaturation = minSaturation;
        profile.minValue = minValue;
        profile.green = snapshotPreset(green);
        profile.purple = snapshotPreset(purple);
        return profile;
    }

    private static PersistedPreset snapshotPreset(ColorPreset preset) {
        PersistedPreset persistedPreset = new PersistedPreset();
        persistedPreset.targetHue = preset.targetHue;
        persistedPreset.minSaturation = preset.minSaturation;
        persistedPreset.minValue = preset.minValue;
        return persistedPreset;
    }

    private static void applyPersistedConfig(PersistedColorConfig persisted) {
        applySensorProfile(persisted.front);
    }

    private static void applySensorProfile(PersistedSensorProfile persisted) {
        if (persisted == null) {
            return;
        }

        FRONT_GAIN = persisted.gain > 0 ? persisted.gain : FRONT_GAIN;
        FRONT_HUE_TOLERANCE = sanitizePositive(persisted.hueTolerance, FRONT_HUE_TOLERANCE);
        FRONT_MIN_SATURATION = sanitizeUnit(persisted.minSaturation, FRONT_MIN_SATURATION);
        FRONT_MIN_VALUE = sanitizeUnit(persisted.minValue, FRONT_MIN_VALUE);

        applyPreset(persisted.green, FRONT_GREEN);
        applyPreset(persisted.purple, FRONT_PURPLE);
        FRONT_ACTIVE_PRESETS = new ColorPreset[]{FRONT_PURPLE, FRONT_GREEN};

        DEFAULT_GAIN = FRONT_GAIN;
        DEFAULT_HUE_TOLERANCE = FRONT_HUE_TOLERANCE;
        DEFAULT_MIN_SATURATION = FRONT_MIN_SATURATION;
        DEFAULT_MIN_VALUE = FRONT_MIN_VALUE;
        DEFAULT_ACTIVE_PRESETS = FRONT_ACTIVE_PRESETS;
    }

    private static void applyPreset(PersistedPreset persisted, ColorPreset target) {
        if (persisted == null || target == null) {
            return;
        }
        target.targetHue = normalizeHue(sanitizeFinite(persisted.targetHue, target.targetHue));
        target.minSaturation = sanitizeUnit(persisted.minSaturation, target.minSaturation);
        target.minValue = sanitizeUnit(persisted.minValue, target.minValue);
    }

    private static float sanitizeFinite(float value, float fallback) {
        return (Float.isNaN(value) || Float.isInfinite(value)) ? fallback : value;
    }

    private static float sanitizePositive(float value, float fallback) {
        float finite = sanitizeFinite(value, fallback);
        return finite > 0 ? finite : fallback;
    }

    private static float sanitizeUnit(float value, float fallback) {
        float finite = sanitizeFinite(value, fallback);
        if (finite < 0f || finite > 1f) {
            return fallback;
        }
        return finite;
    }

    private static float normalizeHue(float hue) {
        float normalized = hue % 360f;
        if (normalized < 0f) {
            normalized += 360f;
        }
        return normalized;
    }

    public static class PersistedColorConfig {
        public PersistedSensorProfile front;
    }

    public static class PersistedSensorProfile {
        public int gain;
        public float hueTolerance;
        public float minSaturation;
        public float minValue;
        public PersistedPreset green;
        public PersistedPreset purple;
    }

    public static class PersistedPreset {
        public float targetHue;
        public float minSaturation;
        public float minValue;
    }

    public interface ColorMatchProfile {
        int gain();
        float hueTolerance();
        float minSaturation();
        float minValue();
        ColorPreset[] activePresets();
    }

    public static class ColorPreset {
        public ScoringElementColor color;
        public float targetHue;
        public float minSaturation;
        public float minValue;

        public ColorPreset(ScoringElementColor color, float targetHue, float minSaturation, float minValue) {
            this.color = color;
            this.targetHue = targetHue;
            this.minSaturation = minSaturation;
            this.minValue = minValue;
        }

        public boolean matches(float hue, float saturation, float value, float hueTolerance) {
            if (saturation < minSaturation || value < minValue) {
                return false;
            }
            return isHueWithinTolerance(hue, targetHue, hueTolerance);
        }

        public float getMatchConfidence(float hue, float saturation, float value, float hueTolerance) {
            if (!matches(hue, saturation, value, hueTolerance)) {
                return 0.0f;
            }

            float hueDiff = Math.abs(normalizeHueDifference(hue - targetHue));
            float safeHueTolerance = Math.max(0.001f, hueTolerance);
            float hueScore = 1.0f - (hueDiff / safeHueTolerance);
            float satScore = minSaturation <= 0f ? 1.0f : Math.min(saturation / minSaturation, 1.0f);
            float valScore = minValue <= 0f ? 1.0f : Math.min(value / minValue, 1.0f);
            return (hueScore * 0.7f) + (satScore * 0.15f) + (valScore * 0.15f);
        }

        private static boolean isHueWithinTolerance(float hue, float targetHue, float tolerance) {
            float diff = normalizeHueDifference(hue - targetHue);
            return Math.abs(diff) <= tolerance;
        }

        private static float normalizeHueDifference(float diff) {
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;
            return diff;
        }
    }
}
