package org.firstinspires.ftc.teamcode.hardware;

public class SmartLEDIndicator extends Device {
    public enum IndicatorColor {
        OFF(OFF_POSITION),
        RED(RED_POSITION),
        ORANGE(ORANGE_POSITION),
        YELLOW(YELLOW_POSITION),
        SAGE(SAGE_POSITION),
        GREEN(GREEN_POSITION),
        AZURE(AZURE_POSITION),
        BLUE(BLUE_POSITION),
        INDIGO(INDIGO_POSITION),
        VIOLET(VIOLET_POSITION),
        WHITE(WHITE_POSITION);

        private final double position;

        IndicatorColor(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    private static final double OFF_POSITION = 0.0;
    private static final double RED_POSITION = 0.285;
    private static final double ORANGE_POSITION = 0.333;
    private static final double YELLOW_POSITION = 0.388;
    private static final double SAGE_POSITION = 0.444;
    private static final double GREEN_POSITION = 0.500;
    private static final double AZURE_POSITION = 0.555;
    private static final double BLUE_POSITION = 0.611;
    private static final double INDIGO_POSITION = 0.666;
    private static final double VIOLET_POSITION = 0.722;
    private static final double WHITE_POSITION = 1.0;

    private static final double OFF_THRESHOLD_MICROS = 1100.0;
    private static final double WHITE_THRESHOLD_MICROS = 1900.0;
    private static final double MIN_MICROS = 500.0;
    private static final double MAX_MICROS = 2500.0;
    private static final double SUPPORTED_HUE_MIN = 0.0;
    private static final double SUPPORTED_HUE_MAX = 270.0;

    private final SmartServo servo;

    SmartLEDIndicator(SmartServo servo) {
        super(servo.getConfigName());
        this.servo = servo;
    }

    public void setColor(IndicatorColor color) {
        if (color == null) {
            throw new IllegalArgumentException("color cannot be null");
        }
        setPosition(color.getPosition());
    }

    public void off() {
        setColor(IndicatorColor.OFF);
    }

    public void white() {
        setColor(IndicatorColor.WHITE);
    }

    /**
     * Sets a continuous value on the supported red->violet gradient.
     * 0 maps to red and 1 maps to violet.
     */
    public void setSpectrum(double value) {
        validateFinite(value, "value");
        double clamped = clamp(value, 0.0, 1.0);
        setPosition(interpolate(RED_POSITION, VIOLET_POSITION, clamped));
    }

    /**
     * Sets color by hue where 0=red and 270=violet over this LED's supported band.
     * Values between 270 and 360 map to the nearest supported endpoint.
     */
    public void setHue(double hueDegrees) {
        validateFinite(hueDegrees, "hueDegrees");
        double normalizedHue = normalizeHue(hueDegrees);

        if (normalizedHue > SUPPORTED_HUE_MAX) {
            double distToRed = 360.0 - normalizedHue;
            double distToViolet = normalizedHue - SUPPORTED_HUE_MAX;
            normalizedHue = distToRed <= distToViolet ? SUPPORTED_HUE_MIN : SUPPORTED_HUE_MAX;
        }

        setSpectrum(normalizedHue / SUPPORTED_HUE_MAX);
    }

    /**
     * Applies the product PWM behavior directly:
     * below 1100us = OFF, above 1900us = WHITE, in-between = gradient.
     */
    public void setPulseWidthMicros(double micros) {
        validateFinite(micros, "micros");

        if (micros < OFF_THRESHOLD_MICROS) {
            off();
            return;
        }

        if (micros > WHITE_THRESHOLD_MICROS) {
            white();
            return;
        }

        double spectrum = (micros - OFF_THRESHOLD_MICROS) / (WHITE_THRESHOLD_MICROS - OFF_THRESHOLD_MICROS);
        setSpectrum(spectrum);
    }

    public SmartServo getServo() {
        return servo;
    }

    /**
     * Returns the commanded position from the underlying servo API.
     * FTC SDK servo position is command-state, not physical angle feedback.
     */
    public double getPosition() {
        return servo.getPosition();
    }

    public double getPulseWidthMicros() {
        double position = getPosition();

        if (position <= OFF_POSITION) {
            return MIN_MICROS;
        }

        if (position >= WHITE_POSITION) {
            return MAX_MICROS;
        }

        if (position <= RED_POSITION) {
            return OFF_THRESHOLD_MICROS;
        }

        if (position >= VIOLET_POSITION) {
            return WHITE_THRESHOLD_MICROS;
        }

        double spectrum = (position - RED_POSITION) / (VIOLET_POSITION - RED_POSITION);
        return interpolate(OFF_THRESHOLD_MICROS, WHITE_THRESHOLD_MICROS, spectrum);
    }

    /**
     * Returns the current value on the supported red->violet gradient in [0,1].
     */
    public double getSpectrum() {
        double position = getPosition();

        if (position <= RED_POSITION) {
            return 0.0;
        }

        if (position >= VIOLET_POSITION) {
            return 1.0;
        }

        return (position - RED_POSITION) / (VIOLET_POSITION - RED_POSITION);
    }

    /**
     * Returns the equivalent supported hue (0..270) based on the current commanded position.
     */
    public double getHue() {
        return getSpectrum() * SUPPORTED_HUE_MAX;
    }

    /**
     * Returns the nearest named color for the current commanded position.
     */
    public IndicatorColor getColor() {
        double position = getPosition();

        IndicatorColor nearest = IndicatorColor.OFF;
        double nearestDistance = Math.abs(position - OFF_POSITION);

        for (IndicatorColor color : IndicatorColor.values()) {
            double dist = Math.abs(position - getPositionForColor(color));
            if (dist < nearestDistance) {
                nearestDistance = dist;
                nearest = color;
            }
        }

        return nearest;
    }

    public double getMinPulseWidthMicros() {
        return MIN_MICROS;
    }

    public double getMaxPulseWidthMicros() {
        return MAX_MICROS;
    }

    public double getOffThresholdMicros() {
        return OFF_THRESHOLD_MICROS;
    }

    public double getWhiteThresholdMicros() {
        return WHITE_THRESHOLD_MICROS;
    }

    private void setPosition(double position) {
        servo.setPosition(clamp(position, 0.0, 1.0));
    }

    private static void validateFinite(double value, String argumentName) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(argumentName + " must be finite");
        }
    }

    private static double normalizeHue(double hueDegrees) {
        double normalized = hueDegrees % 360.0;
        if (normalized < 0.0) {
            normalized += 360.0;
        }
        return normalized;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double interpolate(double min, double max, double t) {
        return min + ((max - min) * t);
    }

    private static double getPositionForColor(IndicatorColor color) {
        if (color == null) {
            throw new IllegalArgumentException("color cannot be null");
        }
        return color.getPosition();
    }
}
