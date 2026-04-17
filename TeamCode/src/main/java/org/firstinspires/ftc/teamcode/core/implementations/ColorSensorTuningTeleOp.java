package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.hardware.ColorMatchConfig;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.SmartColorSensor;

import java.util.Locale;

@Configurable
@Disabled
@TeleOp(name = "5 - Color Sensor Tuning")
public class ColorSensorTuningTeleOp extends TeleOpCore {
    private SmartColorSensor frontLeftSensor;
    private SmartColorSensor frontRightSensor;
    private SmartColorSensor leftSensor;
    private SmartColorSensor rightSensor;

    private SelectedSensor selectedSensor = SelectedSensor.FRONT_LEFT;
    private ScoringElementColor selectedColor = ScoringElementColor.PURPLE;
    private String saveStatus = "not saved";
    private boolean hasUnsavedChanges = false;

    @Override
    protected void onInitialize() {

        frontLeftSensor = tryGetSensor(ColorMatchConfig.FRONT_SENSOR_NAME);
        frontRightSensor = tryGetSensor(ColorMatchConfig.FRONT_SENSOR_NAME);
        leftSensor = tryGetSensor(ColorMatchConfig.FRONT_SENSOR_NAME);
        rightSensor = tryGetSensor(ColorMatchConfig.FRONT_SENSOR_NAME);

        prettyTelem.addLine("Color Sensor Tuning")
                .addData("Selected Sensor", () -> selectedSensor.name())
                .addData("Selected Color", () -> selectedColor.name())
                .addData("Save Status", () -> saveStatus)
                .addData("Unsaved", () -> hasUnsavedChanges ? "yes" : "no")
                .addData("Selected Gain", this::getSelectedGain)
                .addData("Selected Hue Tol", this::getSelectedHueTolerance)
                .addData("Selected Min Sat", this::getSelectedMinSaturationThreshold)
                .addData("Selected Min Val", this::getSelectedMinValueThreshold)
                .addData("Selected Target Hue", this::getSelectedPresetHue)
                .addData("Selected Preset Min Sat", this::getSelectedPresetMinSaturation)
                .addData("Selected Preset Min Val", this::getSelectedPresetMinValue);

        prettyTelem.addLine("Front Left Sensor")
                .addData("Config Name", () -> ColorMatchConfig.FRONT_SENSOR_NAME)
                .addData("HSV", () -> formatHsv(frontLeftSensor))
                .addData("Detected", () -> detectedColor(frontLeftSensor))
                .addData("Best Match", () -> bestMatch(frontLeftSensor));

        prettyTelem.addLine("Front Right Sensor")
                .addData("Config Name", () -> ColorMatchConfig.FRONT_SENSOR_NAME)
                .addData("HSV", () -> formatHsv(frontRightSensor))
                .addData("Detected", () -> detectedColor(frontRightSensor))
                .addData("Best Match", () -> bestMatch(frontRightSensor));

        prettyTelem.addLine("Left Sensor")
                .addData("Config Name", () -> ColorMatchConfig.FRONT_SENSOR_NAME)
                .addData("HSV", () -> formatHsv(leftSensor))
                .addData("Detected", () -> detectedColor(leftSensor))
                .addData("Best Match", () -> bestMatch(leftSensor));

        prettyTelem.addLine("Right Sensor")
                .addData("Config Name", () -> ColorMatchConfig.FRONT_SENSOR_NAME)
                .addData("HSV", () -> formatHsv(rightSensor))
                .addData("Detected", () -> detectedColor(rightSensor))
                .addData("Best Match", () -> bestMatch(rightSensor));

        prettyTelem.addLine("Controls")
                .addData("A", () -> "Cycle Sensor")
                .addData("X", () -> "Cycle Color (Purple/Green)")
                .addData("Dpad Up/Down", () -> "Target Hue +/-1")
                .addData("Dpad Left/Right", () -> "Hue Tolerance -/+1")
                .addData("LB / RB", () -> "Preset Min Sat -/+0.01")
                .addData("B / Y", () -> "Preset Min Val -/+0.01")
                .addData("Back / Start", () -> "Gain -/+5")
                .addData("Right Stick Button", () -> "Save to persistent storage");
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {
        boolean configChanged = false;

        if (gamepad1.aPressed()) {
            selectedSensor = selectedSensor.next();
        }

        if (gamepad1.xPressed()) {
            selectedColor = selectedColor == ScoringElementColor.PURPLE
                    ? ScoringElementColor.GREEN
                    : ScoringElementColor.PURPLE;
        }

        if (gamepad1.dpadUpPressed()) {
            setSelectedPresetHue(getSelectedPresetHue() + 1f);
            configChanged = true;
        }
        if (gamepad1.dpadDownPressed()) {
            setSelectedPresetHue(getSelectedPresetHue() - 1f);
            configChanged = true;
        }

        if (gamepad1.dpadRightPressed()) {
            setSelectedHueTolerance(getSelectedHueTolerance() + 1f);
            configChanged = true;
        }
        if (gamepad1.dpadLeftPressed()) {
            setSelectedHueTolerance(getSelectedHueTolerance() - 1f);
            configChanged = true;
        }

        if (gamepad1.rightBumperPressed()) {
            setSelectedPresetMinSaturation(getSelectedPresetMinSaturation() + 0.01f);
            configChanged = true;
        }
        if (gamepad1.leftBumperPressed()) {
            setSelectedPresetMinSaturation(getSelectedPresetMinSaturation() - 0.01f);
            configChanged = true;
        }

        if (gamepad1.yPressed()) {
            setSelectedPresetMinValue(getSelectedPresetMinValue() + 0.01f);
            configChanged = true;
        }
        if (gamepad1.bPressed()) {
            setSelectedPresetMinValue(getSelectedPresetMinValue() - 0.01f);
            configChanged = true;
        }

        if (gamepad1.startPressed()) {
            setSelectedGain(getSelectedGain() + 5);
            configChanged = true;
        }
        if (gamepad1.optionPressed()) {
            setSelectedGain(getSelectedGain() - 5);
            configChanged = true;
        }

        if (configChanged) {
            hasUnsavedChanges = true;
            saveStatus = "unsaved changes";
        }

        if (gamepad1.rightStickButtonPressed()) {
            boolean saved = ColorMatchConfig.saveToPersistentStorage();
            if (saved) {
                hasUnsavedChanges = false;
                saveStatus = "saved";
            } else {
                saveStatus = "save failed (storage unavailable)";
            }
        }
    }

    private SmartColorSensor tryGetSensor(String name) {
        try {
            return hardware.getColorSensor(name);
        } catch (Exception e) {
            prettyTelem.warning("Color sensor '" + name + "' unavailable: " + e.getMessage());
            return null;
        }
    }

    private String formatHsv(SmartColorSensor sensor) {
        if (sensor == null) {
            return "n/a";
        }
        float[] hsv = sensor.getHSV();
        return String.format(Locale.US, "(%.1f, %.2f, %.2f)", hsv[0], hsv[1], hsv[2]);
    }

    private String detectedColor(SmartColorSensor sensor) {
        if (sensor == null) {
            return "n/a";
        }
        return sensor.getScoringElementColor().name();
    }

    private String bestMatch(SmartColorSensor sensor) {
        if (sensor == null) {
            return "n/a";
        }
        SmartColorSensor.ColorMatchResult result = sensor.getColorMatchResult();
        return String.format(
                Locale.US,
                "%s (conf=%.2f)",
                result.detectedColor,
                result.confidence
        );
    }

    private int getSelectedGain() {
        switch (selectedSensor) {
            case FRONT_LEFT:
                return ColorMatchConfig.FRONT_GAIN;
            case FRONT_RIGHT:
                return ColorMatchConfig.FRONT_GAIN;
            case LEFT:
                return ColorMatchConfig.FRONT_GAIN;
            case RIGHT:
                return ColorMatchConfig.FRONT_GAIN;
            default:
                return ColorMatchConfig.DEFAULT_GAIN;
        }
    }

    private void setSelectedGain(int value) {
        int clamped = Math.max(1, value);
        switch (selectedSensor) {
            case FRONT_LEFT:
                ColorMatchConfig.FRONT_GAIN = clamped;
                break;
            case FRONT_RIGHT:
                ColorMatchConfig.FRONT_GAIN = clamped;
                break;
            case LEFT:
                ColorMatchConfig.FRONT_GAIN = clamped;
                break;
            case RIGHT:
                ColorMatchConfig.FRONT_GAIN = clamped;
                break;
            default:
                ColorMatchConfig.DEFAULT_GAIN = clamped;
                break;
        }
    }

    private float getSelectedHueTolerance() {
        switch (selectedSensor) {
            case FRONT_LEFT:
                return ColorMatchConfig.FRONT_HUE_TOLERANCE;
            case FRONT_RIGHT:
                return ColorMatchConfig.FRONT_HUE_TOLERANCE;
            case LEFT:
                return ColorMatchConfig.FRONT_HUE_TOLERANCE;
            case RIGHT:
                return ColorMatchConfig.FRONT_HUE_TOLERANCE;
            default:
                return ColorMatchConfig.DEFAULT_HUE_TOLERANCE;
        }
    }

    private void setSelectedHueTolerance(float value) {
        float clamped = Math.max(0.1f, value);
        switch (selectedSensor) {
            case FRONT_LEFT:
                ColorMatchConfig.FRONT_HUE_TOLERANCE = clamped;
                break;
            case FRONT_RIGHT:
                ColorMatchConfig.FRONT_HUE_TOLERANCE = clamped;
                break;
            case LEFT:
                ColorMatchConfig.FRONT_HUE_TOLERANCE = clamped;
                break;
            case RIGHT:
                ColorMatchConfig.FRONT_HUE_TOLERANCE = clamped;
                break;
            default:
                ColorMatchConfig.DEFAULT_HUE_TOLERANCE = clamped;
                break;
        }
    }

    private float getSelectedMinSaturationThreshold() {
        switch (selectedSensor) {
            case FRONT_LEFT:
                return ColorMatchConfig.FRONT_MIN_SATURATION;
            case FRONT_RIGHT:
                return ColorMatchConfig.FRONT_MIN_SATURATION;
            case LEFT:
                return ColorMatchConfig.FRONT_MIN_SATURATION;
            case RIGHT:
                return ColorMatchConfig.FRONT_MIN_SATURATION;
            default:
                return ColorMatchConfig.DEFAULT_MIN_SATURATION;
        }
    }

    private float getSelectedMinValueThreshold() {
        switch (selectedSensor) {
            case FRONT_LEFT:
                return ColorMatchConfig.FRONT_MIN_VALUE;
            case FRONT_RIGHT:
                return ColorMatchConfig.FRONT_MIN_VALUE;
            case LEFT:
                return ColorMatchConfig.FRONT_MIN_VALUE;
            case RIGHT:
                return ColorMatchConfig.FRONT_MIN_VALUE;
            default:
                return ColorMatchConfig.DEFAULT_MIN_VALUE;
        }
    }

    private ColorMatchConfig.ColorPreset getSelectedPreset() {
        switch (selectedSensor) {
            case FRONT_LEFT:
                return selectedColor == ScoringElementColor.PURPLE ? ColorMatchConfig.FRONT_PURPLE : ColorMatchConfig.FRONT_GREEN;
            case FRONT_RIGHT:
                return selectedColor == ScoringElementColor.PURPLE ? ColorMatchConfig.FRONT_PURPLE : ColorMatchConfig.FRONT_GREEN;
            case LEFT:
                return selectedColor == ScoringElementColor.PURPLE ? ColorMatchConfig.FRONT_PURPLE : ColorMatchConfig.FRONT_GREEN;
            case RIGHT:
                return selectedColor == ScoringElementColor.PURPLE ? ColorMatchConfig.FRONT_PURPLE : ColorMatchConfig.FRONT_GREEN;
            default:
                return selectedColor == ScoringElementColor.PURPLE ? ColorMatchConfig.DEFAULT_PURPLE : ColorMatchConfig.DEFAULT_GREEN;
        }
    }

    private float getSelectedPresetHue() {
        return getSelectedPreset().targetHue;
    }

    private void setSelectedPresetHue(float value) {
        getSelectedPreset().targetHue = normalizeHue(value);
    }

    private float getSelectedPresetMinSaturation() {
        return getSelectedPreset().minSaturation;
    }

    private void setSelectedPresetMinSaturation(float value) {
        getSelectedPreset().minSaturation = clamp01(value);
    }

    private float getSelectedPresetMinValue() {
        return getSelectedPreset().minValue;
    }

    private void setSelectedPresetMinValue(float value) {
        getSelectedPreset().minValue = clamp01(value);
    }

    private static float clamp01(float value) {
        return Math.max(0f, Math.min(1f, value));
    }

    private static float normalizeHue(float hue) {
        float normalized = hue % 360f;
        if (normalized < 0f) {
            normalized += 360f;
        }
        return normalized;
    }

    private enum SelectedSensor {
        FRONT_LEFT,
        FRONT_RIGHT,
        LEFT,
        RIGHT;

        private SelectedSensor next() {
            SelectedSensor[] values = values();
            return values[(ordinal() + 1) % values.length];
        }
    }
}

