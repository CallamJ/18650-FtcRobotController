package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;
import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;
import org.firstinspires.ftc.teamcode.utilities.LiveMatchTuning;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.Arrays;

/**
 * Wraps a SmartCamera + PredominantColorProcessor so a camera can be used like a color sensor.
 * Provides both the SDK swatch match and SmartColorSensor HSV-based scoring color match.
 */
public class SmartCameraColorSensor extends Device implements AutoCloseable, ScoringColorSensor {
    private static final PredominantColorProcessor.Swatch[] DEFAULT_SWATCHES = {
            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE
    };
    private static final ImageRegion DEFAULT_ROI = ImageRegion.asUnityCenterCoordinates(-0.6, 0.6, 0.6, -0.6);

    private final SmartCamera camera;
    private final ColorMatchConfig.ColorMatchProfile colorProfile;
    private final PredominantColorProcessor processor;
    private final VisionPortal visionPortal;
    private DataFilter hueFilter = DataFilter.NONE;
    private DataFilter saturationFilter = DataFilter.NONE;
    private DataFilter valueFilter = DataFilter.NONE;
    private int appliedHueFilterWindow = Integer.MIN_VALUE;
    private int appliedSaturationFilterWindow = Integer.MIN_VALUE;
    private int appliedValueFilterWindow = Integer.MIN_VALUE;

    SmartCameraColorSensor(SmartCamera camera, ColorMatchConfig.ColorMatchProfile colorProfile) {
        this(camera, colorProfile, DEFAULT_ROI, DEFAULT_SWATCHES);
    }

    SmartCameraColorSensor(
            SmartCamera camera,
            ColorMatchConfig.ColorMatchProfile colorProfile,
            ImageRegion roi,
            PredominantColorProcessor.Swatch... swatches
    ) {
        super(requireCameraConfigName(camera));
        if (camera == null) {
            throw new IllegalArgumentException("camera cannot be null");
        }
        if (roi == null) {
            throw new IllegalArgumentException("roi cannot be null");
        }
        if (swatches == null || swatches.length == 0) {
            throw new IllegalArgumentException("at least one swatch is required");
        }

        this.camera = camera;
        this.colorProfile = colorProfile == null ? ColorMatchConfig.frontProfile() : colorProfile;
        this.processor = new PredominantColorProcessor.Builder()
                .setRoi(roi)
                .setSwatches(swatches.clone())
                .build();

        Size resolution = camera.getResolution();
        this.visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCameraResolution(resolution)
                .setCamera(camera.getRaw())
                .build();
    }

    /**
     * @return camera color reading containing swatch match plus raw and normalized HSV values.
     */
    public CameraColorReading getReading() {
        syncConfiguredFilters();
        PredominantColorProcessor.Result analysis = processor.getAnalysis();

        int[] rgb = analysis.RGB.clone();
        int[] hsvOpenCv = analysis.HSV.clone();
        int[] yCrCb = analysis.YCrCb.clone();

        float hue360 = normalizeHue360(hsvOpenCv[0] * 2f);
        float saturation01 = clamp01(hsvOpenCv[1] / 255f);
        float value01 = clamp01(hsvOpenCv[2] / 255f);
        hue360 = normalizeHue360((float) hueFilter.compute(hue360));
        saturation01 = clamp01((float) saturationFilter.compute(saturation01));
        value01 = clamp01((float) valueFilter.compute(value01));

        SmartColorSensor.ColorMatchResult match = SmartColorSensor.getColorMatchResult(
                colorProfile,
                hue360,
                saturation01,
                value01
        );

        return new CameraColorReading(
                analysis.closestSwatch,
                rgb,
                hsvOpenCv,
                yCrCb,
                hue360,
                saturation01,
                value01,
                match
        );
    }

    @Override
    public float[] getHSV() {
        CameraColorReading reading = getReading();
        return new float[]{reading.hue360, reading.saturation01, reading.value01};
    }

    @Override
    public ScoringElementColor getScoringElementColor() {
        return getReading().scoringMatch.detectedColor;
    }

    @Override
    public SmartColorSensor.ColorMatchResult getColorMatchResult() {
        return getReading().scoringMatch;
    }

    @Override
    public String getClosestColorMatchName() {
        CameraColorReading reading = getReading();
        return reading.closestSwatch == null ? "NONE" : reading.closestSwatch.name();
    }

    public PredominantColorProcessor getProcessor() {
        return processor;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public SmartCamera getCamera() {
        return camera;
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

    @Override
    public void close() {
        visionPortal.close();
    }

    private void syncConfiguredFilters() {
        int hueWindow = Math.max(0, LiveMatchTuning.cameraColorSensorHueFilterWindow);
        int saturationWindow = Math.max(0, LiveMatchTuning.cameraColorSensorSaturationFilterWindow);
        int valueWindow = Math.max(0, LiveMatchTuning.cameraColorSensorValueFilterWindow);

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

    private static float normalizeHue360(float hue) {
        float normalized = hue % 360f;
        if (normalized < 0f) {
            normalized += 360f;
        }
        return normalized;
    }

    private static float clamp01(float value) {
        return Math.max(0f, Math.min(1f, value));
    }

    private static String requireCameraConfigName(SmartCamera camera) {
        if (camera == null) {
            throw new IllegalArgumentException("camera cannot be null");
        }
        return camera.getConfigName() + "_cameraColorSensor";
    }

    public static class CameraColorReading {
        public final PredominantColorProcessor.Swatch closestSwatch;
        public final int[] rgb;
        public final int[] hsvOpenCv;
        public final int[] yCrCb;
        public final float hue360;
        public final float saturation01;
        public final float value01;
        public final SmartColorSensor.ColorMatchResult scoringMatch;

        public CameraColorReading(
                PredominantColorProcessor.Swatch closestSwatch,
                int[] rgb,
                int[] hsvOpenCv,
                int[] yCrCb,
                float hue360,
                float saturation01,
                float value01,
                SmartColorSensor.ColorMatchResult scoringMatch
        ) {
            this.closestSwatch = closestSwatch;
            this.rgb = rgb;
            this.hsvOpenCv = hsvOpenCv;
            this.yCrCb = yCrCb;
            this.hue360 = hue360;
            this.saturation01 = saturation01;
            this.value01 = value01;
            this.scoringMatch = scoringMatch;
        }

        @NonNull
        @Override
        public String toString() {
            return "Swatch=" + closestSwatch
                    + ", scoring=" + scoringMatch.detectedColor
                    + ", HSV(opencv)=" + Arrays.toString(hsvOpenCv)
                    + ", HSV(normalized)=(" + hue360 + ", " + saturation01 + ", " + value01 + ")";
        }
    }
}
