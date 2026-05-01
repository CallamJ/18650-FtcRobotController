package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;

import java.util.ArrayDeque;
import java.util.Arrays;

/**
 * Maintains a rolling window of samples and computes percentiles on demand.
 */
public class RollingPercentileWindow implements DataFilter {
    private int maxSize;
    private double trackedPercentile;
    private final ArrayDeque<Double> values;

    public RollingPercentileWindow(int maxSize) {
        this(maxSize, 50);
    }

    public RollingPercentileWindow(int maxSize, double trackedPercentile) {
        if (maxSize <= 0) {
            throw new IllegalArgumentException("maxSize must be greater than 0");
        }
        this.maxSize = maxSize;
        this.trackedPercentile = trackedPercentile;
        this.values = new ArrayDeque<>(maxSize);
    }

    public void add(double value) {
        if (values.size() >= maxSize) {
            values.pollFirst();
        }
        values.addLast(value);
    }

    public int size() {
        return values.size();
    }

    public int getMaxSize() {
        return maxSize;
    }

    public double getTrackedPercentile() {
        return trackedPercentile;
    }

    public void setTrackedPercentile(double trackedPercentile) {
        this.trackedPercentile = trackedPercentile;
    }

    public void setMaxSize(int newMaxSize) {
        if (newMaxSize <= 0) {
            throw new IllegalArgumentException("maxSize must be greater than 0");
        }
        this.maxSize = newMaxSize;
        while (values.size() > maxSize) {
            values.pollFirst();
        }
    }

    @Override
    public double compute(double newValue) {
        add(newValue);
        return getPercentile(trackedPercentile);
    }

    public double getPercentile(double percentile) {
        if (values.isEmpty()) {
            return 0.0;
        }
        return percentileFromSorted(sortedSnapshot(), percentile);
    }

    public double[] getPercentiles(double... percentiles) {
        double[] result = new double[percentiles.length];
        if (values.isEmpty()) {
            return result;
        }
        double[] sorted = sortedSnapshot();
        for (int i = 0; i < percentiles.length; i++) {
            result[i] = percentileFromSorted(sorted, percentiles[i]);
        }
        return result;
    }

    private double[] sortedSnapshot() {
        double[] sorted = new double[values.size()];
        int index = 0;
        for (double value : values) {
            sorted[index++] = value;
        }
        Arrays.sort(sorted);
        return sorted;
    }

    private static double percentileFromSorted(double[] sorted, double percentile) {
        if (percentile <= 0) {
            return sorted[0];
        }
        if (percentile >= 100) {
            return sorted[sorted.length - 1];
        }

        double rank = (percentile / 100.0) * (sorted.length - 1);
        int lowerIndex = (int) Math.floor(rank);
        int upperIndex = (int) Math.ceil(rank);
        if (lowerIndex == upperIndex) {
            return sorted[lowerIndex];
        }

        double lowerValue = sorted[lowerIndex];
        double upperValue = sorted[upperIndex];
        double blend = rank - lowerIndex;
        return lowerValue + ((upperValue - lowerValue) * blend);
    }

    public double getMax() {
        if (values.isEmpty()) {
            return 0.0;
        }

        double max = Double.NEGATIVE_INFINITY;
        for (double value : values) {
            if (value > max) {
                max = value;
            }
        }
        return max;
    }
}
