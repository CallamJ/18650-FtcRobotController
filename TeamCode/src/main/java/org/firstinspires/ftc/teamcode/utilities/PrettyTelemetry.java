package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * A lightweight, pre-configured wrapper for the FTC Telemetry system that only supports supplier-based updating,
 * with no clearing. This wrapper improves readability by incorporating automatic rounding and HTML formatting.
 *
 * <h5>Features:</h5>
 * <ul>
 * <li>Automatic rounding of numerical values to a specified precision.</li>
 * <li>HTML-based formatting for better visual clarity.</li>
 * <li>Supports telemetry for both the driver station and an optional FTC Dashboard instance.</li>
 * <li>Provides an intelligent logging system for easy debugging.</li>
 * </ul>
 */

public class PrettyTelemetry {
    private double roundingPlaces;

    private final Telemetry telemetry;
    private Telemetry panelsTelemetry;

    //todo: Integrate this better with the Panels native telemetry. we probably do not need this special adaptation for Panels like we did with RR
    private final List<String> dashValueCaptions = new ArrayList<>();
    private final List<Func<?>> dashValueProducers = new ArrayList<>();
    private final Line logLine;
    private String logs = "<i>There are no logs yet...</i>";

    /**
     * Constructs a `PrettyTelemetry` instance using the provided `Telemetry` object.
     *
     * @param telemetry The FTC `Telemetry` instance.
     */
    public PrettyTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
        this.roundingPlaces = 3;
        telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator("");

        this.logLine = addLine("Logs");
        logLine.addData("Log Entries", () -> logs);
    }

    /**
     * Constructs a PrettyTelemetry instance with support for both driver station telemetry and Panels telemetry.
     *
     * @param opmodeTelemetry       The FTC Telemetry instance for the driver station.
     * @param panelsTelemetry The FTC Telemetry instance for the dashboard.
     */
    public PrettyTelemetry(Telemetry opmodeTelemetry, Telemetry panelsTelemetry){
        this(opmodeTelemetry);
        this.panelsTelemetry = panelsTelemetry;
    }

    /**
     * Checks if the telemetry instance has an associated FTC Dashboard telemetry.
     *
     * @return `true` if Panels telemetry is available, otherwise `false`.
     */
    public boolean hasDashboard(){
        return panelsTelemetry != null;
    }

    /**
     * Adds a telemetry data entry to the FTC Dashboard.
     *
     * @param caption       The label for the data entry.
     * @param valueProducer A function that supplies the value dynamically.
     * @param <T>           The type of value being provided.
     */
    public <T> void addDataToDashboard(String caption, Func<T> valueProducer){
        dashValueCaptions.add(caption);
        dashValueProducers.add(wrapFunc(valueProducer));
    }

    /**
     * Retrieves the number of decimal places used for rounding numerical values.
     *
     * @return The current rounding precision.
     */
    public double getRoundingPlaces() {
        return roundingPlaces;
    }

    /**
     * Sets the number of decimal places to which numerical values should be rounded.
     *
     * @param roundingPlaces The desired precision for rounding.
     */
    public void setRoundingPlaces(double roundingPlaces) {
        this.roundingPlaces = roundingPlaces;
    }

    /**
     * Represents a formatted telemetry line that allows for structured logging.
     */
    public static class Line {
        Telemetry.Line line;
        private Line(Telemetry.Line line){
            this.line = line;
        }

        /**
         * Adds a formatted telemetry data entry to the current line.
         *
         * @param caption       The label for the data entry.
         * @param valueProducer A function that supplies the value dynamically.
         * @param <T>           The type of value being provided.
         * @return A reference to the newly created `Item`.
         */
        public <T> Item addData(String caption, Func<T> valueProducer){
            return new Item(line.addData("<br>- " + caption, wrapFunc(valueProducer)));
        }
    }

    /**
     * Represents an individual telemetry data item within a line.
     */
    public static class Item {
        Telemetry.Item item;
        private Item(Telemetry.Item item){
            this.item = item;
        }

        /**
         * Adds additional telemetry data to the current item.
         *
         * @param caption       The label for the additional data entry.
         * @param valueProducer A function that supplies the value dynamically.
         * @param <T>           The type of value being provided.
         * @return A reference to the newly created `Item`.
         */
        public <T> Item addData(String caption, Func<T> valueProducer){
            return new Item(item.addData("<br>- " + caption, wrapFunc(valueProducer)));
        }
    }

    /**
     * Adds a new formatted telemetry line with a bold caption.
     *
     * @param caption The title of the telemetry line.
     * @return A `Line` object representing the newly added telemetry line.
     */
    public PrettyTelemetry.Line addLine(String caption){
        return new Line(telemetry.addLine("<br><b>" + caption + "</b>"));
    }


    /**
     * Adds a new empty telemetry line.
     *
     * @return A `Line` object representing the newly added empty telemetry line.
     */
    public PrettyTelemetry.Line addLine(){
        return new Line(telemetry.addLine());
    }

    /**
     * Adds a formatted telemetry data entry to the telemetry system.
     *
     * @param caption       The label for the data entry.
     * @param valueProducer A function that supplies the value dynamically.
     * @param <T>           The type of value being provided.
     * @return A reference to the newly created `Item`.
     */
    public <T> Item addData(String caption, Func<T> valueProducer){
        return new Item(telemetry.addData("<br>- " + caption, wrapFunc(valueProducer)));
    }

    public enum LogLevel {
        DEBUG, INFO, WARNING, ERROR
    }

    // Default minimum log level to display
    private LogLevel minLogLevel = LogLevel.INFO;

    // Store a limited number of recent log messages for display
    private final int MAX_LOG_ENTRIES = 50;
    private final List<LogEntry> logEntries = new ArrayList<>();
    private boolean showLogsInTelemetry = true;

    // Time-based filtering (in milliseconds)
    private Long logTimeWindow = null; // null means no time filtering

    // Log entry class to store each log message with metadata
    public static class LogEntry {
        final long timestamp;
        final LogLevel level;
        final String message;

        LogEntry(LogLevel level, String message) {
            this.timestamp = System.currentTimeMillis();
            this.level = level;
            this.message = message;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format("[%s] %s", level.toString(), message);
        }

        public String toDetailedString() {
            // Format timestamp as HH:MM:SS.mmm for better readability
            SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault());
            String formattedTime = sdf.format(new Date(timestamp));
            return String.format("%s [%s] %s", formattedTime, level.toString(), message);
        }
    }

    /**
     * Logs a message at the specified level
     *
     * @param level The log level
     * @param message The message to log
     */
    public void log(LogLevel level, String message) {
        // Add to our log entries
        logEntries.add(new LogEntry(level, message));

        // Maintain max size
        if (logEntries.size() > MAX_LOG_ENTRIES) {
            logEntries.remove(0);
        }
    }

    public void debug(String message) { log(LogLevel.DEBUG, message); }
    public void info(String message) { log(LogLevel.INFO, message); }
    public void warning(String message) { log(LogLevel.WARNING, message); }
    public void error(String message) { log(LogLevel.ERROR, message); }

    /**
     * Set the minimum log level to display
     */
    public void setMinLogLevel(LogLevel level) {
        this.minLogLevel = level;
    }

    /**
     * Toggle whether logs should be displayed in telemetry
     */
    public void setShowLogsInTelemetry(boolean show) {
        this.showLogsInTelemetry = show;
    }

    /**
     * Set a time window for filtering logs (only show logs from the last X milliseconds)
     * @param timeWindowMs The time window in milliseconds, or null to disable time filtering
     */
    public void setLogTimeWindow(Long timeWindowMs) {
        this.logTimeWindow = timeWindowMs;
    }

    /**
     * Clear all stored logs
     */
    public void clearLogs() {
        logEntries.clear();
    }

    /**
     * Check if a log entry should be displayed based on level and time filters
     */
    private boolean shouldShowLogEntry(LogEntry entry) {
        // Check level filter
        if (entry.level.ordinal() < minLogLevel.ordinal()) {
            return false;
        }

        // Check time filter if enabled
        if (logTimeWindow != null) {
            long currentTime = System.currentTimeMillis();
            if (currentTime - entry.timestamp > logTimeWindow) {
                return false;
            }
        }

        return true;
    }

    /**
     * Get the current log entries as a single string, applying filters
     */
    public String getLogsAsString() {
        StringBuilder sb = new StringBuilder();
        for (LogEntry entry : logEntries) {
            if (shouldShowLogEntry(entry)) {
                sb.append(entry.toDetailedString()).append("\n");
            }
        }
        return sb.toString();
    }

    /**
     * Get filtered logs
     * @return List of log entries that pass current filters
     */
    public List<LogEntry> getFilteredLogs() {
        List<LogEntry> filtered = new ArrayList<>();
        for (LogEntry entry : logEntries) {
            if (shouldShowLogEntry(entry)) {
                filtered.add(entry);
            }
        }
        return filtered;
    }

    /**
     * Updates the telemetry outputs for both the driver station and, if available, the FTC Dashboard.
     */
    public void update() {
        if (showLogsInTelemetry && !logEntries.isEmpty()) {

//            // Time window info if enabled
//            if (logTimeWindow != null) {
//                logLine.addData("Time Window", () -> "Last " + (logTimeWindow / 1000) + " seconds");
//            }

            // Get filtered logs
            List<LogEntry> filteredLogs = getFilteredLogs();

            int count = 0;
            StringBuilder builder = new StringBuilder();
            for (int i = filteredLogs.size() - 1; i >= 0 && count < MAX_LOG_ENTRIES; i--) {
                LogEntry entry = filteredLogs.get(i);
                String color;
                switch (entry.level) {
                    case ERROR:
                        color = "red";
                        break;
                    case WARNING:
                        color = "orange";
                        break;
                    case INFO:
                        color = "white";
                        break;
                    default:
                        color = "gray";
                }

                SimpleDateFormat sdf = new SimpleDateFormat("mm:ss.SSS", Locale.getDefault());
                String time = sdf.format(new Date(filteredLogs.get(i).timestamp));
                builder.append("<br><font color='" + color + "'>[" + time + "] " + filteredLogs.get(i).toString() + "</font>");
                count++;
            }

            logs = builder.toString();



            // Show log count info
//            int totalFilteredLogs = filteredLogs.size();
//            if (totalFilteredLogs > MAX_LOG_ENTRIES) {
//                logLine.addData("", () -> "<i>Showing " + MAX_LOG_ENTRIES + " of " + totalFilteredLogs + " logs</i>");
//            }
        }else if(!showLogsInTelemetry){
            logs = "<i>Hidden</i>";
        }else {
            logs = "<i>There are no logs yet...</i>";
        }

        telemetry.update();

        for (int i = 0; i < dashValueCaptions.size(); i++) {
            panelsTelemetry.addData(dashValueCaptions.get(i), dashValueProducers.get(i).value());
        }

        panelsTelemetry.update();
    }


    /**
     * Wraps a value-producing function to apply rounding where applicable.
     *
     * @param valueProducer The function that supplies the value.
     * @param <T>           The type of value being provided.
     * @return A wrapped `Func<?>` that applies rounding to numerical values.
     */
    public static <T> Func<?> wrapFunc(Func<T> valueProducer){
        T value = valueProducer.value();
        if(value instanceof Double){
	        //noinspection unchecked
	        Func<Double> doubleProducer = (Func<Double>) valueProducer;
            return () -> roundToPrecision(doubleProducer.value(), 3);
        } else if (value instanceof Float) {
            //noinspection unchecked
            Func<Float> floatProducer = (Func<Float>) valueProducer;
            return () -> roundToPrecision(floatProducer.value(), 3);
        }else{
            return valueProducer;
        }
    }

    /**
     * Rounds a numerical value to the specified number of decimal places.
     *
     * @param value         The value to round.
     * @param decimalPlaces The number of decimal places to retain.
     * @return The rounded value.
     * @throws IllegalArgumentException if decimalPlaces is negative.
     */
    public static double roundToPrecision(double value, int decimalPlaces) {
        if (decimalPlaces < 0) throw new IllegalArgumentException("decimalPlaces must be non-negative");

        double factor = Math.pow(10, decimalPlaces);
        return Math.round(value * factor) / factor;
    }
}
