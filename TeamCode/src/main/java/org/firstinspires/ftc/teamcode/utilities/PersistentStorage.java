package org.firstinspires.ftc.teamcode.utilities;

import android.content.Context;
import android.content.SharedPreferences;
import com.google.gson.Gson;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.reflect.Type;
import java.util.Set;

/**
 * A utility class for persistent data storage using SharedPreferences and Gson.
 * Supports saving and retrieving primitive types, strings, sets, and most objects.
 * This class is used statically and requires initialization in the OpMode.
 */
public class PersistentStorage {
    private static final String PREFS_NAME = "FTC_Persistent_Storage";
    private static SharedPreferences prefs;
    private static SharedPreferences.Editor editor;
    private static final Gson gson = new Gson();

    /**
     * Initializes the storage system. Must be called once in OpMode before usage.
     *
     * @param hardwareMap The HardwareMap from the OpMode, used to get application context.
     */
    public static void init(HardwareMap hardwareMap) {
        if (prefs == null) {
            Context context = hardwareMap.appContext;
            prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
            editor = prefs.edit();
        }
    }

    /**
     * Saves an integer value.
     *
     * @param key   The key for storage.
     * @param value The integer value to save.
     */
    public static void saveInt(String key, int value) {
        editor.putInt(key, value).apply();
    }

    /**
     * Saves a double value by converting it to JSON.
     *
     * @param key   The key for storage.
     * @param value The double value to save.
     */
    public static void saveDouble(String key, double value) {
        editor.putString(key, gson.toJson(value)).apply();
    }

    /**
     * Saves a boolean value.
     *
     * @param key   The key for storage.
     * @param value The boolean value to save.
     */
    public static void saveBoolean(String key, boolean value) {
        editor.putBoolean(key, value).apply();
    }

    /**
     * Saves a string value.
     *
     * @param key   The key for storage.
     * @param value The string value to save.
     */
    public static void saveString(String key, String value) {
        editor.putString(key, value).apply();
    }

    /**
     * Saves a set of strings.
     *
     * @param key   The key for storage.
     * @param value The set of strings to save.
     */
    public static void saveStringSet(String key, Set<String> value) {
        editor.putStringSet(key, value).apply();
    }

    /**
     * Retrieves an integer value.
     *
     * @param key          The key for retrieval.
     * @param defaultValue The default value if the key is not found.
     * @return The stored integer value or the default value.
     */
    public static int getInt(String key, int defaultValue) {
        return prefs.getInt(key, defaultValue);
    }

    /**
     * Retrieves a double value.
     *
     * @param key          The key for retrieval.
     * @param defaultValue The default value if the key is not found.
     * @return The stored double value or the default value.
     */
    public static double getDouble(String key, double defaultValue) {
        String json = prefs.getString(key, gson.toJson(defaultValue));
        return gson.fromJson(json, Double.class);
    }

    /**
     * Retrieves a boolean value.
     *
     * @param key          The key for retrieval.
     * @param defaultValue The default value if the key is not found.
     * @return The stored boolean value or the default value.
     */
    public static boolean getBoolean(String key, boolean defaultValue) {
        return prefs.getBoolean(key, defaultValue);
    }

    /**
     * Retrieves a string value.
     *
     * @param key          The key for retrieval.
     * @param defaultValue The default value if the key is not found.
     * @return The stored string value or the default value.
     */
    public static String getString(String key, String defaultValue) {
        return prefs.getString(key, defaultValue);
    }

    /**
     * Retrieves a set of strings.
     *
     * @param key          The key for retrieval.
     * @param defaultValue The default value if the key is not found.
     * @return The stored set of strings or the default value.
     */
    public static Set<String> getStringSet(String key, Set<String> defaultValue) {
        return prefs.getStringSet(key, defaultValue);
    }

    /**
     * Saves an object by converting it to JSON.
     *
     * @param key    The key for storage.
     * @param object The object to save.
     */
    public static void saveObject(String key, Object object) {
        editor.putString(key, gson.toJson(object)).apply();
    }

    /**
     * Retrieves an object of the specified class type.
     *
     * @param key       The key for retrieval.
     * @param classType The class type of the stored object.
     * @param <T>       The type of object to retrieve.
     * @return The stored object or null if not found.
     */
    public static <T> T getObject(String key, Class<T> classType) {
        String json = prefs.getString(key, null);
        return json == null ? null : gson.fromJson(json, classType);
    }

    /**
     * Retrieves an object with a complex type (e.g., List, Map) using TypeToken.
     *
     * @param key      The key for retrieval.
     * @param typeOfT  The Type of the stored object.
     * @param <T>      The type of object to retrieve.
     * @return The stored object or null if not found.
     */
    public static <T> T getObject(String key, Type typeOfT) {
        String json = prefs.getString(key, null);
        return json == null ? null : gson.fromJson(json, typeOfT);
    }

    /**
     * Removes a specific key and its associated value.
     *
     * @param key The key to remove.
     */
    public static void remove(String key) {
        editor.remove(key).apply();
    }

    /**
     * Clears all stored values from the persistent storage.
     */
    public static void clear() {
        editor.clear().apply();
    }
}
