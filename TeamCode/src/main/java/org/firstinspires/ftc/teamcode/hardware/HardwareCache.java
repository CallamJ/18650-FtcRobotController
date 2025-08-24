package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Supplier;

/**
 * A generic caching mechanism for hardware values, reducing redundant hardware reads.
 * This class supports different caching strategies to optimize performance.
 *
 * @param <T> The type of value being cached.
 */
public class HardwareCache<T> implements Caching {
    private T cachedValue;
    private boolean cacheValid = false;
    private Strategy strategy = Strategy.UPDATE_WHEN_INVALIDATED;
    private final Supplier<T> valueSupplier;
    private boolean cacheRead = false;

    /**
     * Constructs a HardwareCache with a given value supplier.
     *
     * @param valueSupplier A function that retrieves the latest hardware value.
     */
    public HardwareCache(Supplier<T> valueSupplier) {
        this.valueSupplier = valueSupplier;
    }

    /**
     * Invalidates the cache based on the current caching strategy.
     * If the strategy is {@link Strategy#UPDATE_WHEN_INVALIDATED},
     * the cache will be immediately refreshed upon invalidation.
     */
    @Override
    public void invalidateCache() {
        if (strategy == Strategy.UPDATE_WHEN_INVALIDATED) {
            updateCache();
        } else {
            this.cacheValid = false;
        }

        cacheRead = false;
    }

    /**
     * Updates the cache by fetching the latest value from the supplier.
     */
    @Override
    public void updateCache() {
        this.cachedValue = valueSupplier.get();
        cacheValid = true;
        cacheRead = false;
    }

    /**
     * Reads the cached value, updating it if necessary based on the caching strategy.
     *
     * @return The latest cached value.
     */
    public T read() {
        if (cachedValue == null || !cacheValid) {
            updateCache();
        }

        switch (strategy) {
            case UPDATE_WHEN_INVALIDATED:
            case VALID_UNTIL_INVALIDATED:
                cacheRead = true;
                return cachedValue;
            case INVALID_AFTER_FIRST_READ:
                if (cacheRead) {
                    updateCache();
                } else {
                    cacheRead = true;
                }
                return cachedValue;
        }

        return cachedValue;
    }

    /**
     * Checks if the cache is currently valid.
     *
     * @return {@code true} if the cache is valid, {@code false} otherwise.
     */
    public boolean isCacheValid() {
        return cacheValid;
    }

    /**
     * Sets the caching strategy, invalidating the cache in the process.
     *
     * @param strategy The new caching strategy.
     */
    public void setStrategy(Strategy strategy) {
        this.strategy = strategy;
        cacheValid = false;
        cacheRead = false;
    }

    /**
     * Gets the current caching strategy.
     *
     * @return The active caching strategy.
     */
    public Strategy getStrategy() {
        return strategy;
    }

    /**
     * Updates the cache, then returns the new value.
     * @return the newly cached value.
     */
    public T updateAndGet(){
        updateCache();
        return cachedValue;
    }
}