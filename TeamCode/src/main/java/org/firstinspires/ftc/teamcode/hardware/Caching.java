package org.firstinspires.ftc.teamcode.hardware;

public interface Caching {
    void invalidateCache();

    void updateCache();

    void setStrategy(Strategy strategy);

    Strategy getStrategy();

    enum Strategy {
        /**
         * When the cache is updated, the first read is from the cache. The second read will invalidate the cache and update the cache before reading..
         */
        INVALID_AFTER_FIRST_READ,
        /**
         * After the cache is updated, all reads will read from the cache. Once the cache is invalidated the next read will first update the cache.
         */
        VALID_UNTIL_INVALIDATED,
        /**
         * After the cache is updated, all reads will read from the cache (similar to VALID_UNTIL_INVALIDATED). When the cache is invalidated the cache is automatically updated.
         */
        UPDATE_WHEN_INVALIDATED
    }
}
