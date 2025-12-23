package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;

/**
 * SmartEncoder is a wrapper around either a RoadRunner Encoder or a DcMotorEx encoder,
 * providing caching and additional functionality like resetting offsets and direction control.
 */
public class SmartEncoder extends Device implements Caching {
    private final HardwareCache<Integer> positionCache;
    private final HardwareCache<Double> velocityCache;
    private final boolean usesRRBase;
    private final String storageKey;

    private int tickOffsetToZero;
    private Direction direction = Direction.FORWARD;

    /**
     * Constructs a SmartEncoder that directly reads from a DcMotorEx encoder.
     *
     * @param motor The motor with an internal encoder.
     */
    SmartEncoder(DcMotorEx motor, String name) {
        super(name);
        this.positionCache = new HardwareCache<>(motor::getCurrentPosition);
        this.velocityCache = new HardwareCache<>(motor::getVelocity);
        this.usesRRBase = false;
        this.storageKey = "encoder_angle_offset(" + name + ")";
    }

    /**
     * Returns the current position of the encoder, adjusted for offset and direction.
     *
     * @return The adjusted encoder position.
     */
    public int getPosition() {
        if (usesRRBase) {
            return positionCache.read() - tickOffsetToZero;
        } else {
            return (positionCache.read() - tickOffsetToZero) * (direction == Direction.FORWARD ? 1 : -1);
        }
    }

    /**
     * Returns the current velocity of the encoder.
     *
     * @return The velocity in ticks per second.
     */
    public double getVelocity() {
        return velocityCache.read();
    }

    /**
     * Invalidates the cached encoder values.
     */
    @Override
    public void invalidateCache() {
        positionCache.invalidateCache();
        velocityCache.invalidateCache();
    }

    /**
     * Updates the cached encoder values.
     */
    @Override
    public void updateCache() {
        positionCache.updateCache();
        velocityCache.updateCache();
    }

    /**
     * Sets the caching strategy for the encoder.
     *
     * @param strategy The caching strategy to use.
     */
    @Override
    public void setStrategy(Strategy strategy) {
        positionCache.setStrategy(strategy);
        velocityCache.setStrategy(strategy);
    }

    /**
     * Gets the current caching strategy.
     *
     * @return The current caching strategy.
     */
    @Override
    public Strategy getStrategy() {
        return positionCache.getStrategy();
    }

    /**
     * Resets the encoder by setting the current position as the new zero.
     */
    public void reset() {
        tickOffsetToZero = positionCache.updateAndGet();
        PersistentStorage.saveInt(storageKey, tickOffsetToZero);
    }

    public void resetAs(int position) {
        tickOffsetToZero = positionCache.updateAndGet() - position;
        PersistentStorage.saveInt(storageKey, tickOffsetToZero);
    }

    /**
     * Sets the direction of the encoder.
     *
     * @param direction The new direction (FORWARD or REVERSE).
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Gets the current direction of the encoder.
     *
     * @return The encoder direction.
     */
    public Direction getDirection() {
        return direction;
    }
}
