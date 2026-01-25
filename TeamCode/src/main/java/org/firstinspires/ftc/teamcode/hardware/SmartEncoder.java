package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.utilities.Direction;

/**
 * SmartEncoder is a wrapper around either a RoadRunner Encoder or a DcMotorEx encoder,
 * providing caching and additional functionality like resetting offsets and direction control.
 */
public class SmartEncoder extends Device implements Caching {
    private final HardwareCache<Integer> positionCache;
    private final HardwareCache<Double> velocityCache;
    private final boolean usesBase;
    private BaseEncoder encoder;
    private int tickOffsetToZero;
    private Direction direction = Direction.FORWARD;

    /**
     * Constructs a SmartEncoder that may read frm an external encoder.
     *
     * @param motor The motor with an internal encoder.
     * @param isExternal if the encoder is an external encoder
     */
    SmartEncoder(DcMotorEx motor, String name, boolean isExternal) {
        super(name);
        this.usesBase = isExternal;
        if(usesBase){
            this.encoder = new BaseEncoder(motor);
            this.positionCache = new HardwareCache<>(encoder::getCurrentPosition);
            this.velocityCache = new HardwareCache<>(encoder::getCorrectedVelocity);
        } else {
            this.positionCache = new HardwareCache<>(motor::getCurrentPosition);
            this.velocityCache = new HardwareCache<>(motor::getVelocity);
        }
    }

    /**
     * Constructs a SmartEncoder that directly reads from a DcMotorEx encoder.
     *
     * @param motor The motor with an internal encoder.
     */
    SmartEncoder(DcMotorEx motor, String name) {
        super(name);
        this.usesBase = false;
        this.positionCache = new HardwareCache<>(motor::getCurrentPosition);
        this.velocityCache = new HardwareCache<>(motor::getVelocity);
    }

    /**
     * Returns the current position of the encoder, adjusted for offset and direction.
     *
     * @return The adjusted encoder position.
     */
    public int getPosition() {
        if (usesBase) {
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
    }

    public void resetAs(int position) {
        tickOffsetToZero = positionCache.updateAndGet() - position;
    }

    /**
     * Sets the direction of the encoder.
     *
     * @param direction The new direction (FORWARD or REVERSE).
     */
    public void setDirection(Direction direction) {
        if(this.encoder != null){
            if(direction == Direction.FORWARD){
                encoder.setDirection(BaseEncoder.Direction.FORWARD);
            } else if(direction == Direction.REVERSE) {
                encoder.setDirection(BaseEncoder.Direction.REVERSE);
            }
        }
        if(this.direction != direction){
            tickOffsetToZero = -tickOffsetToZero;
        }
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

    /**
     * Wraps a motor instance to provide corrected velocity counts and allow reversing independently of the corresponding
     * slot's motor direction
     */
    public static class BaseEncoder {
        private final static int CPS_STEP = 0x10000;

        private static double inverseOverflow(double input, double estimate) {
            // convert to uint16
            int real = (int) input & 0xffff;
            // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
            // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
            real += ((real % 20) / 4) * CPS_STEP;
            // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
            real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
            return real;
        }

        public enum Direction {
            FORWARD(1),
            REVERSE(-1);

            private int multiplier;

            Direction(int multiplier) {
                this.multiplier = multiplier;
            }

            public int getMultiplier() {
                return multiplier;
            }
        }

        private DcMotorEx motor;
        private long startTimeNanos;

        private Direction direction;

        private int lastPosition;
        private int velocityEstimateIdx;
        private double[] velocityEstimates;
        private double lastUpdateTime;

        public BaseEncoder(DcMotorEx motor) {
            this.motor = motor;
            this.startTimeNanos = System.nanoTime();

            this.direction = Direction.FORWARD;

            this.lastPosition = 0;
            this.velocityEstimates = new double[3];
            this.lastUpdateTime = seconds();
        }

        private double seconds() {
            return (System.nanoTime() - startTimeNanos) / 1e9;
        }

        public Direction getDirection() {
            return direction;
        }

        private int getMultiplier() {
            return getDirection().getMultiplier() * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
        }

        /**
         * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
         * @param direction either reverse or forward depending on if encoder counts should be negated
         */
        public void setDirection(Direction direction) {
            this.direction = direction;
        }

        /**
         * Gets the position from the underlying motor and adjusts for the set direction.
         * Additionally, this method updates the velocity estimates used for compensated velocity
         *
         * @return encoder position
         */
        public int getCurrentPosition() {
            int multiplier = getMultiplier();
            int currentPosition = motor.getCurrentPosition() * multiplier;
            if (currentPosition != lastPosition) {
                double currentTime = seconds();
                double dt = currentTime - lastUpdateTime;
                velocityEstimates[velocityEstimateIdx] = (currentPosition - lastPosition) / dt;
                velocityEstimateIdx = (velocityEstimateIdx + 1) % 3;
                lastPosition = currentPosition;
                lastUpdateTime = currentTime;
            }
            return currentPosition;
        }

        /**
         * Gets the velocity directly from the underlying motor and compensates for the direction
         * See {@link #getCorrectedVelocity} for high (>2^15) counts per second velocities (such as on REV Through Bore)
         *
         * @return raw velocity
         */
        public double getRawVelocity() {
            int multiplier = getMultiplier();
            return motor.getVelocity() * multiplier;
        }

        /**
         * Uses velocity estimates gathered in {@link #getCurrentPosition} to estimate the upper bits of velocity
         * that are lost in overflow due to velocity being transmitted as 16 bits.
         * CAVEAT: must regularly call {@link #getCurrentPosition} for the compensation to work correctly.
         *
         * @return corrected velocity
         */
        public double getCorrectedVelocity() {
            double median = velocityEstimates[0] > velocityEstimates[1]
                    ? Math.max(velocityEstimates[1], Math.min(velocityEstimates[0], velocityEstimates[2]))
                    : Math.max(velocityEstimates[0], Math.min(velocityEstimates[1], velocityEstimates[2]));
            return inverseOverflow(getRawVelocity(), median);
        }
    }

}
