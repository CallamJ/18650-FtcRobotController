package org.firstinspires.ftc.teamcode.hardware;

/**
 * A class that converts an analog signal into a potentiometer based absolute encoder.
 * This class allows reading angular positions and adjusting offsets.
 */
public class SmartPotentiometer extends Device implements Caching {
	private final SmartAnalogInput input;
	private final double maxAngle, maxVoltage;
	private double offsetToZero;
	private final HardwareCache<Double> rawAngleCache;
	private boolean normalize;

	/**
	 * Constructs a `SmartPotentiometer` instance with the given parameters.
	 *
	 * @param input      The `AnalogInput` representing the potentiometer's analog sensor.
	 * @param name       The unique name used for persistent storage of the offset value.
	 * @param maxAngle   The maximum angle (in degrees) the potentiometer can measure.
	 * @param maxVoltage The maximum voltage output of the potentiometer at its highest position.
	 */
	SmartPotentiometer(SmartAnalogInput input, String name, double maxAngle, double maxVoltage) {
		super(name);
		this.input = input;
		this.maxAngle = maxAngle;
		this.maxVoltage = maxVoltage;
		this.offsetToZero = 0;
		this.rawAngleCache = new HardwareCache<>(() -> voltageToAngle(input.getVoltage()));
		this.normalize = false;
	}

	/**
	 * Constructs a `SmartPotentiometer` instance with the given parameters.
	 *
	 * @param input      The `AnalogInput` representing the potentiometer's analog sensor.
	 * @param name       The unique name used for persistent storage of the offset value.
	 * @param maxAngle   The maximum angle (in degrees) the potentiometer can measure.
	 * @param maxVoltage The maximum voltage output of the potentiometer at its highest position.
	 */
	SmartPotentiometer(SmartAnalogInput input, String name, double maxAngle, double maxVoltage, double offset) {
		super(name);
		this.input = input;
		this.maxAngle = maxAngle;
		this.maxVoltage = maxVoltage;
		this.offsetToZero = offset;
		this.rawAngleCache = new HardwareCache<>(() -> voltageToAngle(input.getVoltage()));
		this.normalize = false;
	}

	public boolean isNormalizing(){
		return normalize;
	}

	public void setNormalize(boolean normalize){
		this.normalize = normalize;
	}

	public void setOffset(double offset){
		offsetToZero = offset;
	}

	/**
	 * Gets the current angle of the potentiometer, adjusted for the stored offset.
	 *
	 * @return The adjusted angular position in degrees.
	 */
	public double getAngle() {
		if(normalize){
			return normalizeAngle(getRawAngle() + offsetToZero, maxAngle);
		}
		return getRawAngle() + offsetToZero;
	}

	/**
	 * Gets the raw angle of the potentiometer without applying any offset.
	 * This is calculated based on the sensor's voltage and the conversion factor.
	 *
	 * @return The raw angular position in degrees.
	 */
	public double getRawAngle() {
		return rawAngleCache.read();
	}

	/**
	 * Resets the potentiometer's zero position to the current raw angle.
	 * This effectively sets the current position as the new zero and saves the offset persistently.
	 */
	public void reset() {
		offsetToZero = -getRawAngle();
	}

	/**
	 * Removes any previous offset of the angle.
	 */
	public void clearOffset() {
		offsetToZero = 0;
	}

	public double getOffset(){
		return offsetToZero;
	}

	/**
	 * Normalizes an angle to be within the range [0, maxAngle].
	 * Ensures that negative angles wrap around and large angles are reduced within bounds.
	 *
	 * @param angle    The angle to be normalized.
	 * @param maxAngle The maximum allowable angle before wrapping occurs.
	 * @return The normalized angle within the range [0, maxAngle). Returns in the units of angle & maxAngle.
	 */
	private static double normalizeAngle(double angle, double maxAngle) {
		angle %= maxAngle;

		if (angle < 0) {
			angle += maxAngle;
		}

		return angle == maxAngle ? 0 : angle;
	}

	public double getVoltage(){
		return input.getVoltage();
	}

	/**
	 *
	 */
	@Override
	public void invalidateCache(){
		rawAngleCache.invalidateCache();
	}

	/**
	 *
	 */
	@Override
	public void updateCache(){
		rawAngleCache.updateCache();
	}

	@Override
	public void setStrategy(Strategy strategy){
		rawAngleCache.setStrategy(strategy);
	}

	@Override
	public Strategy getStrategy(){
		return rawAngleCache.getStrategy();
	}

	private static double voltageToAngle(double v){
		return -1.96682 * Math.pow(v, 3) - 7.04864 * Math.pow(v, 2) + 126.39282 * v;
	}
}
