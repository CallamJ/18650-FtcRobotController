package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.hardware.filters.DataFilter;

/**
 * This class is a wrapper for AnalogInputs that allows you to optionally filter the direct voltage output for better consistency.
 * Please note this class does NOT cache values.
 */
public class SmartAnalogInput extends Device {

	private final AnalogInput base;
	private final DataFilter dataFilter;

	/**
	 * Creates a SmartAnalogInput which does not use a filter, the resulting object will return the same value when calling getVoltage() or getRawVoltage().
	 *
	 * @param base the FTC SDK AnalogInput which will supply the raw voltage reading.
	 * @param configName the name of the input in the hardware map.
	 */
	protected SmartAnalogInput(AnalogInput base, String configName){
		super(configName);
		this.base = base;
		this.dataFilter = DataFilter.NONE;
	}

	/**
	 * Creates a SmartAnalogInput which uses the passed filter, the resulting object will return the different values when calling getVoltage() versus getRawVoltage().
	 *
	 * @param base the FTC SDK AnalogInput which will supply the raw voltage reading.
	 * @param configName the name of the input in the hardware map.
	 * @param dataFilter the filter which will be applied to the voltage, used when accessing the voltage via getVoltage().
	 */
	protected SmartAnalogInput(AnalogInput base, String configName, DataFilter dataFilter){
		super(configName);
		this.base = base;
		this.dataFilter = dataFilter;
	}

	public double getVoltage(){
		return dataFilter.compute(base.getVoltage());
	}

	public double getRawVoltage(){
		double val = base.getVoltage();
		dataFilter.compute(val);
		return val;
	}

}
