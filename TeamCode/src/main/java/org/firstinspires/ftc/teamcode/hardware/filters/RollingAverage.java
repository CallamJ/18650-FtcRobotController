package org.firstinspires.ftc.teamcode.hardware.filters;

import androidx.annotation.NonNull;

import java.util.ArrayDeque;

public class RollingAverage implements DataFilter {
	private int maxSize;
	private final ArrayDeque<Double> values;
	private double sum = 0.0;

	public RollingAverage(int maxSize){
		this.maxSize = maxSize;
		this.values = new ArrayDeque<>(maxSize);
	}

	/**
	 * @noinspection DataFlowIssue
	 */
	public void add(double value){
		if(values.size() >= maxSize){
			sum -= values.pollFirst(); // Remove oldest value
		}
		values.addLast(value);
		sum += value;
	}

	public double getAverage(){
		return values.isEmpty() ? 0.0 : sum / values.size();
	}

	public double compute(double newValue){
		add(newValue);
		return getAverage();
	}

	public int size(){
		return values.size();
	}

	public void setMaxSize(int newMaxSize){
		if(newMaxSize <= 0) throw new IllegalArgumentException("maxSize must be greater than 0");

		this.maxSize = newMaxSize;

		// Trim excess values if the new maxSize is smaller
		while(values.size() > maxSize){
			//noinspection DataFlowIssue
			sum -= values.pollFirst(); // Remove oldest value to match new max size
		}
	}

	public int getMaxSize(){
		return maxSize;
	}

	@NonNull
	@Override
	public String toString(){
		return values.toString();
	}
}
