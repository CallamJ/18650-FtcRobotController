package org.firstinspires.ftc.teamcode.hardware.filters;

public interface DataFilter {
	double compute(double value);

	DataFilter NONE = (value -> value);
}
