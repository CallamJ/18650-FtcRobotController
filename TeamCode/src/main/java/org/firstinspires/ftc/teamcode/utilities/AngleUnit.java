package org.firstinspires.ftc.teamcode.utilities;

public enum AngleUnit {
	DEGREES(360),
	RADIANS(Math.PI * 2);

	private final double perRevolution;
	AngleUnit(double perRevolution){
		this.perRevolution = perRevolution;
	}
}
