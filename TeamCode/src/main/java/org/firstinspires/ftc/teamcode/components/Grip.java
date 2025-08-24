package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.SmartServo;

@Config
public class Grip{

	//config
	public static float WRIST_TICKS_PER_DEGREE = 8192f/360f;
	public static float OPEN_POSITION = 0.4f, CLOSED_POSITION = 1;

	final SmartServo servo;

	public Grip(SmartServo servo){
		this.servo = servo;
	}

	public void openGrip(){
		servo.setPosition(OPEN_POSITION);
	}

	public void closeGrip(){
		servo.setPosition(CLOSED_POSITION);
	}

	public void setGripPosition(double position) {
		servo.setPosition(position);
	}

	public boolean isOpen(){
		return Helper.errorTolerable(getGripPosition(), OPEN_POSITION, 0.1);
	}

	public boolean isClosed(){
		return Helper.errorTolerable(getGripPosition(), CLOSED_POSITION, 0.1);
	}

	/**
	 * Checks the current position and if detected it as open or closed, closes it or opens it respectively.
	 * If position is not close enough to a closed or open position to estimate, it does nothing and returns false.
	 * @return whether the grip was toggled
	 */
	public boolean toggleGrip(){
		if (isOpen()) {
			closeGrip();
			return true;
		}else if (isClosed()) {
			openGrip();
			return true;
		}else{
			return false;
		}
	}

	public double getGripPosition(){
		return servo.getPosition();
	}

	private static class Helper {
		private static boolean errorTolerable(Number number1, Number number2, Number tolerance){
			return Math.abs(number2.doubleValue() - number1.doubleValue()) <= tolerance.doubleValue();
		}
	}
}