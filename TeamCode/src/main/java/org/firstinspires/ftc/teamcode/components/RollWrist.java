package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.SmartServo;

@Config
public class RollWrist {

	//config
	public static float WRIST_TICKS_PER_DEGREE = 8192f/360f;
	public static float FORWARD_POSITION = 0.8f, BACKWARD_POSITION = 0;

	final SmartServo servo;

	public RollWrist(SmartServo servo){
		this.servo = servo;
	}

	public void goForward(){
		servo.setPosition(FORWARD_POSITION);
	}

	public void goBackward(){
		servo.setPosition(BACKWARD_POSITION);
	}

	public void goTo(double position) {servo.setPosition(position);}

	public boolean isForward(){
		return Helper.errorTolerable(getPosition(), FORWARD_POSITION, 0.1);
	}

	public boolean isBackward(){
		return Helper.errorTolerable(getPosition(), BACKWARD_POSITION, 0.1);
	}

	/**
	 * Checks the current position and if detected it as forward or backward, goes to the backward or forward position respectively.
	 * If position is not close enough to a position to estimate, it does nothing and returns false.
	 *
	 * @return whether the position was toggled
	 */
	public boolean toggle(){
		if (isForward()) {
			goBackward();
			return true;
		}else if (isBackward()) {
			goForward();
			return true;
		}else{
			return false;
		}
	}

	public double getPosition(){
		return servo.getPosition();
	}

	private static class Helper {
		private static boolean errorTolerable(Number number1, Number number2, Number tolerance){
			return Math.abs(number2.doubleValue() - number1.doubleValue()) <= tolerance.doubleValue();
		}
	}
}