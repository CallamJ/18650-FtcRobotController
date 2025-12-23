package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.SmartEncoder;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.controllers.GravityPID;
import org.firstinspires.ftc.teamcode.utilities.Direction;

@Configurable
public class PitchWrist extends AxisComponent {

	//config
	public static float WRIST_TICKS_PER_DEGREE = 8192f/360f;
	public static int UP_POSITION = 90, DOWN_POSITION = 0;
	public static double upKP = 0.03, upKI = 0.0000, upKD = 0.03, upKF = 0.1;
	public static double downKP = 0.005, downKI = 0.000, downKD = 0.0, downKF = -0.04;
	public static double kG = 0;

	private final SmartEncoder wristEncoder;

	private final SmartMotor motor;
	private Mode mode;
	private final TiltBase tiltBase;
	private double wristPower = 0.0;

	public enum Mode {
		MOVE_TO_TARGET, STAY_PARALLEL, STAY_PERPENDICULAR, FLOAT, SET_POWER
	}

	public PitchWrist(TiltBase tiltBase, SmartMotor wristMotor){
		super(new GravityPID.Builder()
				.forwardKP(() -> upKP)
				.forwardKI(() -> upKI)
				.forwardKD(() -> upKD)
				.forwardKF(() -> upKF)

				.reverseKP(() -> downKP)
				.reverseKI(() -> downKI)
				.reverseKD(() -> downKD)
				.reverseKF(() -> downKF)
				.g(() -> kG)
				.setGravityFunction((target, actual) -> Math.sin(Math.toRadians(tiltBase.getCurrentPosition() + actual)))
				.tolerance(3)
				.build()
		);
		this.motor = wristMotor;
		this.tiltBase = tiltBase;
		this.wristEncoder = motor.getEncoder();

		wristEncoder.reset();
		wristEncoder.setDirection(Direction.FORWARD);
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		setMode(Mode.FLOAT);
	}

	public void setWristPower(double wristPower) {
		this.wristPower = wristPower;
	}

	//wrist

	public void pitchUp(){
		setTargetPosition(UP_POSITION);
	}

	public void pitchDown(){
		setTargetPosition(DOWN_POSITION);
	}

	public boolean isUp(){
		return Helper.errorTolerable(getCurrentPosition(), UP_POSITION, 5);
	}

	public boolean isDown(){
		return Helper.errorTolerable(getCurrentPosition(), DOWN_POSITION, 5);
	}

	public boolean isTargetUp(){
		return Helper.errorTolerable(getTargetPosition(), UP_POSITION, 5);
	}

	public boolean isTargetDown(){
		return Helper.errorTolerable(getTargetPosition(), DOWN_POSITION, 5);
	}

	public double getCurrentPosition(){
		return (wristEncoder.getPosition() / WRIST_TICKS_PER_DEGREE);
	}

	public Mode getMode(){
		return mode;
	}

	public void setMode(Mode mode){
		this.mode = mode;
		if(this.mode == Mode.FLOAT){
			motor.setPower(0);
			motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		}else{
			motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}
	}

	/**
	 * Attempts to toggle the wrist target between up and down. If the target is not up or down, the target will not be changed.
	 * @return whether the wrist target was changed.
	 */
	public boolean toggle(){
		if (isTargetUp()) {
			pitchDown();
			return true;
		}else if (isTargetDown()) {
			pitchUp();
			return true;
		}else{
			return false;
		}
	}

	protected void tickPIDF(){
		double power = 0;
		switch (mode) {
			case FLOAT:
				power = 0;
				break;
			case SET_POWER:
				power = wristPower;
				break;
			case STAY_PARALLEL:
				power = controller.calc(90 - tiltBase.getCurrentPosition(), getCurrentPosition());
				break;
			case STAY_PERPENDICULAR:
				power = controller.calc(tiltBase.getCurrentPosition() - 90, getCurrentPosition());
				break;
			case MOVE_TO_TARGET:
				power = controller.calc(getTargetPosition(), getCurrentPosition());
				break;
		}

		motor.setPower(power);
	}

	private static class Helper {
		private static boolean errorTolerable(Number number1, Number number2, Number tolerance){
			return Math.abs(number2.doubleValue() - number1.doubleValue()) <= tolerance.doubleValue();
		}
	}
}