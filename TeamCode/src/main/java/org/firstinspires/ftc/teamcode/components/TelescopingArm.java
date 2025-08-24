package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartTouchSensor;
import org.firstinspires.ftc.teamcode.hardware.controllers.DirectionalPID;

@Config
public class TelescopingArm extends AxisComponent {
	//<editor-fold desc="Config">

	public static float ARM_TICKS_PER_INCH = 190f;
	public static double MAX_ARM_EXTENSION = 37.0;

	public static double MAX_HORIZONTAL_EXTENSION = 38.0;

	public static double extensionKP = 0.35, extensionKI, extensionKD, extensionKF = 0.15;
	public static double retractionKP = 0.4, retractionKI, retractionKD, retractionKF = 0.35;
	//</editor-fold>

	private final SmartMotor spool;
	public final SmartTouchSensor limitSensor;

	public TelescopingArm(SmartMotor spoolMotor, SmartTouchSensor limitSensor) {
		super(new DirectionalPID.Builder()
				      .forwardKP(() -> extensionKP)
				      .forwardKI(() -> extensionKI)
				      .forwardKD(() -> extensionKD)
				      .forwardKF(() -> extensionKF)

				      .reverseKP(() -> retractionKP)
				      .reverseKI(() -> retractionKI)
				      .reverseKD(() -> retractionKD)
				      .reverseKF(() -> retractionKF)

				      .tolerance(0.1)
				      .build()
		);

		this.spool = spoolMotor;
		this.limitSensor = limitSensor;

		this.spool.setDirection(DcMotorSimple.Direction.REVERSE);

		reset();
		setTargetPosition(getCurrentPosition());
	}

	/**
	 * Get the current extension of the end of the arm past the minimum extension (fully retracted).
	 *
	 * @return the extension of the end of the arm.
	 */
	public double getCurrentPosition(){
		return spool.getCurrentPosition() / ARM_TICKS_PER_INCH;
	}

	/**
	 * Sets the current position as the zero position.
	 */
	public void reset(){
		spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	@Override
	public void tick(){
		if(limitSensor.isPressed()){
			reset();
			if(getTargetPosition() < 0)
				setTargetPosition(0);
		}

		super.tick();
	}

	/**
	 * Runs a cycle on the PIDF control loop for the arm.
	 */
	protected void tickPIDF(){
		controller.calc(getTargetPosition(), getCurrentPosition());

		spool.setPower(controller.result());
	}

	public boolean isValidExtension(double inches){
		return !(inches > MAX_ARM_EXTENSION) && !(inches < -0.5);
	}
}
