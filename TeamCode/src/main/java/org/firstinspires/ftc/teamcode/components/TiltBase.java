package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;
import org.firstinspires.ftc.teamcode.hardware.SmartTouchSensor;
import org.firstinspires.ftc.teamcode.hardware.controllers.GravityPID;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;

@Configurable
public class TiltBase extends AxisComponent {
	//<editor-fold desc="Config">

	public static double forwardKP = 0.03, forwardKI = 0, forwardKD = 0.1, forwardKF = 0.2;
	public static double reverseKP = 0.01, reverseKI = 0, reverseKD = 0.1, reverseKF = 0;
	public static double kG = 0;
	//</editor-fold>

	private final SmartMotor angleMotorRight;
	private final SmartMotor angleMotorLeft;

	private final SmartPotentiometer anglePotentiometer;
	public final SmartTouchSensor limitSensor;

	private final TelescopingArm telescopingArm;

	public TiltBase(TelescopingArm telescopingArm, SmartMotor angleMotorLeft, SmartMotor angleMotorRight, SmartTouchSensor limitSensor, SmartPotentiometer tiltPotentiometer) {
		super(new GravityPID.Builder()
				      .setGravityFunction((g, actual) -> g * Math.cos(Math.toRadians(actual)) * (19 + telescopingArm.getCurrentPosition()))

				      .forwardKP(() -> forwardKP)
				      .forwardKI(() -> forwardKI)
				      .forwardKD(() -> forwardKD)
				      .forwardKF(() -> forwardKF)

				      .reverseKP(() -> reverseKP)
				      .reverseKI(() -> reverseKI)
				      .reverseKD(() -> reverseKD)
				      .reverseKF(() -> reverseKF)

				      .g(() -> kG)

				      .tolerance(0.75)
				      .build()
		);

		//<editor-fold desc="Hardware Config">
		this.angleMotorRight = angleMotorRight;
		this.angleMotorLeft = angleMotorLeft;
		this.limitSensor = limitSensor;
		this.anglePotentiometer = tiltPotentiometer;
		this.telescopingArm = telescopingArm;

		this.angleMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		this.angleMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

		this.angleMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.angleMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		this.angleMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//</editor-fold>

		PrettyTelemetry prettyTelem = OpModeCore.getTelemetry();
		prettyTelem.setShowLogsInTelemetry(true);
		prettyTelem.setMinLogLevel(PrettyTelemetry.LogLevel.DEBUG);
		prettyTelem.debug("START OFFSET: " + tiltPotentiometer.getOffset());
		prettyTelem.debug("START RAW ANGLE: " + tiltPotentiometer.getRawAngle());
		prettyTelem.debug("START ANGLE: " + tiltPotentiometer.getAngle());
		setTargetPosition(getCurrentPosition());
	}

	/**
	 * Get the current angle of the arm. This is relative to the base, at 0 the arm is horizontal, and at 90 the arm is vertical.
	 * Uses should be able to handle angles past 90 degrees, since the motor will not always land at exactly 90.
	 *
	 * @return the angle of the arm relative to the base.
	 */
	public double getCurrentPosition(){
		return anglePotentiometer.getAngle();
	}

	/**
	 * Sets the current angle as the zero position.
	 */
	public void resetAngle(){
		anglePotentiometer.reset();
	}

	/**
	 * Runs a controller cycle for the arm.
	 * This method should be called once per OpMode cycle to maintain the arm's position when at target,
	 * or adjust the arm's position when not at target. This controls both extension and retraction.
	 */
	public void tick(){
		if(limitSensor.isPressed()){
			resetAngle();
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

		angleMotorRight.setPower(controller.result());
		angleMotorLeft.setPower(controller.result());
	}

	public boolean isValidAngle(double degrees){
		return !(degrees < 0) && !(degrees > 100);
	}
}
