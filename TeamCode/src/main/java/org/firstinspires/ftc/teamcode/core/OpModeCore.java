package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Grip;
import org.firstinspires.ftc.teamcode.components.PitchWrist;
import org.firstinspires.ftc.teamcode.components.RollWrist;
import org.firstinspires.ftc.teamcode.components.TelescopingArm;
import org.firstinspires.ftc.teamcode.components.TiltBase;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.filters.RollingAverage;

import java.util.List;

@Config
public abstract class OpModeCore extends BasicOpModeCore {

	//<editor-fold desc="Fields">
	//components
	protected static OpModeCore instance;
	protected static Grip grip;
	protected static PitchWrist pitch;
	protected static RollWrist roll;
	protected static DriveBase driveBase;
	protected static TelescopingArm telescoping;
	protected static TiltBase tilt;
	protected List<LynxModule> lynxModules;
	protected ElapsedTime tickTimer;
	//</editor-fold>

	//<editor-fold desc="Instance Getters">
	public static OpModeCore getInstance(){
		return instance;
	}
	//</editor-fold>

	@Override
	protected void initialize(){
		super.initialize();
		instance = this;

		//initialize hardware
		driveBase = new DriveBase(hardwareMap);

		telescoping = new TelescopingArm(
				Hardware.getMotor("extensionMotor"),
				Hardware.getTouchSensor("extensionLimitSensor")
		);
		tilt = new TiltBase(
				telescoping,
				Hardware.getMotor("tiltMotorLeft"),
				Hardware.getMotor("tiltMotorRight", true),
				Hardware.getTouchSensor("tiltLimitSensor"),
				Hardware.getPotentiometer("tiltPotentiometer", 270, 3.33)
		);


		grip = new Grip(
				Hardware.getServo("gripServo")
		);
		pitch = new PitchWrist(
				tilt,
				Hardware.getMotor("wristMotor", true)
		);
		roll = new RollWrist(
				Hardware.getServo("wristServo")
		);

		tickTimer = new ElapsedTime();

		// always configure telemetry last
		configureTelemetry();
	}

	protected void configureTelemetry(){

		prettyTelem.addLine("System Status")
				.addData("Tick Time", () -> Math.round(tickTimer.milliseconds()))
				.addData("Localization: ", () -> driveBase.getPoseSimple())
		;

		prettyTelem.addLine("Tilt")
				.addData("Current Angle", () -> tilt.getCurrentPosition())
				.addData("Target Angle", () -> tilt.getTargetPosition())
				.addData("Power", () -> tilt.getPower())
				.addData("Limit Sensor Pressed?", () -> tilt.limitSensor.isPressed());

		prettyTelem.addLine("Extension")
				.addData("Current Length", () -> telescoping.getCurrentPosition())
				.addData("Target Length", () -> telescoping.getTargetPosition())
				.addData("Power", () -> telescoping.getPower())
				.addData("Limit Sensor Pressed?", () -> telescoping.limitSensor.isPressed());

		prettyTelem.addLine("Grip")
				.addData("Position", () -> grip.getGripPosition())
				.addData("Open/Closed", () -> grip.isOpen() ? "Open" : grip.isClosed() ? "Closed" : "No");

		prettyTelem.addLine("Pitch Wrist")
				.addData("Position", pitch::getCurrentPosition)
				.addData("Target", pitch::getTargetPosition)
				.addData("Power", pitch::getPower)
				.addData("Up/Down", () -> pitch.isUp() ? "Up" : pitch.isDown() ? "Down" : "No")
				.addData("Mode", pitch::getMode);

		prettyTelem.addLine("Roll Wrist")
				.addData("Position", () -> roll.getPosition())
				.addData("Forward/Backward", () -> roll.isForward() ? "Forward" : roll.isBackward() ? "Backward" : "No");

	}

	public void tick(){
		super.tick();
		tilt.tick();
		telescoping.tick();
		pitch.tick();
		tickTimer.reset();
	}
}
