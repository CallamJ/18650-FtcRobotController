package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;

/**
 * The most core features of any OpMode without any robot-specific code, usable on any control hub to provide a framework with all the custom components initialized properly.
 */
public abstract class BasicOpModeCore extends LinearOpMode {
	private static BasicOpModeCore instance;
	protected PrettyTelemetry prettyTelem;

	public static BasicOpModeCore getInstance(){
		return instance;
	}

	public static PrettyTelemetry getTelemetry(){
		return instance.prettyTelem;
	}

	@Override
	public void runOpMode(){
		instance = this;
		initialize();
		waitForStart();
		while(opModeIsActive()){
			tick();
		}
	}

	protected void initialize(){
		Hardware.init(hardwareMap);
		PersistentStorage.init(hardwareMap);
		this.prettyTelem = new PrettyTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
	}

	public void tick(){
		simpleTick();
	}

	public static void simpleTick(){
		Hardware.invalidateCaches();
		getTelemetry().update();
	}
}
