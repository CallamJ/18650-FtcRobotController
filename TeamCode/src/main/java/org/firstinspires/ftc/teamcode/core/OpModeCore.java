package org.firstinspires.ftc.teamcode.core;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;

/**
 * The most core features of any OpMode without any robot-specific code, usable on any control hub to provide a framework with all the custom components initialized properly.
 */
public abstract class OpModeCore extends LinearOpMode {
	private static OpModeCore instance;
	protected PrettyTelemetry prettyTelem;

	public static OpModeCore getInstance(){
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
		this.prettyTelem = new PrettyTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
	}

	public void tick(){
		simpleTick();
	}

	public static void simpleTick(){
		Hardware.invalidateCaches();
		getTelemetry().update();
	}
}
