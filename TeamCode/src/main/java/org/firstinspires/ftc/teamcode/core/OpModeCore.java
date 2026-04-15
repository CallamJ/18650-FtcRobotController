package org.firstinspires.ftc.teamcode.core;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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
		run();
		while(opModeIsActive()){
			tick();
		}
	}

	protected final void initialize(){
		Hardware.init(hardwareMap);
		PersistentStorage.init(hardwareMap);
		this.prettyTelem = new PrettyTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
		onInitialize();
	}

	protected void onInitialize(){}

	protected final void run(){
		onRun();
	}

	protected void onRun(){}

	public final void tick(){
		onTickBeforeFramework();
		beforeTick();
		doFrameworkTick();
		onTickAfterFramework();
	}

	protected void onTickBeforeFramework(){}

	protected void beforeTick(){}

	protected void onTickAfterFramework(){}

	protected void doFrameworkTick(){
		simpleTick();
	}

	protected final void frameworkTick(){
		beforeTick();
		doFrameworkTick();
	}

	public static void simpleTick(){
		Hardware.invalidateCaches();
		getTelemetry().update();
	}
}
