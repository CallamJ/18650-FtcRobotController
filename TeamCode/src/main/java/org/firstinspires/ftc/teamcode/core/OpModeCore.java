package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
public abstract class OpModeCore extends BasicOpModeCore {

	//<editor-fold desc="Fields">
	//components
	protected List<LynxModule> lynxModules;
	protected ElapsedTime tickTimer;
	//</editor-fold>

	@Override
	protected void initialize(){
		super.initialize();

		tickTimer = new ElapsedTime();

		// always configure telemetry last
		configureTelemetry();
	}

    abstract protected void configureTelemetry();

	public void tick(){
		super.tick();
	}
}
