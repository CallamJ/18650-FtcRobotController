package org.firstinspires.ftc.teamcode.core;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.utilities.PersistentStorage;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;

/**
 * The most core features of any OpMode without any robot-specific code, usable on any control hub to start an opmode with all the custom components initialized properly.
 */
public abstract class OpModeCore extends LinearOpMode {
	private static OpModeCore instance;
	protected PrettyTelemetry prettyTelem;

	/**
	 * Returns the currently running {@link OpModeCore} instance.
	 *
	 * @return the active op mode instance.
	 */
	public static OpModeCore getInstance(){
		return instance;
	}

	/**
	 * Returns the shared pretty telemetry wrapper for the active op mode.
	 *
	 * @return the active {@link PrettyTelemetry} instance.
	 */
	public static PrettyTelemetry getTelemetry(){
		return instance.prettyTelem;
	}

	/**
	 * Runs the FTC linear op mode lifecycle and dispatches to framework hooks in a fixed order.
	 */
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

	/**
	 * Performs one-time framework initialization and then invokes {@link #onInitialize()} for subclass setup.
	 */
	protected final void initialize(){
		Hardware.init(hardwareMap);
		PersistentStorage.init(hardwareMap);
		this.prettyTelem = new PrettyTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
		onInitialize();
	}

	/**
	 * Hook for subclass-specific initialization after framework services and telemetry are ready.
	 * Runs when the 'init' button is pressed on the Driver Station.
	 */
	protected void onInitialize(){}

	/**
	 * Runs the subclass start hook once after {@link #waitForStart()} completes.
	 */
	protected final void run(){
		onRun();
	}

	/**
	 * Hook for subclass work that should run once when the op mode starts.
	 * Runs when the play button is pressed on the Driver Station.
	 */
	protected void onRun(){}

	/**
	 * Runs one full tick in this order:
	 * {@link #beforeTick()}, {@link #frameworkTick()}, then {@link #onTick()}.
	 * <p>
	 * If you want to invoke a tick, this is the standard method to call.
	 */
	public final void tick(){
		beforeTick();
		frameworkTick();
		onTick();
	}

	/**
	 * Hook for subclass work that must happen before the framework tick runs.
	 * Runs before {@link #frameworkTick()}, meaning that it will run before any hardware updates.
	 */
	protected void beforeTick(){}

	/**
	 * Performs the shared framework tick behavior. Subclasses that override this must call {@code super.frameworkTick()}.
	 * Do not override this method unless you know what you're doing.
	 */
	protected void frameworkTick(){
		Hardware.invalidateCaches();
		this.prettyTelem.update();
	}

	/**
	 * Hook for subclass work that should run after the framework tick completes.
	 * Runs after {@link #frameworkTick()}, meaning that it will run after any hardware updates.
	 * <p>
	 * If you want to run something every tick, this is the standard method to override.
	 */
	protected void onTick(){}
}
