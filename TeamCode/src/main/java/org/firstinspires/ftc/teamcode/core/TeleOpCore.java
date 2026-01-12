package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class TeleOpCore extends OpModeCore {

	protected final Gamepad previousGamepad1 = new Gamepad();
	protected final Gamepad previousGamepad2 = new Gamepad();

	@Override
	protected void initialize(){
		super.initialize();
		//save the current gamepad states to compare against to avoid errors
		previousGamepad1.copy(gamepad1);
		previousGamepad2.copy(gamepad2);
	}

	@Override
	public void tick() {
		checkGamepads();
		super.tick();
	}

	//this might be moved to a separate class
	private void checkGamepads() {
		//store the current game pads since this state can change while in a check cycle
		Gamepad gamepad1Base = new Gamepad();
		gamepad1Base.copy(this.gamepad1);
		Gamepad gamepad2Base = new Gamepad();
		gamepad2Base.copy(this.gamepad2);

        SmartGamepad gamepad1 = new SmartGamepad(gamepad1Base, previousGamepad1);
        SmartGamepad gamepad2 = new SmartGamepad(gamepad2Base, previousGamepad2);

		checkGamepads(gamepad1, gamepad2);

		//save the last gamepad state to compare again later
		previousGamepad1.copy(gamepad1Base);
		previousGamepad2.copy(gamepad2Base);
	}

	/**
	 * Check for button updates on all controllers.
	 *
	 * @param gamepad1 the state of gamepad1.
	 * @param gamepad2 the state of gamepad2.
	 */
	protected abstract void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2);
}
