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
        Gamepad gamepad1 = new Gamepad();
        gamepad1.copy(this.gamepad1);
        Gamepad gamepad2 = new Gamepad();
        gamepad2.copy(this.gamepad2);

        checkGamepads(gamepad1, gamepad2, previousGamepad1, previousGamepad2);

        //save the last gamepad state to compare again later
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }

    /**
     * Check for button updates on all controllers.
     *
     * @param gamepad1 the current state of gamepad1.
     * @param gamepad2 the current state of gamepad2.
     * @param lastGamepad1 the last state of gamepad1.
     * @param lastGamepad2 the last state of gamepad2.
     */
    protected abstract void checkGamepads(Gamepad gamepad1, Gamepad gamepad2, Gamepad lastGamepad1, Gamepad lastGamepad2);
}
