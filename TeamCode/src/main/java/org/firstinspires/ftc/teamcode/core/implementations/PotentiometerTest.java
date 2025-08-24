package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.core.BasicTeleOpCore;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;

@TeleOp(name = "PotentiometerTestA")
public class PotentiometerTest extends BasicTeleOpCore {
	SmartPotentiometer potentiometer;

	@Override
	protected void initialize(){
		super.initialize();
		potentiometer = Hardware.getPotentiometer("tiltPotentiometer", 270, 3.33);
		prettyTelem.addData("Voltage", () -> potentiometer.getVoltage());
		prettyTelem.addData("Angle", () -> potentiometer.getAngle());
		prettyTelem.addData("Offset", () -> potentiometer.getOffset());
		prettyTelem.addData("Raw Angle", () -> potentiometer.getRawAngle());
		prettyTelem.setShowLogsInTelemetry(true);
		prettyTelem.info("START RAW ANGLE: " + potentiometer.getRawAngle());
		prettyTelem.info("START ANGLE: " + potentiometer.getAngle());
	}

	/**
	 * Check for button updates on all controllers.
	 *
	 * @param gamepad1     the current state of gamepad1.
	 * @param gamepad2     the current state of gamepad2.
	 * @param lastGamepad1 the last state of gamepad1.
	 * @param lastGamepad2 the last state of gamepad2.
	 */
	@Override
	protected void checkGamepads(Gamepad gamepad1, Gamepad gamepad2, Gamepad lastGamepad1, Gamepad lastGamepad2){
		if(gamepad1.a && !lastGamepad1.a){
			potentiometer.reset();
			prettyTelem.info("RESET");
		}

		if(gamepad1.b && !lastGamepad1.b){
			potentiometer.clearOffset();
			prettyTelem.info("CLEAN OFFSET");
		}
	}
}
