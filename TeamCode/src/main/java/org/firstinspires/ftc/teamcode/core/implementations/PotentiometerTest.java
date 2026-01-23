package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;

@TeleOp(name = "PotentiometerTestA")
public class PotentiometerTest extends TeleOpCore {
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

	@Override
	protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2){
		if(gamepad1.aPressed()){
			potentiometer.reset();
			prettyTelem.info("RESET");
		}

		if(gamepad1.bPressed()){
			potentiometer.clearOffset();
			prettyTelem.info("CLEAN OFFSET");
		}
	}
}
