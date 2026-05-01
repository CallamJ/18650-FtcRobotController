package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.hardware.SmartCamera;
import org.firstinspires.ftc.teamcode.hardware.SmartCameraColorSensor;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "8 - Color Cam Diagnostics")
public class ColorCamDiagnosticsTeleOp extends TeleOpCore {
    private SmartCamera camera;
    private SmartCameraColorSensor colorSensor;
    private VisionPortal portal;

    @Override
    protected void onInitialize() {
        super.onInitialize();
        camera = hardware.getCamera("colorCamera", new Pose(0, 0, 0));
        colorSensor = camera.asColorSensor();
        portal = colorSensor.getVisionPortal();

        prettyTelem.info("Camera initialized");
        prettyTelem.addData("Detected Color", colorSensor::getClosestColorMatchName);
    }

    @Override
    protected void checkGamepads(SmartGamepad gamepad1, SmartGamepad gamepad2) {

    }
}
