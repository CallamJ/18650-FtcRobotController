package org.firstinspires.ftc.teamcode.core.implementations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.*;
import org.firstinspires.ftc.teamcode.core.TeleOpCore;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

@Config
@TeleOp(name = "Old Teleop")
public class MainTeleOpIntoTheDeep extends TeleOpCore {
    private boolean isHighPower = false;
    protected ElapsedTime gamepadTimer;

    protected static Grip grip;
    protected static PitchWrist pitch;
    protected static RollWrist roll;
    protected static DriveBase driveBase;
    protected static TelescopingArm telescoping;
    protected static TiltBase tilt;

    //<editor-fold desc="Config">
    public static float LOW_POWER_MODIFIER = 0.25f;
    public static float HIGH_POWER_MODIFIER = 0.75f;
    public static float MAX_INCHES_PER_SECOND = 12f;
    //</editor-fold>


    @Override
    protected void initialize(){
        super.initialize();
        //initialize hardware
        driveBase = new DriveBase(hardwareMap);

        telescoping = new TelescopingArm(
                Hardware.getMotor("extensionMotor"),
                Hardware.getTouchSensor("extensionLimitSensor")
        );
        tilt = new TiltBase(
                telescoping,
                Hardware.getMotor("tiltMotorLeft"),
                Hardware.getMotor("tiltMotorRight", true),
                Hardware.getTouchSensor("tiltLimitSensor"),
                Hardware.getPotentiometer("tiltPotentiometer", 270, 3.33, -70)
        );


        grip = new Grip(
                Hardware.getServo("gripServo")
        );
        pitch = new PitchWrist(
                tilt,
                Hardware.getMotor("wristMotor", true)
        );
        roll = new RollWrist(
                Hardware.getServo("wristServo")
        );

        gamepadTimer = new ElapsedTime();
        telemetry.log();
    }

    @Override
    public void tick() {
        super.tick();
        tilt.tick();
        telescoping.tick();
        pitch.tick();
        tickTimer.reset();
    }

    @Override
    protected void checkGamepads(Gamepad gamepad1, Gamepad gamepad2, Gamepad lastGamepad1, Gamepad lastGamepad2) {
        //toggle grip on pressing a, if failed to detect if open or closed, default to close.
        if(gamepad1.a){
            if(!previousGamepad1.a) {
                if (!grip.toggleGrip()) {
                    grip.closeGrip();
                }
            }
        }

        //toggle wrist on pressing b, if failed to detect if up or down, default to up.
        if(gamepad1.b && !previousGamepad1.b){
            pitch.setMode(PitchWrist.Mode.MOVE_TO_TARGET);
            if(!pitch.toggle())
                pitch.pitchUp();
        }

        if(gamepad1.x && !previousGamepad1.x) {
            isHighPower = !isHighPower;
            if (isHighPower) {
                driveBase.setPowerFactor(HIGH_POWER_MODIFIER);
            } else {
                driveBase.setPowerFactor(LOW_POWER_MODIFIER);
            }
        }

        if(gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            tilt.setTargetPosition(30);
            telescoping.setTargetPosition(9.5);
            pitch.setMode(PitchWrist.Mode.MOVE_TO_TARGET);
            pitch.setTargetPosition(-34);
        }

        if(gamepad1.dpad_down && !previousGamepad1.dpad_down){
            //pitch up, then extension in, then tilt down
            pitch.setMode(PitchWrist.Mode.MOVE_TO_TARGET);
            pitch.goToAsync(90, 750)
                    .thenRun(() -> telescoping.goToAsync(0)
                            .thenRun(() -> tilt.goToAsync(0)));

        }else if(gamepad1.dpad_up && !previousGamepad1.dpad_up){
            //tilt up
            tilt.setTargetPosition(100);
        }

        double extensionOffset =  (-gamepad1.left_trigger + gamepad1.right_trigger) * gamepadTimer.seconds() * MAX_INCHES_PER_SECOND;
        if(Math.abs(extensionOffset) > 0.02){
            telescoping.setTargetPosition(telescoping.getTargetPosition() + extensionOffset);
        }

        if(gamepad1.y && !lastGamepad1.y){
            if(!roll.toggle())
                roll.goForward();
        }

        //specimen delivery macro
        if(gamepad1.dpad_left && !lastGamepad1.dpad_left){
            pitch.setMode(PitchWrist.Mode.MOVE_TO_TARGET);
            pitch.goToAsync(40, 750)
                    .thenRun(() -> telescoping.goToAsync(16.2)
                            .thenRun(() -> tilt.goToAsync(45)));
        }

        gamepadTimer.reset();

        driveBase.moveUsingRR(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    protected void configureTelemetry(){

        prettyTelem.addLine("System Status")
                .addData("Tick Time", () -> Math.round(tickTimer.milliseconds()))
                .addData("Localization: ", () -> driveBase.getPoseSimple())
        ;

        prettyTelem.addLine("Tilt")
                .addData("Current Angle", () -> tilt.getCurrentPosition())
                .addData("Target Angle", () -> tilt.getTargetPosition())
                .addData("Power", () -> tilt.getPower())
                .addData("Limit Sensor Pressed?", () -> tilt.limitSensor.isPressed());

        prettyTelem.addLine("Extension")
                .addData("Current Length", () -> telescoping.getCurrentPosition())
                .addData("Target Length", () -> telescoping.getTargetPosition())
                .addData("Power", () -> telescoping.getPower())
                .addData("Limit Sensor Pressed?", () -> telescoping.limitSensor.isPressed());

        prettyTelem.addLine("Grip")
                .addData("Position", () -> grip.getGripPosition())
                .addData("Open/Closed", () -> grip.isOpen() ? "Open" : grip.isClosed() ? "Closed" : "No");

        prettyTelem.addLine("Pitch Wrist")
                .addData("Position", pitch::getCurrentPosition)
                .addData("Target", pitch::getTargetPosition)
                .addData("Power", pitch::getPower)
                .addData("Up/Down", () -> pitch.isUp() ? "Up" : pitch.isDown() ? "Down" : "No")
                .addData("Mode", pitch::getMode);

        prettyTelem.addLine("Roll Wrist")
                .addData("Position", () -> roll.getPosition())
                .addData("Forward/Backward", () -> roll.isForward() ? "Forward" : roll.isBackward() ? "Backward" : "No");

    }
}
