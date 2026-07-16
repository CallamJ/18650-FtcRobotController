package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.Gamepad;

public class SmartGamepad {
    private Gamepad base;
    private Gamepad baseLast;
    public SmartGamepad(Gamepad gamepad, Gamepad lastGamepad) {
        this.base = gamepad;
        this.baseLast = lastGamepad;
        this.a = gamepad.a;
        this.b = gamepad.b;
        this.x = gamepad.x;
        this.y = gamepad.y;
        this.cross = gamepad.cross;
        this.triangle = gamepad.triangle;
        this.circle = gamepad.circle;
        this.square = gamepad.square;

        this.leftBumper = gamepad.left_bumper;
        this.rightBumper = gamepad.right_bumper;
        this.leftTrigger = gamepad.left_trigger;
        this.rightTrigger = gamepad.right_trigger;

        this.dpadLeft = gamepad.dpad_left;
        this.dpadRight = gamepad.dpad_right;
        this.dpadUp = gamepad.dpad_up;
        this.dpadDown = gamepad.dpad_down;

        this.leftStickButton = gamepad.left_stick_button;
        this.rightStickButton = gamepad.right_stick_button;

        this.leftStickX = gamepad.left_stick_x;
        this.leftStickY = gamepad.left_stick_y;
        this.rightStickX = gamepad.right_stick_x;
        this.rightStickY = gamepad.right_stick_y;

        this.start = gamepad.start;
        this.options = gamepad.options;
        this.guide = gamepad.guide;
        this.share = gamepad.share;

        this.touchpad = gamepad.touchpad;
        this.touchpadFinger1 = gamepad.touchpad_finger_1;
        this.touchpadFinger2 = gamepad.touchpad_finger_2;
        this.touchpadFinger1X = gamepad.touchpad_finger_1_x;
        this.touchpadFinger1Y = gamepad.touchpad_finger_1_y;
        this.touchpadFinger2X = gamepad.touchpad_finger_2_x;
        this.touchpadFinger2Y = gamepad.touchpad_finger_2_y;
    }

    public final boolean a, b, x, y, cross, triangle, circle, square;
    public final boolean leftBumper, rightBumper;
    public final boolean dpadLeft, dpadRight, dpadUp, dpadDown;
    public final boolean leftStickButton, rightStickButton;
    public final boolean start, options, guide, share;
    public final boolean touchpad, touchpadFinger1, touchpadFinger2;

    public double leftTrigger, rightTrigger;
    public double leftStickX, leftStickY, rightStickX, rightStickY;
    public double touchpadFinger1X, touchpadFinger1Y, touchpadFinger2X, touchpadFinger2Y;

    public boolean aPressed(){
        return a && !baseLast.a;
    }

    public boolean bPressed(){
        return b && !baseLast.b;
    }

    public boolean xPressed(){
        return x && !baseLast.x;
    }

    public boolean yPressed(){
        return y && !baseLast.y;
    }

    public boolean leftBumperPressed(){
        return leftBumper && !baseLast.left_bumper;
    }

    public boolean rightBumperPressed(){
        return rightBumper && !baseLast.right_bumper;
    }

    public boolean dpadLeftPressed(){
        return dpadLeft && !baseLast.dpad_left;
    }

    public boolean dpadRightPressed(){
        return dpadRight && !baseLast.dpad_right;
    }

    public boolean dpadUpPressed(){
        return dpadUp && !baseLast.dpad_up;
    }

    public boolean dpadDownPressed(){
        return dpadDown && !baseLast.dpad_down;
    }

    public boolean leftStickButtonPressed(){
        return leftStickButton && !baseLast.left_stick_button;
    }

    public boolean rightStickButtonPressed(){
        return rightStickButton && !baseLast.right_stick_button;
    }

    public boolean startPressed(){
        return start && !baseLast.start;
    }
    public boolean sharePressed() {
        return share && !baseLast.share;
    }

    public boolean optionPressed(){
        return options && !baseLast.options;
    }

    public boolean guidePressed(){
        return guide && !baseLast.guide;
    }

    public boolean crossPressed(){
        return cross && !baseLast.cross;
    }

    public boolean trianglePressed(){
        return triangle && !baseLast.triangle;
    }

    public boolean circlePressed(){
        return circle && !baseLast.circle;
    }

    public boolean squarePressed(){
        return square && !baseLast.square;
    }

    public boolean aReleased(){
        return !a && baseLast.a;
    }

    public boolean bReleased(){
        return !b && baseLast.b;
    }

    public boolean xReleased(){
        return !x && baseLast.x;
    }

    public boolean yReleased(){
        return !y && baseLast.y;
    }

    public boolean leftBumperReleased(){
        return !leftBumper && baseLast.left_bumper;
    }

    public boolean rightBumperReleased(){
        return !rightBumper && baseLast.right_bumper;
    }

    public boolean dpadLeftReleased(){
        return !dpadLeft && baseLast.dpad_left;
    }

    public boolean dpadRightReleased(){
        return !dpadRight && baseLast.dpad_right;
    }

    public boolean dpadUpReleased(){
        return !dpadUp && baseLast.dpad_up;
    }

    public boolean dpadDownReleased(){
        return !dpadDown && baseLast.dpad_down;
    }

    public boolean leftStickButtonReleased(){
        return !leftStickButton && baseLast.left_stick_button;
    }

    public boolean rightStickButtonReleased(){
        return !rightStickButton && baseLast.right_stick_button;
    }

    public boolean startReleased(){
        return !start && baseLast.start;
    }

    public boolean optionReleased(){
        return !options && baseLast.options;
    }

    public boolean guideReleased(){
        return !guide && baseLast.guide;
    }

    public boolean crossReleased(){
        return !cross && baseLast.cross;
    }

    public boolean triangleReleased(){
        return !triangle && baseLast.triangle;
    }

    public boolean circleReleased(){
        return !circle && baseLast.circle;
    }

    public boolean squareReleased(){
        return !square && baseLast.square;
    }

    public boolean leftTriggerAtRest(){
        return leftTrigger == 0;
    }

    public boolean rightTriggerAtRest(){
        return rightTrigger == 0;
    }

    public boolean leftStickAtRest(){
        return leftStickX == 0 && leftStickY == 0;
    }

    public boolean rightStickAtRest(){
        return rightStickX == 0 && rightStickY == 0;
    }

    public boolean touchpadPressed() {
        return touchpad && !baseLast.touchpad;
    }

    public boolean touchpadReleased() {
        return !touchpad && baseLast.touchpad;
    }

    public boolean touchpadFinger1Pressed() {
        return touchpadFinger1 && !baseLast.touchpad_finger_1;
    }

    public boolean touchpadFinger1Released() {
        return !touchpadFinger1 && baseLast.touchpad_finger_1;
    }

    public boolean touchpadFinger2Pressed() {
        return touchpadFinger2 && !baseLast.touchpad_finger_2;
    }

    public boolean touchpadFinger2Released() {
        return !touchpadFinger2 && baseLast.touchpad_finger_2;
    }

    public double touchpadFinger1DeltaX() {
        if (!touchpadFinger1 || !baseLast.touchpad_finger_1) {
            return 0;
        }
        return touchpadFinger1X - baseLast.touchpad_finger_1_x;
    }

    public double touchpadFinger1DeltaY() {
        if (!touchpadFinger1 || !baseLast.touchpad_finger_1) {
            return 0;
        }
        return touchpadFinger1Y - baseLast.touchpad_finger_1_y;
    }

    public double touchpadFinger2DeltaX() {
        if (!touchpadFinger2 || !baseLast.touchpad_finger_2) {
            return 0;
        }
        return touchpadFinger2X - baseLast.touchpad_finger_2_x;
    }

    public double touchpadFinger2DeltaY() {
        if (!touchpadFinger2 || !baseLast.touchpad_finger_2) {
            return 0;
        }
        return touchpadFinger2Y - baseLast.touchpad_finger_2_y;
    }
}
