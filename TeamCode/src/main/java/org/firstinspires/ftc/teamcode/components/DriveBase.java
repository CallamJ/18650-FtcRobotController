package org.firstinspires.ftc.teamcode.components;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.ConfiguredMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.Pose;

@Config
public class DriveBase extends ConfiguredMecanumDrive {
    public static float TRANSLATIONAL_VELOCITY_MULTIPLIER = 40f;
    public static float HEADING_VELOCITY_MULTIPLIER = 3f;

    private double powerFactor = 1;

    public DriveBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        setPoseEstimate(new Pose2d(0, 63, Math.toRadians(-90)));
    }

    /**
     * Moves and turns the robot using general power modifiers in each direction/axis
     * @param x the power to move left/right with. Positive -> right, Negative -> left
     * @param y the power to move forward/back with. Positive -> forward, Negative -> backward
     * @param turn the power to turn with. Positive -> turn right, Negative -> turn left
     */
    public void moveUsingRR(double x, double y, double turn){
        setDriveSignal(new DriveSignal(
                new Pose2d(
                TRANSLATIONAL_VELOCITY_MULTIPLIER * y,
                TRANSLATIONAL_VELOCITY_MULTIPLIER * x,
                HEADING_VELOCITY_MULTIPLIER * -turn
                )
        ));
    }

    public void moveUsingPower(double x, double y, double turn){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the correct ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double leftFront = ((y - x + turn) / denominator) * powerFactor;
        double leftRear = ((y + x + turn) / denominator) * powerFactor;
        double rightFront = ((y + x - turn) / denominator) * powerFactor;
        double rightRear = ((y - x - turn) / denominator) * powerFactor;

        setMotorPowers(leftFront, leftRear, rightRear, rightFront);
    }

    public Pose getPoseSimple(){
        updatePoseEstimate();
        return Pose.from(super.getPoseEstimate());
    }

    /**
     * Stops all motors. This is a shortcut method for <code>driveBase.setMotorPowers(0, 0, 0, 0)</code>`.
     */
    public void stop(){
        setMotorPowers(0,0,0,0);
    }

    public void setPowerFactor(double powerFactor){
        this.powerFactor = powerFactor;
    }

    public void followTrajectories(Trajectory... trajectories) {
        for (Trajectory trajectory : trajectories) {
            followTrajectory(trajectory);
        }
    }

    public void reverseEncoders(){
        ((StandardTrackingWheelLocalizer) getLocalizer()).reverseEncoders();
    }
}
