package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.utilities.Pose;

@Configurable
public class DriveBase {

    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private final Follower follower;
    public static float TRANSLATIONAL_VELOCITY_MULTIPLIER = 40f;
    public static float HEADING_VELOCITY_MULTIPLIER = 3f;

    private double powerFactor = 1;

    public DriveBase(HardwareMap hardwareMap, DriveBaseMotorConfig config) {
        MecanumConstants mConstants = new MecanumConstants();

        mConstants.leftFrontMotorName = config.leftFrontName;
        mConstants.rightFrontMotorName = config.rightFrontName;
        mConstants.leftRearMotorName = config.leftRearName;
        mConstants.rightRearMotorName = config.rightRearName;

        mConstants.leftFrontMotorDirection = config.leftFrontDirection.toMotorDirection();
        mConstants.rightFrontMotorDirection = config.rightFrontDirection.toMotorDirection();
        mConstants.leftRearMotorDirection = config.leftRearDirection.toMotorDirection();
        mConstants.rightRearMotorDirection = config.rightRearDirection.toMotorDirection();

        //todo: add values for drivetrain hardware constants

        drivetrain = new Mecanum(hardwareMap, mConstants);

        PinpointConstants pinpointConstants = new PinpointConstants();
        pinpointConstants.hardwareMapName = "pinpoint";
        //todo: add values for pinpoint system hardware constants
        localizer = new PinpointLocalizer(hardwareMap, pinpointConstants);


        FollowerConstants followerConstants = new FollowerConstants();

        //todo: tune follower constants

        follower = new Follower(followerConstants, localizer, drivetrain);
    }

    /**
     * Moves and turns the robot using general power modifiers in each direction/axis
     * @param x the power to move left/right with. Positive -> right, Negative -> left
     * @param y the power to move forward/back with. Positive -> forward, Negative -> backward
     * @param turn the power to turn with. Positive -> turn right, Negative -> turn left
     */
    public void moveUsingRR(double x, double y, double turn){
//        setDriveSignal(new DriveSignal(
//                new Pose2d(
//                TRANSLATIONAL_VELOCITY_MULTIPLIER * y,
//                TRANSLATIONAL_VELOCITY_MULTIPLIER * x,
//                HEADING_VELOCITY_MULTIPLIER * -turn
//                )
//        ));
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

        //setMotorPowers(leftFront, leftRear, rightRear, rightFront);
    }

    public Pose getPoseSimple(){
//        updatePoseEstimate();
//        return Pose.from(super.getPoseEstimate());
        return new Pose(0,0,0);
    }

    /**
     * Stops all motors. This is a shortcut method for <code>driveBase.setMotorPowers(0, 0, 0, 0)</code>`.
     */
    public void stop(){
        //setMotorPowers(0,0,0,0);
    }

    public void setPowerFactor(double powerFactor){
        this.powerFactor = powerFactor;
    }

}
