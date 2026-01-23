package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.drive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.Pose;

@Configurable
public class DriveBase {

    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private Follower follower;
    public static float TRANSLATIONAL_VELOCITY_MULTIPLIER = 40f;
    public static float HEADING_VELOCITY_MULTIPLIER = 3f;

    private double powerFactor = 1;

    public static DriveMotorPosition zeroIndex = DriveMotorPosition.LEFT_FRONT;
    public static DriveMotorPosition oneIndex = DriveMotorPosition.RIGHT_FRONT;
    public static DriveMotorPosition twoIndex = DriveMotorPosition.LEFT_REAR;
    public static DriveMotorPosition threeIndex = DriveMotorPosition.RIGHT_REAR;

    public enum DriveMotorPosition {
        LEFT_FRONT, RIGHT_FRONT,
        LEFT_REAR, RIGHT_REAR
    }
    private double mapPosition(DriveMotorPosition position, double lf, double rf, double lr, double rr){
        switch (position) {
            case LEFT_FRONT:
                return lf;
            case RIGHT_FRONT:
                return rf;
            case LEFT_REAR:
                return lr;
            case RIGHT_REAR:
                return rr;
        }
        return 0;
    }

    public DriveBase(HardwareMap hardwareMap, DriveBaseMotorConfig config, boolean startFollower) {
        if(startFollower){
            follower = Constants.createConfiguredFollower(hardwareMap, config);
            localizer = follower.getPoseTracker().getLocalizer();
            drivetrain = follower.getDrivetrain();
        } else {
            localizer = new PinpointLocalizer(hardwareMap, Constants.pinpointConstants);
            drivetrain = new Mecanum(hardwareMap, Constants.mecanumConstants);

            config.configAndFetchLeftFront(hardwareMap);
            config.configAndFetchLeftRear(hardwareMap);
            config.configAndFetchRightFront(hardwareMap);
            config.configAndFetchRightRear(hardwareMap);
        }
    }

    /**
     * Moves and turns the robot using general power modifiers in each direction/axis
     * @param x the power to move left/right with. Positive -> right, Negative -> left
     * @param y the power to move forward/back with. Positive -> forward, Negative -> backward
     * @param turn the power to turn with. Positive -> turn right, Negative -> turn left
     */
    public void moveUsingPP(double x, double y, double turn){
        follower.setTeleOpDrive(x, y, turn);
    }

    public void moveUsingPower(double x, double y, double turn){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the correct ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double leftFront = ((y - x - turn) / denominator) * powerFactor;
        double leftRear = ((y + x - turn) / denominator) * powerFactor;
        double rightFront = ((y + x + turn) / denominator) * powerFactor;
        double rightRear = ((y - x + turn) / denominator) * powerFactor;

        setMotorPowers(leftFront, leftRear, rightFront, rightRear);
    }

    public Pose getPoseSimple(){
        com.pedropathing.geometry.Pose pose = localizer.getPose();
        return new Pose(pose.getX(), pose.getY(),pose.getHeading());
    }

    /**
     * Stops all motors. This is a shortcut method for <code>driveBase.setMotorPowers(0, 0, 0, 0)</code>`.
     */
    public void stop(){
        setMotorPowers(0,0,0,0);
    }

    public void setMotorPowers(double lf, double lr, double rf, double rr){
        drivetrain.runDrive(new double[]{
                mapPosition(zeroIndex, lf, lr, rf, rr),
                mapPosition(oneIndex, lf, lr, rf, rr),
                mapPosition(twoIndex, lf, lr, rf, rr),
                mapPosition(threeIndex, lf, lr, rf, rr),
        });
    }

    public void setPowerFactor(double powerFactor){
        this.powerFactor = powerFactor;
    }
}
