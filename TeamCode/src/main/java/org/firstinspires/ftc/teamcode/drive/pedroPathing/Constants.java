package org.firstinspires.ftc.teamcode.drive.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.INCH)
            .strafePodX(-7.5)
            .forwardPodY(4.5)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftFrontMotorName("LFront")
            .rightFrontMotorName("RFront")
            .leftRearMotorName("LRear")
            .rightRearMotorName("RRear")
            .maxPower(1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, pinpointConstants);
        return new Follower(
                followerConstants,
                localizer,
                new Mecanum(hardwareMap, mecanumConstants)
        );
    }

    public static Follower createConfiguredFollower(HardwareMap hardwareMap, DriveBaseMotorConfig config) {
        config.configAndFetchLeftFront(hardwareMap);
        config.configAndFetchLeftRear(hardwareMap);
        config.configAndFetchRightFront(hardwareMap);
        config.configAndFetchRightRear(hardwareMap);
        return new Follower(
                followerConstants,
                new PinpointLocalizer(hardwareMap, pinpointConstants),
                new Mecanum(hardwareMap, mecanumConstants)
        );
    }
}
