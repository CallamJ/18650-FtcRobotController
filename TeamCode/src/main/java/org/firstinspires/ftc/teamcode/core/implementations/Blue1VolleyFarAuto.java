package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.components.subsystems.StorageController;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

import java.util.ArrayList;
import java.util.List;

@Configurable
@Autonomous(name = "2 - Blue 1 Volley Far")
public class Blue1VolleyFarAuto extends AutoOpBase {
    public static double START_X = -60.9;
    public static double START_Y = 15.1;
    public static double START_HEADING_RAD = Math.toRadians(0.0);

    public static double FIRE_X = -60.9;
    public static double FIRE_Y = 15.1;
    public static double FIRE_HEADING_RAD = Math.toRadians(0.0);

    public static boolean DRIVE_TO_FIRE_POSE = false;
    public static double DRIVE_TO_FIRE_TIMEOUT_SEC = 8.0;
    public static double FCS_READY_TIMEOUT_SEC = 3.0;
    public static double PATTERN_QUEUE_TIMEOUT_SEC = 12.0;
    public static double FORWARD_AFTER_VOLLEY_INCHES = 24.0;
    public static double FORWARD_AFTER_VOLLEY_TIMEOUT_SEC = 4.0;

    @Override
    protected void onInitialize() {
        if (storageController != null) {
            storageController.setLeftContent(StorageController.SlotContent.PURPLE);
            storageController.setRightContent(StorageController.SlotContent.GREEN);
            storageController.setFrontContent(StorageController.SlotContent.PURPLE);
        }
        if (driveBase != null && driveBase.getFollower() != null) {
            driveBase.getFollower().setStartingPose(startPose());
        }
    }

    @Override
    protected MatchStateStore.AllianceColor getAutonomousAllianceColor() {
        return MatchStateStore.AllianceColor.BLUE;
    }

    @Override
    protected List<StepSpec> buildPlan() {
        List<StepSpec> steps = new ArrayList<>();
        steps.add(instantStep("Start Launcher", () -> {
            if (fcs == null || !runFCS) {
                throw new IllegalStateException("FCS unavailable for far auto launch control");
            }
            fcs.startLauncher();
        }));

        if (DRIVE_TO_FIRE_POSE) {
            steps.add(followPathStep("Drive To Fire", this::buildStartToFirePath, DRIVE_TO_FIRE_TIMEOUT_SEC));
        }

        steps.add(waitUntilStep("Wait For FCS Ready", this::isFcsReady, FCS_READY_TIMEOUT_SEC));
        steps.add(queuePatternWhenFcsReadyStep(
                "Queue Load Pattern",
                () -> detectTagOrDefault(DEFAULT_TAG_PATTERN),
                PATTERN_QUEUE_TIMEOUT_SEC
        ));
        steps.add(followPathStep(
                "Drive Forward 24in",
                this::buildForwardAfterVolleyPath,
                FORWARD_AFTER_VOLLEY_TIMEOUT_SEC
        ));
        steps.add(instantStep("Stop Launcher", () -> {
            if (fcs != null) {
                fcs.stopLauncher();
            }
        }));

        return steps;
    }

    private PathChain buildStartToFirePath() {
        Follower follower = driveBase != null ? driveBase.getFollower() : null;
        if (follower == null) {
            throw new IllegalStateException("Follower unavailable");
        }

        return autoPathBuilder(follower)
                .addPath(new BezierLine(startPose(), firePose()))
                .setLinearHeadingInterpolation(startPose().getHeading(), firePose().getHeading())
                .build();
    }

    private PathChain buildForwardAfterVolleyPath() {
        Follower follower = driveBase != null ? driveBase.getFollower() : null;
        if (follower == null) {
            throw new IllegalStateException("Follower unavailable");
        }

        Pose current = follower.getPose();
        double heading = current.getHeading();
        Pose target = new Pose(
                current.getX() + (Math.cos(heading) * FORWARD_AFTER_VOLLEY_INCHES),
                current.getY() + (Math.sin(heading) * FORWARD_AFTER_VOLLEY_INCHES),
                heading
        );

        Pose start = new Pose(current.getX(), current.getY(), heading);
        return autoPathBuilder(follower)
                .addPath(new BezierLine(start, target))
                .setLinearHeadingInterpolation(start.getHeading(), target.getHeading())
                .build();
    }

    private Pose startPose() {
        return new Pose(START_X, START_Y, START_HEADING_RAD);
    }

    private Pose firePose() {
        return new Pose(FIRE_X, FIRE_Y, FIRE_HEADING_RAD);
    }
}
