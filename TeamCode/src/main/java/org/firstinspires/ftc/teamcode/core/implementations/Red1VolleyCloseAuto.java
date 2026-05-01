package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.components.subsystems.IndexerStorage;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

import java.util.List;

@Autonomous(name = "1 - Red 1 Volley Close")
public class Red1VolleyCloseAuto extends AutoOpBase {
    private final Pose start = new Pose(54.7, -50.8, 2.225);
    private final Pose fire = new Pose(15, -15.7, -0.757);
    private final Pose move = new Pose(29.5, -53.5, -0.65);

    @Override
    protected void onInitialize() {
        if (storageController != null) {
            storageController.indexerStorage().setLeftContent(IndexerStorage.SlotContent.PURPLE);
            storageController.indexerStorage().setRightContent(IndexerStorage.SlotContent.GREEN);
            storageController.indexerStorage().setFrontContent(IndexerStorage.SlotContent.PURPLE);
        }
        if (driveBase != null && driveBase.getFollower() != null) {
            driveBase.getFollower().setStartingPose(start);
        }
    }

    @Override
    protected MatchStateStore.AllianceColor getAutonomousAllianceColor() {
        return MatchStateStore.AllianceColor.RED;
    }

    @Override
    protected List<StepSpec> buildPlan() {
        return plan(
                instantStep("Start Intake + Launcher", () -> {
                    if (collector != null) {
                        collector.setPower(1);
                    }
                    if (fcs == null || !AutonomousConfiguration.runFCS) {
                        throw new IllegalStateException("FCS unavailable for close auto launch control");
                    }
                    fcs.startLauncher();
                }),
                followPathStep("Drive To Fire", this::buildStartToFirePath, 8),
                waitUntilStep("Wait For FCS Ready", this::isFcsReady, 3),
                queuePatternWhenFcsReadyStep(
                        "Queue Load Pattern",
                        () -> detectTagOrDefault(DEFAULT_TAG_PATTERN),
                        12
                ),
                instantStep("Stop Intake + Launcher", () -> {
                    if (fcs != null) {
                        fcs.stopLauncher();
                    }
                    if (collector != null) {
                        collector.setPower(0);
                    }
                }),
                followPathStep("Drive To Move", this::buildFireToMovePath, 8)
        );
    }

    private com.pedropathing.paths.PathChain buildStartToFirePath() {
        Follower follower = driveBase != null ? driveBase.getFollower() : null;
        if (follower == null) {
            throw new IllegalStateException("Follower unavailable");
        }

        return autoPathBuilder(follower)
                .addPath(new BezierLine(start, fire))
                .setLinearHeadingInterpolation(start.getHeading(), fire.getHeading())
                .build();
    }

    private com.pedropathing.paths.PathChain buildFireToMovePath() {
        Follower follower = driveBase != null ? driveBase.getFollower() : null;
        if (follower == null) {
            throw new IllegalStateException("Follower unavailable");
        }

        return autoPathBuilder(follower)
                .addPath(new BezierLine(fire, move))
                .setLinearHeadingInterpolation(fire.getHeading(), move.getHeading())
                .build();
    }
}
