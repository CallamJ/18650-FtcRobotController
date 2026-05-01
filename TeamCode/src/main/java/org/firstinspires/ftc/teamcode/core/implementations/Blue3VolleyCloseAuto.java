package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.subsystems.IndexerStorage;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Configurable
@Autonomous(name = "1 - Blue 3 Volley Close")
public class Blue3VolleyCloseAuto extends AutoOpBase {
    public static double START_X = 54.7;
    public static double START_Y = 50.8;
    public static double START_HEADING_RAD = -2.225;

    public static double FIRE_X = 15;
    public static double FIRE_Y = 15.7;
    public static double FIRE_HEADING_RAD = 0.757;

    public static double MOVE_OFF_LINE_X = 29.5;
    public static double MOVE_OFF_LINE_Y = 53.5;
    public static double MOVE_OFF_LINE_HEADING_RAD = 0.65;

    public static double SPIKE_1_APPROACH_X = 12.8;
    public static double SPIKE_1_APPROACH_Y = 25;
    public static double SPIKE_1_APPROACH_HEADING_RAD = 1.56;

    public static double SPIKE_2_APPROACH_X = -10.7;
    public static double SPIKE_2_APPROACH_Y = 25;
    public static double SPIKE_2_APPROACH_HEADING_RAD = 1.56;

    public static double SPIKE_3_APPROACH_X = -35.2;
    public static double SPIKE_3_APPROACH_Y = 25;
    public static double SPIKE_3_APPROACH_HEADING_RAD = 1.56;

    public static double CYCLE_START_CUTOFF_SEC = 21.0;
    public static double PATH_TIMEOUT_SEC = 8.0;
    public static double FCS_READY_TIMEOUT_SEC = 3.0;
    public static double STORAGE_WAIT_TIMEOUT_SEC = 8.0;
    public static double PATTERN_QUEUE_TIMEOUT_SEC = 12.0;
    public static double COLLECT_TIMEOUT_SEC = 4.0;

    public static double COLLECTOR_POWER = 1.0;
    public static double COLLECTOR_IDLE_POWER = 0.5;
    public static double COLLECT_FORWARD_POWER = 0.4;
    public static double COLLECT_FORWARD_SECONDS = 1.8;
    public static double COLLECT_SETTLE_SECONDS = 0.2;
    public static double COLLECT_BACKOUT_DISTANCE = 6.0;
    public static int MAX_COLLECTION_CYCLES = 3;

    private static final int TOTAL_SPIKE_CYCLES = 3;
    private final boolean[] cycleEnabled = new boolean[TOTAL_SPIKE_CYCLES];
    private boolean skipRemainingCycles = false;

    @Override
    protected void onInitialize() {
        if (storageController != null) {
            storageController.indexerStorage().setLeftContent(IndexerStorage.SlotContent.PURPLE);
            storageController.indexerStorage().setRightContent(IndexerStorage.SlotContent.GREEN);
            storageController.indexerStorage().setFrontContent(IndexerStorage.SlotContent.PURPLE);
        }

        Follower follower = driveBase != null ? driveBase.getFollower() : null;
        if (follower != null) {
            follower.setStartingPose(startPose());
        }
    }

    @Override
    protected MatchStateStore.AllianceColor getAutonomousAllianceColor() {
        return MatchStateStore.AllianceColor.BLUE;
    }

    @Override
    protected List<StepSpec> buildPlan() {
        Arrays.fill(cycleEnabled, false);
        skipRemainingCycles = false;

        List<StepSpec> steps = new ArrayList<>();

        // Preload volley (existing behavior)
        steps.add(instantStep("Start Intake + Launcher", () -> {
            if (collector != null) {
                collector.setPower(COLLECTOR_IDLE_POWER);
            }
            if (fcs == null || !AutonomousConfiguration.runFCS) {
                throw new IllegalStateException("FCS unavailable for close auto launch control");
            }
            fcs.startLauncher();
        }));
        steps.add(followPathStep("Drive To Fire", this::buildStartToFirePath, PATH_TIMEOUT_SEC));
        steps.add(waitUntilStep("Wait For FCS Ready", this::isFcsReady, FCS_READY_TIMEOUT_SEC));
        steps.add(queuePatternWhenFcsReadyStep(
                "Queue Preload Pattern",
                () -> detectTagOrDefault(DEFAULT_TAG_PATTERN),
                PATTERN_QUEUE_TIMEOUT_SEC
        ).onTimeoutContinue());

        addSpikeCollectionCycle(steps, 0, "Spike Mark 1", this::spike1ApproachPose);
        addSpikeCollectionCycle(steps, 1, "Spike Mark 2", this::spike2ApproachPose);
        addSpikeCollectionCycle(steps, 2, "Spike Mark 3", this::spike3ApproachPose);

        steps.add(instantStep("Stop Intake + Launcher", this::stopCollectorAndLauncher));
        steps.add(followPathStep("Move Off Launch Line", this::buildCurrentToMoveOffLinePath, PATH_TIMEOUT_SEC));

        return steps;
    }

    private void addSpikeCollectionCycle(List<StepSpec> steps, int cycleIndex, String label, Supplier<Pose> approachPoseSupplier) {
        steps.add(cycleGateStep(cycleIndex, label));
        steps.add(conditionalInstantStep(cycleIndex, "Prepare " + label + " Collect", () -> {
            if (fcs != null) {
                fcs.stopLauncher();
            }
            if (collector != null) {
                collector.setPower(COLLECTOR_IDLE_POWER);
            }
        }));
        steps.add(conditionalFollowPathStep(
                cycleIndex,
                "Drive To " + label + " Approach",
                () -> buildPathFromCurrentToPose(approachPoseSupplier.get()),
                PATH_TIMEOUT_SEC
        ));
        steps.add(conditionalCollectStep(cycleIndex, "Collect Through " + label, COLLECT_TIMEOUT_SEC));
        steps.add(conditionalWaitSecondsStep(cycleIndex, "Collect Settle " + label, COLLECT_SETTLE_SECONDS));
        steps.add(conditionalFollowPathStep(
                cycleIndex,
                "Back Out From " + label,
                () -> buildBackOutPathFromCurrent(COLLECT_BACKOUT_DISTANCE),
                PATH_TIMEOUT_SEC
        ));
        steps.add(conditionalFollowPathStep(
                cycleIndex,
                "Return To Fire From " + label,
                () -> buildPathFromCurrentToPose(firePose()),
                PATH_TIMEOUT_SEC
        ));
        steps.add(conditionalInstantStep(cycleIndex, "Start Launcher " + label, () -> {
            if (fcs == null || !AutonomousConfiguration.runFCS) {
                throw new IllegalStateException("FCS unavailable for close auto launch control");
            }
            fcs.startLauncher();
        }));
        steps.add(conditionalWaitUntilStep(cycleIndex, "Wait For FCS Ready " + label, this::isFcsReady, FCS_READY_TIMEOUT_SEC));
        steps.add(conditionalQueuePatternStep(cycleIndex, "Queue Pattern " + label));
    }

    private StepSpec cycleGateStep(int cycleIndex, String label) {
        return instantStep("Cycle Gate " + label, () -> {
            int clampedMaxCycles = Math.max(0, Math.min(MAX_COLLECTION_CYCLES, TOTAL_SPIKE_CYCLES));
            boolean enabled = cycleIndex < clampedMaxCycles
                    && !skipRemainingCycles
                    && getRuntime() < CYCLE_START_CUTOFF_SEC;
            cycleEnabled[cycleIndex] = enabled;
            if (!enabled) {
                skipRemainingCycles = true;
            }
        });
    }

    private StepSpec conditionalInstantStep(int cycleIndex, String name, Runnable action) {
        return StepSpec.required(new AutoStep() {
            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                if (shouldRunCycle(cycleIndex)) {
                    action.run();
                }
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                return StepStatus.COMPLETE;
            }
        }, 0);
    }

    private StepSpec conditionalFollowPathStep(
            int cycleIndex,
            String name,
            Supplier<PathChain> pathSupplier,
            double timeoutSec
    ) {
        return StepSpec.required(new AutoStep() {
            private Follower follower;
            private boolean runStep;

            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                runStep = shouldRunCycle(cycleIndex);
                if (!runStep) {
                    return;
                }
                follower = ctx.follower();
                if (follower == null) {
                    throw new IllegalStateException("Follower unavailable");
                }
                PathChain path = pathSupplier.get();
                if (path == null) {
                    throw new IllegalStateException("Path supplier returned null");
                }
                follower.followPath(path);
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                if (!runStep) {
                    return StepStatus.COMPLETE;
                }
                if (follower == null) {
                    return StepStatus.FAILED;
                }
                return follower.isBusy() ? StepStatus.RUNNING : StepStatus.COMPLETE;
            }
        }, timeoutSec);
    }

    private StepSpec conditionalWaitUntilStep(
            int cycleIndex,
            String name,
            BooleanSupplier condition,
            double timeoutSec
    ) {
        return StepSpec.required(new AutoStep() {
            private boolean runStep;

            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                runStep = shouldRunCycle(cycleIndex);
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                if (!runStep) {
                    return StepStatus.COMPLETE;
                }
                return condition.getAsBoolean() ? StepStatus.COMPLETE : StepStatus.RUNNING;
            }
        }, timeoutSec);
    }

    private StepSpec conditionalWaitSecondsStep(int cycleIndex, String name, double durationSec) {
        return StepSpec.required(new AutoStep() {
            private boolean runStep;
            private ElapsedTime timer;

            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                runStep = shouldRunCycle(cycleIndex);
                if (!runStep) {
                    return;
                }
                timer = new ElapsedTime();
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                if (!runStep) {
                    return StepStatus.COMPLETE;
                }
                if (timer == null) {
                    return StepStatus.FAILED;
                }
                return timer.seconds() >= durationSec ? StepStatus.COMPLETE : StepStatus.RUNNING;
            }
        }, durationSec + 0.5);
    }

    private StepSpec conditionalCollectStep(int cycleIndex, String name, double timeoutSec) {
        return StepSpec.required(new AutoStep() {
            private boolean runStep;
            private ElapsedTime timer;

            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                runStep = shouldRunCycle(cycleIndex);
                if (!runStep) {
                    return;
                }
                Follower follower = ctx.follower();
                if (follower != null) {
                    follower.breakFollowing();
                }
                timer = new ElapsedTime();
                if (collector != null) {
                    collector.setPower(COLLECTOR_POWER);
                }
                if (driveBase != null) {
                    driveBase.moveUsingPower(0, COLLECT_FORWARD_POWER, 0);
                } else {
                    throw new IllegalStateException("Drive base unavailable");
                }
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                if (!runStep) {
                    return StepStatus.COMPLETE;
                }
                if (timer == null) {
                    return StepStatus.FAILED;
                }
                if (collector != null) {
                    collector.setPower(COLLECTOR_POWER);
                }
                if (driveBase != null) {
                    driveBase.moveUsingPower(0, COLLECT_FORWARD_POWER, 0);
                }
                if (storageController != null && storageController.indexerStorage().isFull()) {
                    return StepStatus.COMPLETE;
                }
                return timer.seconds() >= COLLECT_FORWARD_SECONDS ? StepStatus.COMPLETE : StepStatus.RUNNING;
            }

            @Override
            public void stop(AutoContext ctx) {
                if (!runStep) {
                    return;
                }
                if (driveBase != null) {
                    driveBase.stop();
                }
                if (collector != null) {
                    collector.setPower(COLLECTOR_IDLE_POWER);
                }
            }
        }, timeoutSec);
    }

    private boolean shouldRunCycle(int cycleIndex) {
        return cycleIndex >= 0 && cycleIndex < cycleEnabled.length && cycleEnabled[cycleIndex];
    }

    private StepSpec conditionalQueuePatternStep(int cycleIndex, String name) {
        return StepSpec.required(new AutoStep() {
            private StepSpec delegateSpec;
            private AutoStep delegate;
            private boolean runStep;

            @Override
            public String name() {
                return name;
            }

            @Override
            public void start(AutoContext ctx) {
                runStep = shouldRunCycle(cycleIndex);
                if (!runStep) {
                    return;
                }
                delegateSpec = queuePatternWhenFcsReadyStep(
                        name,
                        () -> detectTagOrDefault(DEFAULT_TAG_PATTERN),
                        PATTERN_QUEUE_TIMEOUT_SEC
                );
                delegate = delegateSpec.step;
                if (delegate == null) {
                    throw new IllegalStateException("Queue pattern step delegate unavailable");
                }
                delegate.start(ctx);
            }

            @Override
            public StepStatus tick(AutoContext ctx) {
                if (!runStep) {
                    return StepStatus.COMPLETE;
                }
                if (delegate == null) {
                    return StepStatus.FAILED;
                }
                return delegate.tick(ctx);
            }

            @Override
            public void stop(AutoContext ctx) {
                if (!runStep || delegate == null) {
                    return;
                }
                delegate.stop(ctx);
            }
        }, PATTERN_QUEUE_TIMEOUT_SEC).onTimeoutContinue();
    }

    private void stopCollectorAndLauncher() {
        if (fcs != null) {
            fcs.stopLauncher();
        }
        if (collector != null) {
            collector.setPower(COLLECTOR_IDLE_POWER);
        }
        if (driveBase != null) {
            driveBase.stop();
        }
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

    private PathChain buildCurrentToMoveOffLinePath() {
        return buildPathFromCurrentToPose(moveOffLinePose());
    }

    private PathChain buildBackOutPathFromCurrent(double distanceInches) {
        Follower follower = driveBase != null ? driveBase.getFollower() : null;
        if (follower == null) {
            throw new IllegalStateException("Follower unavailable");
        }

        Pose current = follower.getPose();
        double heading = current.getHeading();
        double clampedDistance = Math.max(0, distanceInches);
        Pose target = new Pose(
                current.getX() - Math.cos(heading) * clampedDistance,
                current.getY() - Math.sin(heading) * clampedDistance,
                heading
        );
        return buildPathFromCurrentToPose(target);
    }

    private PathChain buildPathFromCurrentToPose(Pose targetPose) {
        Follower follower = driveBase != null ? driveBase.getFollower() : null;
        if (follower == null) {
            throw new IllegalStateException("Follower unavailable");
        }
        if (targetPose == null) {
            throw new IllegalStateException("Target pose unavailable");
        }

        Pose current = follower.getPose();
        Pose startPose = new Pose(current.getX(), current.getY(), current.getHeading());

        return autoPathBuilder(follower)
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();
    }

    private Pose startPose() {
        return new Pose(START_X, START_Y, START_HEADING_RAD);
    }

    private Pose firePose() {
        return new Pose(FIRE_X, FIRE_Y, FIRE_HEADING_RAD);
    }

    private Pose moveOffLinePose() {
        return new Pose(MOVE_OFF_LINE_X, MOVE_OFF_LINE_Y, MOVE_OFF_LINE_HEADING_RAD);
    }

    private Pose spike1ApproachPose() {
        return new Pose(SPIKE_1_APPROACH_X, SPIKE_1_APPROACH_Y, SPIKE_1_APPROACH_HEADING_RAD);
    }

    private Pose spike2ApproachPose() {
        return new Pose(SPIKE_2_APPROACH_X, SPIKE_2_APPROACH_Y, SPIKE_2_APPROACH_HEADING_RAD);
    }

    private Pose spike3ApproachPose() {
        return new Pose(SPIKE_3_APPROACH_X, SPIKE_3_APPROACH_Y, SPIKE_3_APPROACH_HEADING_RAD);
    }
}
