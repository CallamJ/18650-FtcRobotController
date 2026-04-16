package org.firstinspires.ftc.teamcode.core.teleoptasks.tasks;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.components.mechanisms.DriveBase;
import org.firstinspires.ftc.teamcode.components.subsystems.FireControlSystem;
import org.firstinspires.ftc.teamcode.components.subsystems.StorageController;
import org.firstinspires.ftc.teamcode.core.teleoptasks.CancelReason;
import org.firstinspires.ftc.teamcode.core.teleoptasks.TaskResult;
import org.firstinspires.ftc.teamcode.core.teleoptasks.TeleOpTask;
import org.firstinspires.ftc.teamcode.core.teleoptasks.TeleOpTaskContext;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

public class FarFiringTask implements TeleOpTask {
    private enum State {
        IDLE,
        DRIVE_TO_BASE_INIT,
        DRIVE_TO_BASE_RUNNING,
        WAIT_FOR_FULL_RUNNING,
        RETURN_TO_START_INIT,
        RETURN_TO_START_RUNNING,
        WAIT_FOR_FCS_READY,
        PREPARE_FIRE_BURST,
        WAIT_FOR_FCS_READY_FOR_NEXT_LOAD,
        QUEUE_NEXT_LOAD,
        WAIT_FOR_STORAGE_IDLE,
        REPEAT_DELAY,
        COMPLETE,
        FAILED,
        CANCELED
    }

    private final double baseBlueXInches;
    private final double baseBlueYInches;
    private final double baseBlueHeadingDeg;
    private final double baseRedXInches;
    private final double baseRedYInches;
    private final double baseRedHeadingDeg;
    private final double driveToBaseTimeoutSec;
    private final double waitForFullTimeoutSec;
    private final double returnToStartTimeoutSec;
    private final double readyToFireTimeoutSec;
    private final double storageDrainTimeoutSec;
    private final double repeatDelaySec;

    private State state = State.IDLE;
    private Pose startPose;
    private Pose basePose;
    private double stateStartSec = 0;
    private double storageDrainStartSec = 0;
    private double waitForFullLastAddedSec = 0;
    private int waitForFullLastArtifactCount = 0;
    private int remainingGreenLoads = 0;
    private int remainingPurpleLoads = 0;

    public FarFiringTask(
            double baseBlueXInches,
            double baseBlueYInches,
            double baseBlueHeadingDeg,
            double baseRedXInches,
            double baseRedYInches,
            double baseRedHeadingDeg,
            double driveToBaseTimeoutSec,
            double waitForFullTimeoutSec,
            double returnToStartTimeoutSec,
            double readyToFireTimeoutSec,
            double storageDrainTimeoutSec,
            double repeatDelaySec
    ) {
        this.baseBlueXInches = baseBlueXInches;
        this.baseBlueYInches = baseBlueYInches;
        this.baseBlueHeadingDeg = baseBlueHeadingDeg;
        this.baseRedXInches = baseRedXInches;
        this.baseRedYInches = baseRedYInches;
        this.baseRedHeadingDeg = baseRedHeadingDeg;
        this.driveToBaseTimeoutSec = driveToBaseTimeoutSec;
        this.waitForFullTimeoutSec = waitForFullTimeoutSec;
        this.returnToStartTimeoutSec = returnToStartTimeoutSec;
        this.readyToFireTimeoutSec = readyToFireTimeoutSec;
        this.storageDrainTimeoutSec = storageDrainTimeoutSec;
        this.repeatDelaySec = repeatDelaySec;
    }

    @Override
    public String name() {
        return "GoToBaseWaitFullReturn";
    }

    @Override
    public void start(TeleOpTaskContext ctx) {
        Follower follower = requireFollower(ctx);
        StorageController storageController = requireStorage(ctx);
        FireControlSystem fcs = requireFcs(ctx);
        if (storageController == null) {
            throw new IllegalStateException("Storage controller unavailable");
        }
        fcs.startLauncher();
        Pose current = follower.getPose();
        startPose = new Pose(current.getX(), current.getY(), current.getHeading());
        basePose = getAllianceBasePose(ctx.allianceColor());
        state = State.DRIVE_TO_BASE_INIT;
        stateStartSec = ctx.runtimeSec();
    }

    @Override
    public TaskResult tick(TeleOpTaskContext ctx) {
        Follower follower = requireFollower(ctx);
        switch (state) {
            case DRIVE_TO_BASE_INIT: {
                followLine(follower, follower.getPose(), basePose);
                state = State.DRIVE_TO_BASE_RUNNING;
                stateStartSec = ctx.runtimeSec();
                return TaskResult.RUNNING;
            }
            case DRIVE_TO_BASE_RUNNING: {
                if (elapsed(ctx) > driveToBaseTimeoutSec) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                if (!follower.isBusy()) {
                    state = State.WAIT_FOR_FULL_RUNNING;
                    stateStartSec = ctx.runtimeSec();
                    initWaitForFullTracking(ctx);
                }
                return TaskResult.RUNNING;
            }
            case WAIT_FOR_FULL_RUNNING: {
                StorageController storageController = requireStorage(ctx);
                if (storageController == null) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                if (storageController.isFull()) {
                    state = State.RETURN_TO_START_INIT;
                    stateStartSec = ctx.runtimeSec();
                    return TaskResult.RUNNING;
                }

                int currentArtifactCount = getStoredArtifactCount(storageController);
                if (currentArtifactCount > waitForFullLastArtifactCount) {
                    waitForFullLastAddedSec = ctx.runtimeSec();
                }
                waitForFullLastArtifactCount = currentArtifactCount;

                boolean hasAnyArtifacts = currentArtifactCount > 0;
                boolean noRecentAdds = ctx.runtimeSec() - waitForFullLastAddedSec > waitForFullTimeoutSec;
                if (hasAnyArtifacts && noRecentAdds) {
                    state = State.RETURN_TO_START_INIT;
                    stateStartSec = ctx.runtimeSec();
                }
                return TaskResult.RUNNING;
            }
            case RETURN_TO_START_INIT: {
                followLine(follower, follower.getPose(), startPose);
                state = State.RETURN_TO_START_RUNNING;
                stateStartSec = ctx.runtimeSec();
                return TaskResult.RUNNING;
            }
            case RETURN_TO_START_RUNNING: {
                if (elapsed(ctx) > returnToStartTimeoutSec) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                if (!follower.isBusy()) {
                    state = State.WAIT_FOR_FCS_READY;
                    stateStartSec = ctx.runtimeSec();
                }
                return TaskResult.RUNNING;
            }
            case WAIT_FOR_FCS_READY: {
                FireControlSystem fcs = requireFcs(ctx);
                if (elapsed(ctx) > readyToFireTimeoutSec) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                if (fcs.isTurretAligned() && fcs.isLauncherSpun()) {
                    state = State.PREPARE_FIRE_BURST;
                    stateStartSec = ctx.runtimeSec();
                }
                return TaskResult.RUNNING;
            }
            case PREPARE_FIRE_BURST: {
                StorageController storageController = requireStorage(ctx);
                if (storageController == null) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                remainingGreenLoads = storageController.countGreen();
                remainingPurpleLoads = storageController.countPurple();
                storageDrainStartSec = ctx.runtimeSec();
                state = State.WAIT_FOR_FCS_READY_FOR_NEXT_LOAD;
                stateStartSec = ctx.runtimeSec();
                return TaskResult.RUNNING;
            }
            case WAIT_FOR_FCS_READY_FOR_NEXT_LOAD: {
                StorageController storageController = requireStorage(ctx);
                FireControlSystem fcs = requireFcs(ctx);
                if (storageController == null) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                if (ctx.runtimeSec() - storageDrainStartSec > storageDrainTimeoutSec) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                if (remainingGreenLoads + remainingPurpleLoads <= 0) {
                    state = State.REPEAT_DELAY;
                    stateStartSec = ctx.runtimeSec();
                    return TaskResult.RUNNING;
                }
                if (storageController.allTasksComplete() && fcs.isTurretAligned() && fcs.isLauncherSpun()) {
                    state = State.QUEUE_NEXT_LOAD;
                    stateStartSec = ctx.runtimeSec();
                }
                return TaskResult.RUNNING;
            }
            case QUEUE_NEXT_LOAD: {
                StorageController storageController = requireStorage(ctx);
                if (storageController == null) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                if (remainingGreenLoads > 0) {
                    storageController.loadGreen();
                    remainingGreenLoads--;
                } else if (remainingPurpleLoads > 0) {
                    storageController.loadPurple();
                    remainingPurpleLoads--;
                } else {
                    state = State.REPEAT_DELAY;
                    stateStartSec = ctx.runtimeSec();
                    return TaskResult.RUNNING;
                }
                state = State.WAIT_FOR_STORAGE_IDLE;
                stateStartSec = ctx.runtimeSec();
                return TaskResult.RUNNING;
            }
            case WAIT_FOR_STORAGE_IDLE: {
                StorageController storageController = requireStorage(ctx);
                if (storageController == null) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                if (ctx.runtimeSec() - storageDrainStartSec > storageDrainTimeoutSec) {
                    state = State.FAILED;
                    return TaskResult.FAILED;
                }
                if (storageController.allTasksComplete()) {
                    state = State.WAIT_FOR_FCS_READY_FOR_NEXT_LOAD;
                    stateStartSec = ctx.runtimeSec();
                }
                return TaskResult.RUNNING;
            }
            case REPEAT_DELAY: {
                if (elapsed(ctx) >= repeatDelaySec) {
                    state = State.DRIVE_TO_BASE_INIT;
                    stateStartSec = ctx.runtimeSec();
                }
                return TaskResult.RUNNING;
            }
            case COMPLETE:
                return TaskResult.COMPLETED;
            case FAILED:
            case CANCELED:
                return TaskResult.FAILED;
            case IDLE:
            default:
                state = State.FAILED;
                return TaskResult.FAILED;
        }
    }

    @Override
    public void cancel(TeleOpTaskContext ctx, CancelReason reason) {
        state = State.CANCELED;
        cleanupDriveControl(ctx);
    }

    @Override
    public void stop(TeleOpTaskContext ctx) {
        cleanupDriveControl(ctx);
    }

    @Override
    public boolean onGuidePressed(TeleOpTaskContext ctx) {
        if (state == State.WAIT_FOR_FULL_RUNNING) {
            state = State.RETURN_TO_START_INIT;
            stateStartSec = ctx.runtimeSec();
            return true;
        }
        return false;
    }

    @Override
    public String stateName() {
        return state.name();
    }

    private double elapsed(TeleOpTaskContext ctx) {
        return Math.max(0, ctx.runtimeSec() - stateStartSec);
    }

    private StorageController requireStorage(TeleOpTaskContext ctx) {
        return ctx.storageController();
    }

    private FireControlSystem requireFcs(TeleOpTaskContext ctx) {
        FireControlSystem fcs = ctx.simpleFcs();
        if (fcs == null) {
            throw new IllegalStateException("SimpleFCS unavailable");
        }
        return fcs;
    }

    private Follower requireFollower(TeleOpTaskContext ctx) {
        Follower follower = ctx.follower();
        if (follower == null) {
            throw new IllegalStateException("Follower unavailable");
        }
        return follower;
    }

    private Pose getAllianceBasePose(MatchStateStore.AllianceColor allianceColor) {
        if (allianceColor == MatchStateStore.AllianceColor.RED) {
            return new Pose(baseRedXInches, baseRedYInches, Math.toRadians(baseRedHeadingDeg));
        }
        return new Pose(baseBlueXInches, baseBlueYInches, Math.toRadians(baseBlueHeadingDeg));
    }

    private void followLine(Follower follower, Pose from, Pose to) {
        PathChain path = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(from.getX(), from.getY(), from.getHeading()),
                        new Pose(to.getX(), to.getY(), to.getHeading())
                ))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
        follower.followPath(path);
    }

    private void initWaitForFullTracking(TeleOpTaskContext ctx) {
        StorageController storageController = requireStorage(ctx);
        if (storageController == null) {
            waitForFullLastArtifactCount = 0;
            waitForFullLastAddedSec = ctx.runtimeSec();
            return;
        }
        waitForFullLastArtifactCount = getStoredArtifactCount(storageController);
        waitForFullLastAddedSec = ctx.runtimeSec();
    }

    private int getStoredArtifactCount(StorageController storageController) {
        return Math.max(0, storageController.countGreen()) + Math.max(0, storageController.countPurple());
    }

    private void cleanupDriveControl(TeleOpTaskContext ctx) {
        DriveBase driveBase = ctx.driveBase();
        Follower follower = ctx.follower();
        if (follower != null) {
            follower.breakFollowing();
            follower.startTeleopDrive(true);
            follower.setTeleOpDrive(0, 0, 0, true);
        }
        if (driveBase != null) {
            driveBase.stop();
        }
    }

}
