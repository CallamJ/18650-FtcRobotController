package org.firstinspires.ftc.teamcode.core.teleoptasks;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.core.teleoptasks.tasks.FarFiringTask;

@Configurable
public class TeleOpTaskManager {
    public static double farFiringTaskBaseBlueXIn = -55.5;
    public static double farFiringTaskBaseBlueYIn = -48;
    public static double farFiringTaskBaseBlueHeadingDeg = -105;
    public static double farFiringTaskBaseRedXIn = -55.5;
    public static double farFiringTaskBaseRedYIn = 48;
    public static double farFiringTaskBaseRedHeadingDeg = 105;
    public static double farFiringTaskDriveToBaseTimeoutSec = 4.0;
    public static double farFiringTaskWaitForFullTimeoutSec = 10.0;
    public static double farFiringTaskReturnTimeoutSec = 4.0;
    public static double farFiringTaskReadyToFireTimeoutSec = 4.0;
    public static double farFiringTaskStorageDrainTimeoutSec = 10.0;
    public static double farFiringTaskRepeatDelaySec = 0.5;
    public static double driverOverrideDeadband = 0.08;

    private final TeleOpTaskContext context;
    private TeleOpTask activeTask;
    private String lastExitReason = "NONE";

    public TeleOpTaskManager(TeleOpTaskContext context) {
        this.context = context;
    }

    public boolean start(TeleOpTask task) {
        if (task == null || activeTask != null) {
            return false;
        }
        try {
            activeTask = task;
            activeTask.start(context);
            return true;
        } catch (Exception e) {
            lastExitReason = CancelReason.START_REJECTED + ": " + e.getMessage();
            activeTask = null;
            context.warn("Task start rejected: " + e.getMessage());
            return false;
        }
    }

    public boolean startFarFiringTask() {
        return start(new FarFiringTask(
                farFiringTaskBaseBlueXIn,
                farFiringTaskBaseBlueYIn,
                farFiringTaskBaseBlueHeadingDeg,
                farFiringTaskBaseRedXIn,
                farFiringTaskBaseRedYIn,
                farFiringTaskBaseRedHeadingDeg,
                farFiringTaskDriveToBaseTimeoutSec,
                farFiringTaskWaitForFullTimeoutSec,
                farFiringTaskReturnTimeoutSec,
                farFiringTaskReadyToFireTimeoutSec,
                farFiringTaskStorageDrainTimeoutSec,
                farFiringTaskRepeatDelaySec
        ));
    }

    public void update() {
        if (activeTask == null) {
            return;
        }
        try {
            TaskResult result = activeTask.tick(context);
            if (result == TaskResult.COMPLETED) {
                finish("COMPLETED");
            } else if (result == TaskResult.FAILED) {
                finish("FAILED");
            }
        } catch (Exception e) {
            cancelActive(CancelReason.INTERNAL_ERROR);
            context.warn("Task runtime error: " + e.getMessage());
        }
    }

    public boolean onGuidePressed() {
        if (activeTask == null) {
            return false;
        }
        try {
            return activeTask.onGuidePressed(context);
        } catch (Exception e) {
            cancelActive(CancelReason.INTERNAL_ERROR);
            context.warn("Task guide action failed: " + e.getMessage());
            return false;
        }
    }

    public void cancelActive(CancelReason reason) {
        if (activeTask == null) {
            return;
        }
        try {
            activeTask.cancel(context, reason);
        } catch (Exception ignored) {
            context.warn("Task cancel failed cleanly");
        } finally {
            lastExitReason = reason.name();
            activeTask = null;
        }
    }

    public boolean hasActiveTask() {
        return activeTask != null;
    }

    public String activeTaskName() {
        return activeTask == null ? "NONE" : activeTask.name();
    }

    public String activeTaskState() {
        return activeTask == null ? "IDLE" : activeTask.stateName();
    }

    public String lastExitReason() {
        return lastExitReason;
    }

    private void finish(String reason) {
        if (activeTask == null) {
            return;
        }
        try {
            activeTask.stop(context);
        } catch (Exception ignored) {
            context.warn("Task stop failed cleanly");
        } finally {
            lastExitReason = reason;
            activeTask = null;
        }
    }
}
