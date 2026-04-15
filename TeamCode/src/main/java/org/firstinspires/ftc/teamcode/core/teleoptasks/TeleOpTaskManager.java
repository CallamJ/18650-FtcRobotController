package org.firstinspires.ftc.teamcode.core.teleoptasks;

public class TeleOpTaskManager {
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
