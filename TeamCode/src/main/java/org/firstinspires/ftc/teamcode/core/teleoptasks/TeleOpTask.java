package org.firstinspires.ftc.teamcode.core.teleoptasks;

public interface TeleOpTask {
    String name();

    void start(TeleOpTaskContext ctx);

    TaskResult tick(TeleOpTaskContext ctx);

    void cancel(TeleOpTaskContext ctx, CancelReason reason);

    void stop(TeleOpTaskContext ctx);

    default boolean onGuidePressed(TeleOpTaskContext ctx) {
        return false;
    }

    default String stateName() {
        return "RUNNING";
    }
}
