package org.firstinspires.ftc.teamcode.components.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.mechanisms.Collector;
import org.firstinspires.ftc.teamcode.components.mechanisms.Indexer;

import java.util.ArrayDeque;
import java.util.Queue;

public class SingleFireStorageManager {
    private final FeedSystem feeder;
    private final Indexer indexer;
    private final IndexerStorage indexerStorage;
    private final Queue<Task> taskQueue;
    private Task activeTask;
    private State state;
    private boolean isJamCorrecting = false;

    public SingleFireStorageManager(
            FeedSystem feeder,
            Indexer indexer,
            Collector collector,
            IndexerStorage indexerStorage
    ) {
        this.feeder = feeder;
        this.indexer = indexer;
        this.indexerStorage = indexerStorage;
        this.taskQueue = new ArrayDeque<>();
        this.state = State.RESTING;
    }

    public IndexerStorage indexerStorage() {
        return indexerStorage;
    }

    public String getFrontClosestColorMatch() {
        return indexerStorage.getFrontClosestColorMatch();
    }

    public String getFrontSensorColorName() {
        return indexerStorage.getFrontSensorColorName();
    }

    public float getFrontSensorHue() {
        return indexerStorage.getFrontSensorHue();
    }

    public float getFrontSensorSaturation() {
        return indexerStorage.getFrontSensorSaturation();
    }

    public float getFrontSensorValue() {
        return indexerStorage.getFrontSensorValue();
    }

    public State getState() {
        return state;
    }

    public String getActiveTaskName() {
        return activeTask == null ? "None" : activeTask.toString();
    }

    public boolean isJamCorrecting() {
        return isJamCorrecting;
    }

    public double getJamTimerMs() {
        return jamTimer.milliseconds();
    }

    public String getTaskQueueSummary() {
        return taskQueue.toString();
    }

    public void tick() {
        indexer.tick();
        indexerStorage.tick();

        checkForIndexerJam();

        if (isJamCorrecting && (!indexer.isBusy() || activeJamTimer.milliseconds() > 1000)) {
            isJamCorrecting = false;
            indexer.setTargetIndex(lastTarget);
            return;
        }

        switch (state) {
            case RESTING:
                activeTask = null;
                checkTasks();
                if (!indexer.isBusy()) {
                    indexerStorage.updateIndexerContent();
                    if (taskQueue.isEmpty() && activeTask == null && indexerStorage.isFrontFresh()) {
                        if (indexerStorage.getFrontContent() != IndexerStorage.SlotContent.OPEN && !indexerStorage.isFull()) {
                            taskQueue.add(Task.READY_FOR_COLLECTION);
                        }
                    }
                }
                break;
            case BUMPING:
                if (!indexer.isBusy()) {
                    state = State.RESTING;
                    indexerStorage.dropFreshFlag();
                }
                break;
            case READYING_GREEN:
            case READYING_PURPLE:
                if (!indexer.isBusy()) {
                    indexerStorage.dropFreshFlag();
                    feeder.startFeeding();
                    state = State.LOADING_GREEN;
                }
                break;
            case LOADING_GREEN:
            case LOADING_PURPLE:
                indexer.advanceIndexCounterclockwise();
                state = State.BUMPING;
                break;
        }
    }

    public void updateIndexerContent() {
        indexerStorage.updateIndexerContent();
    }

    public void checkTasks() {
        if (activeTask == null && !taskQueue.isEmpty()) {
            activeTask = taskQueue.poll();
        }

        if (activeTask == null) {
            return;
        }

        switch (activeTask) {
            case LOAD_PURPLE:
                if (indexerStorage.hasPurple()) {
                    if (indexerStorage.readyContentToLeft(IndexerStorage.SlotContent.PURPLE)) {
                        state = State.READYING_PURPLE;
                    } else {
                        feeder.startFeeding();
                        state = State.LOADING_PURPLE;
                    }
                } else {
                    activeTask = null;
                    checkTasks();
                }
                break;
            case LOAD_GREEN:
                if (indexerStorage.hasGreen()) {
                    if (indexerStorage.readyContentToLeft(IndexerStorage.SlotContent.GREEN)) {
                        state = State.READYING_GREEN;
                    } else {
                        feeder.startFeeding();
                        state = State.LOADING_GREEN;
                    }
                } else {
                    activeTask = null;
                    checkTasks();
                }
                break;
            case READY_FOR_COLLECTION:
                if (!indexerStorage.isFull()) {
                    state = indexerStorage.readyContentToFront(IndexerStorage.SlotContent.OPEN)
                            ? State.BUMPING
                            : State.RESTING;
                } else {
                    activeTask = null;
                    checkTasks();
                }
                break;
            case COUNTERCLOCKWISE_BUMP:
                indexer.advanceIndexCounterclockwise();
                state = State.BUMPING;
                break;
            case CLOCKWISE_BUMP:
                indexer.advanceIndexClockwise();
                state = State.BUMPING;
                break;
        }
    }

    public void bumpClockwise() {
        taskQueue.add(Task.CLOCKWISE_BUMP);
    }

    public void bumpCounterclockwise() {
        taskQueue.add(Task.COUNTERCLOCKWISE_BUMP);
    }

    public IndexerStorage.SlotContent getFrontContent() {
        return indexerStorage.getFrontContent();
    }

    public IndexerStorage.SlotContent getRightContent() {
        return indexerStorage.getRightContent();
    }

    public IndexerStorage.SlotContent getLeftContent() {
        return indexerStorage.getLeftContent();
    }

    public boolean hasGreen() {
        return indexerStorage.hasGreen();
    }

    public boolean hasPurple() {
        return indexerStorage.hasPurple();
    }

    public boolean isFull() {
        return indexerStorage.isFull();
    }

    public int countGreen() {
        return indexerStorage.countGreen();
    }

    public int countPurple() {
        return indexerStorage.countPurple();
    }

    public int countOpen() {
        return indexerStorage.countOpen();
    }

    public void setFrontContent(IndexerStorage.SlotContent content) {
        indexerStorage.setFrontContent(content);
    }

    public void setRightContent(IndexerStorage.SlotContent content) {
        indexerStorage.setRightContent(content);
    }

    public void setLeftContent(IndexerStorage.SlotContent content) {
        indexerStorage.setLeftContent(content);
    }

    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime activeJamTimer = new ElapsedTime();
    private long lastTarget = 0;

    private void checkForIndexerJam() {
        if (indexer.isBusy() && Math.abs(indexer.getVelocity()) <= 25 && !isJamCorrecting) {
            if (jamTimer.milliseconds() > 300) {
                jamTimer.reset();
                activeJamTimer.reset();
                lastTarget = indexer.getTargetIndex();
                long lastCurrent = indexer.getCurrentIndex();
                indexer.setTargetIndex(lastCurrent);
                isJamCorrecting = true;
            }
        } else {
            jamTimer.reset();
        }
    }

    public void clearCommandQueue() {
        taskQueue.clear();
    }

    public boolean allTasksComplete() {
        return activeTask == null && taskQueue.isEmpty();
    }

    public void loadGreen() {
        taskQueue.add(Task.LOAD_GREEN);
    }

    public void loadPurple() {
        taskQueue.add(Task.LOAD_PURPLE);
    }

    public void dropFreshFlag() {
        indexerStorage.dropFreshFlag();
    }

    public enum Task {
        LOAD_PURPLE,
        LOAD_GREEN,
        READY_FOR_COLLECTION,
        COUNTERCLOCKWISE_BUMP,
        CLOCKWISE_BUMP,
    }

    public enum State {
        RESTING,
        BUMPING,
        READYING_GREEN,
        LOADING_GREEN,
        READYING_PURPLE,
        LOADING_PURPLE,
    }
}
