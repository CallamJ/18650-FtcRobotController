package org.firstinspires.ftc.teamcode.components.subsystems;

import org.firstinspires.ftc.teamcode.components.mechanisms.Collector;
import org.firstinspires.ftc.teamcode.components.mechanisms.Indexer;

import java.util.ArrayDeque;
import java.util.Queue;

public class VolleyFireStorageManager {
    private final FeedSystem feeder;
    private final Indexer indexer;
    private final IndexerStorage indexerStorage;
    private final Queue<Task> taskQueue;
    private Task activeTask;
    private State state;
    private long lastFireIndex = 0;

    public VolleyFireStorageManager(
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

    public void tick(){
        indexer.tick();
        indexerStorage.tick();

        switch (state){
            case RESTING: {
                activeTask = null;
                checkTasks();

                if (!indexer.isBusy()) {
                    indexerStorage.updateIndexerContent();
                }

                if(taskQueue.isEmpty() && activeTask == null && indexerStorage.isFrontFresh()){
                    readyForCollection();
                }
            } break;

            case BUMPING: {
                if (!indexer.isBusy()) {
                    state = State.RESTING;
                    indexerStorage.dropFreshFlag();
                }
            } break;

            case PREPARING_TO_FIRE:{
                if (!indexer.isBusy()) {
                    state = State.FIRING;
                    indexerStorage.dropFreshFlag();
                    feeder.startFeeding();
                    lastFireIndex = indexer.getCurrentIndex();
                    indexer.advanceIndexCounterclockwiseWithPower(4, 1);
                }
            } break;

            case FIRING:{
                if(lastFireIndex != indexer.getCurrentIndex()){
                    indexerStorage.setRightContent(IndexerStorage.SlotContent.OPEN);
                }

                if(!indexer.isRunningPoweredMove()){
                    feeder.stopFeeding();
                    state = State.RESTING;
                }

                lastFireIndex = indexer.getCurrentIndex();
            } break;
        }
    }

    public void checkTasks() {
        if (activeTask == null && !taskQueue.isEmpty()) {
            activeTask = taskQueue.poll();
        }

        if (activeTask == null) {
            return;
        }

        switch (activeTask) {
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

            case FIRE_PPG: {
                if (indexerStorage.isEmpty()) {
                    break;
                }

                if(indexerStorage.hasPurple() && indexerStorage.hasGreen() && indexerStorage.isFull()) {
                    indexerStorage.readyContentToRight(IndexerStorage.SlotContent.GREEN);
                } else {
                    indexerStorage.readyContentToLeft(IndexerStorage.SlotContent.PURPLE);
                }
                state = State.PREPARING_TO_FIRE;
            } break;

            case FIRE_PGP: {
                if(indexerStorage.hasPurple() && indexerStorage.hasGreen() && indexerStorage.isFull()) {
                    indexerStorage.readyContentToFront(IndexerStorage.SlotContent.GREEN);
                } else {
                    indexerStorage.readyContentToLeft(IndexerStorage.SlotContent.PURPLE);
                }
                state = State.PREPARING_TO_FIRE;
            } break;

            case FIRE_GPP: {
                indexerStorage.readyContentToLeft(IndexerStorage.SlotContent.GREEN);
                state = State.PREPARING_TO_FIRE;
            } break;

            case FIRE_ANY: {
                state = State.PREPARING_TO_FIRE;
            } break;
        }
    }

    public State getState() {
        return state;
    }

    public String getActiveTaskName() {
        return activeTask == null ? "None" : activeTask.toString();
    }

    public String getTaskQueueSummary() {
        return taskQueue.toString();
    }

    public void clearCommandQueue() {
        taskQueue.clear();
    }

    public boolean allTasksComplete() {
        return activeTask == null && taskQueue.isEmpty();
    }

    public void bumpClockwise() {
        taskQueue.add(Task.CLOCKWISE_BUMP);
    }

    public void bumpCounterclockwise() {
        taskQueue.add(Task.COUNTERCLOCKWISE_BUMP);
    }

    public void readyForCollection() {
        taskQueue.add(Task.READY_FOR_COLLECTION);
    }

    public void firePPG() {
        taskQueue.add(Task.FIRE_PPG);
    }

    public void firePGP() {
        taskQueue.add(Task.FIRE_PGP);
    }

    public void fireGPP() {
        taskQueue.add(Task.FIRE_GPP);
    }

    public void fireAny() {
        taskQueue.add(Task.FIRE_ANY);
    }

    public void dropFreshFlag() {
        indexerStorage.dropFreshFlag();
    }

    /**
     * External task given to the storage controller to complete.
     */
    public enum Task {
        READY_FOR_COLLECTION,
        COUNTERCLOCKWISE_BUMP,
        CLOCKWISE_BUMP,
        FIRE_PPG,
        FIRE_PGP,
        FIRE_GPP,
        FIRE_ANY
    }

    public enum State {
        RESTING,
        BUMPING,
        PREPARING_TO_FIRE,
        FIRING
    }
}
