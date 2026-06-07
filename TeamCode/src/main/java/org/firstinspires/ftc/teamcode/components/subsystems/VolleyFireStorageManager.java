package org.firstinspires.ftc.teamcode.components.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.mechanisms.Collector;
import org.firstinspires.ftc.teamcode.components.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.utilities.LiveMatchTuning;

import java.util.ArrayDeque;
import java.util.Objects;
import java.util.Queue;

public class VolleyFireStorageManager {
    private final FeedSystem feeder;
    private final Indexer indexer;
    private final IndexerStorage indexerStorage;
    private final Queue<Task> taskQueue;
    private Task activeTask;
    private State state;
    private long lastFireIndex = 0;
    private final ElapsedTime firePrepareTimer = new ElapsedTime();
    private final FireControlSystem fcs;

    // jam correcting
        private boolean isJamCorrecting = false;
        private long lastTarget = 0;
        private final ElapsedTime jamTimer = new ElapsedTime(), activeJamTimer = new ElapsedTime();;



    public VolleyFireStorageManager(
            FeedSystem feeder,
            Indexer indexer,
            Collector collector,
            IndexerStorage indexerStorage,
            FireControlSystem fcs
    ) {
        this.feeder = Objects.requireNonNull(feeder, "feeder");
        this.indexer = Objects.requireNonNull(indexer, "indexer");
        this.indexerStorage = Objects.requireNonNull(indexerStorage, "indexerStorage");
        this.taskQueue = new ArrayDeque<>();
        this.state = State.RESTING;
        this.fcs = Objects.requireNonNull(fcs, "fcs");
    }

    public IndexerStorage indexerStorage() {
        return indexerStorage;
    }

    public void tick(){
        indexer.tick();
        indexerStorage.tick();

        if(LiveMatchTuning.volleyJamCorrectingEnabled){
            checkForIndexerJam();
        }

        if (isJamCorrecting && (!indexer.isBusy() || activeJamTimer.milliseconds() > LiveMatchTuning.volleyJamCorrectingTimeMs)) {
            isJamCorrecting = false;
            indexer.setTargetIndex(lastTarget);
            return;
        }

        switch (state){
            case RESTING: {
                activeTask = null;
                checkTasks();

                if (!indexer.isBusy()) {
                    indexerStorage.updateIndexerContent();
                }

                if(
                        taskQueue.isEmpty()
                        && activeTask == null
                        && indexerStorage.isFrontFresh()
                        && indexerStorage.isFrontFull()
                        && !indexerStorage.isFull()
                ){
                    readyForCollection();
                }
            } break;

            case BUMPING: {
                feeder.stopFeeding();
                if (!indexer.isBusy()) {
                    state = State.RESTING;
                    indexerStorage.dropFreshFlag();
                }
            } break;

            case PREPARING_TO_FIRE: {
                if (!indexer.isBusy()) {
                    state = State.ENGAGING_FEEDER;
                    indexerStorage.dropFreshFlag();
                    feeder.startFeeding();
                    firePrepareTimer.reset();

                }
            } break;

            case ENGAGING_FEEDER: {
                if(firePrepareTimer.milliseconds() > LiveMatchTuning.volleyFirePrepareTimeMs){
                    indexerStorage.dropFreshFlag();
                    state = State.FIRING;
                    lastFireIndex = indexer.getCurrentIndex();
                    indexer.advanceIndexCounterclockwiseWithPower(4, 1);
                    fcs.setFiring(true);
                }
            } break;

            case FIRING:{
                if(lastFireIndex != indexer.getCurrentIndex()){
                    indexerStorage.setRightContent(IndexerStorage.SlotContent.OPEN);
                }

                if(!indexer.isRunningPoweredMove()){
                    firePrepareTimer.reset();
                    state = State.ENDING_FIRING;
                }

                lastFireIndex = indexer.getCurrentIndex();
            } break;

            case ENDING_FIRING: {
                if(firePrepareTimer.milliseconds() > LiveMatchTuning.volleyFireEndTimeMs){
                    state = State.RESTING;
                    feeder.stopFeeding();
                    fcs.setFiring(false);
                }
            } break;
        }
    }

    private void checkForIndexerJam() {
        if (indexer.isBusy() && Math.abs(indexer.getVelocity()) <= LiveMatchTuning.volleyJamCorrectingVelocityThreshold && !isJamCorrecting) {
            if (jamTimer.milliseconds() > LiveMatchTuning.volleyJamCorrectingTimeThresholdMs) {
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
                    activeTask = null;
                    checkTasks();
                    return;
                }

                if(indexerStorage.hasPurple() && indexerStorage.hasGreen() && indexerStorage.isFull()) {
                    indexerStorage.readyContentToRight(IndexerStorage.SlotContent.GREEN);
                } else {
                    indexerStorage.readyContentToLeft(IndexerStorage.SlotContent.PURPLE);
                }
                state = State.PREPARING_TO_FIRE;
            } break;

            case FIRE_PGP: {
                if (indexerStorage.isEmpty()) {
                    activeTask = null;
                    checkTasks();
                    return;
                }

                if(indexerStorage.hasPurple() && indexerStorage.hasGreen() && indexerStorage.isFull()) {
                    indexerStorage.readyContentToFront(IndexerStorage.SlotContent.GREEN);
                } else {
                    indexerStorage.readyContentToLeft(IndexerStorage.SlotContent.PURPLE);
                }
                state = State.PREPARING_TO_FIRE;
            } break;

            case FIRE_GPP: {
                if (indexerStorage.isEmpty()) {
                    activeTask = null;
                    checkTasks();
                    return;
                }

                indexerStorage.readyContentToLeft(IndexerStorage.SlotContent.GREEN);
                state = State.PREPARING_TO_FIRE;
            } break;

            case FIRE_ANY: {
                if (indexerStorage.isEmpty()) {
                    activeTask = null;
                    checkTasks();
                    return;
                }

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
        if(
                !(taskQueue.contains(Task.FIRE_PPG) ||
                taskQueue.contains(Task.FIRE_PGP) ||
                taskQueue.contains(Task.FIRE_GPP) ||
                taskQueue.contains(Task.FIRE_ANY)
        )) {
            taskQueue.add(Task.FIRE_PPG);
        }
    }

    public void firePGP() {
        if(
                !(taskQueue.contains(Task.FIRE_PPG) ||
                        taskQueue.contains(Task.FIRE_PGP) ||
                        taskQueue.contains(Task.FIRE_GPP) ||
                        taskQueue.contains(Task.FIRE_ANY)
                )) {
            taskQueue.add(Task.FIRE_PGP);
        }
    }

    public void fireGPP() {
        if(
                !(taskQueue.contains(Task.FIRE_PPG) ||
                        taskQueue.contains(Task.FIRE_PGP) ||
                        taskQueue.contains(Task.FIRE_GPP) ||
                        taskQueue.contains(Task.FIRE_ANY)
        )) {
            taskQueue.add(Task.FIRE_GPP);
        }
    }

    public void fireAny() {
        if(
                !(taskQueue.contains(Task.FIRE_PPG) ||
                        taskQueue.contains(Task.FIRE_PGP) ||
                        taskQueue.contains(Task.FIRE_GPP) ||
                        taskQueue.contains(Task.FIRE_ANY)
        )) {
            taskQueue.add(Task.FIRE_ANY);
        }
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
        ENGAGING_FEEDER,
        FIRING,
        ENDING_FIRING
    }
}
