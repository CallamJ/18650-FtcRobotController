package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.SmartColorSensor;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;
import org.firstinspires.ftc.teamcode.utilities.PrettyTelemetry;

import java.util.*;
import java.util.concurrent.ExecutionException;

@Configurable
public class StorageController {
    private final Feeder feeder;
    private final Indexer indexer;
    private final Collector collector;
    private final SmartColorSensor frontSensor;
    private final Queue<Task> taskQueue;
    private Task activeTask;
    private State state;

    /**
     * Slot 0: The slot that faces forward when the opmode is started.<br>
     * Slot 1: The slot that faces back-right when the opmode is started.<br>
     * Slot 2: The slot that faces back-left when the opmode is started.<br>
     *
     * Slots indexes do not change as the mixer moves, instead they refer to the position on the mixer itself.
     */
    private final SlotContent[] indexerContent;
    /**
     * When the front color sensor detects a new artifact (not result of rotating an existing artifact to the front) it is flagged as fresh.
     * If auto-advance triggers to move a slot away from the front, or the indexer is otherwise bumped it flags it as no longer fresh.
     */
    private boolean isFrontFresh;

    public StorageController(
            Feeder feeder,
            Indexer indexer,
            Collector collector,
            SmartColorSensor frontSensor
    ) {
        this.feeder = feeder;
        this.indexer = indexer;
        this.collector = collector;
        this.frontSensor = frontSensor;
        this.indexerContent = new SlotContent[]{SlotContent.OPEN, SlotContent.OPEN, SlotContent.OPEN};
        this.isFrontFresh = false;
        this.taskQueue = new ArrayDeque<>();
        this.state = State.RESTING;

        PrettyTelemetry.Item item = OpModeCore.getTelemetry().addLine("Storage Controller")
                .addData("State", () -> this.state.toString())
                .addData("Front Content", this::getFrontContent)
                .addData("Right Content", this::getRightContent)
                .addData("Left Content", this::getLeftContent)
                .addData("Active Task", () -> this.activeTask == null ? "None" : this.activeTask.toString())
                .addData("Task Queue", taskQueue::toString);

        OpModeCore.getTelemetry().addLine("Color Sensor")
                .addData("Sensor Color", frontSensor::getScoringElementColor)
                .addData("Sensor Hue", () -> frontSensor.getHSV()[0])
                .addData("Sensor Saturation", () -> frontSensor.getHSV()[1])
                .addData("Sensor Value", () -> frontSensor.getHSV()[2]);
    }

    public void tick(){
        feeder.tick();
        indexer.tick();
        switch(state){
            case RESTING: {
                activeTask = null;
                checkTasks();
                if(!indexer.isBusy()){
                    updateIndexerContent();
                    // if we aren't busy and the collection slot is full, make room.
                    if(taskQueue.isEmpty() && activeTask == null && isFrontFresh){
                        if(getFrontContent() != SlotContent.OPEN && !isFull()){
                            taskQueue.add(Task.READY_FOR_COLLECTION);
                        }
                    }
                }

                break;
            }
            case BUMPING: {
                if(!indexer.isBusy()){
                    this.state = State.RESTING;
                    this.isFrontFresh = false;
                }
                break;
            }
            case READYING_GREEN:
            case READYING_PURPLE: {
                if(!indexer.isBusy()){
                    this.isFrontFresh = false;
                    feeder.trigger();
                    this.state = State.LOADING_GREEN;
                }
                break;
            }
            case LOADING_GREEN:
            case LOADING_PURPLE: {
                if(feeder.getState() == Feeder.State.RESTING){
                    this.state = State.RESTING;
                    try {
                        if(feeder.triggerFuture != null && feeder.triggerFuture.get() < 2000){
                            setLeftContent(SlotContent.OPEN);
                        }
                    } catch (InterruptedException | ExecutionException e) {
                        throw new RuntimeException(e);
                    }
                }
                break;
            }

        }
    }

    public void updateIndexerContent(){
        ScoringElementColor detectedColor = frontSensor.getScoringElementColor();
        switch (detectedColor) {
            case GREEN: {
                if(getFrontContent() == SlotContent.OPEN){
                    this.isFrontFresh = true;
                }
                setFrontContent(SlotContent.GREEN);
                break;
            }
            case PURPLE: {
                if(getFrontContent() == SlotContent.OPEN){
                    this.isFrontFresh = true;
                }
                setFrontContent(SlotContent.PURPLE);
                break;
            }
            case NONE: {
                setFrontContent(SlotContent.OPEN);
                break;
            }
        }
    }

    public void checkTasks(){
        if (activeTask == null && !taskQueue.isEmpty()) {
            activeTask = taskQueue.poll(); // this grabs removes the oldest pending task
        }

        if(activeTask == null) return;

        switch (activeTask) {
            // check we actually have a purple to load
            case LOAD_PURPLE: {
                if(hasPurple()){
                    if(readyContentToLeft(SlotContent.PURPLE)){
                        // Need to rotate first
                        this.state = State.READYING_PURPLE;
                    } else {
                        // Already in position, trigger feeder immediately
                        feeder.trigger();
                        this.state = State.LOADING_PURPLE;
                    }
                } else {
                    activeTask = null;
                    checkTasks();
                }
                break;
            }
            case LOAD_GREEN: {
                if(hasGreen()){
                    if(readyContentToLeft(SlotContent.GREEN)){
                        // Need to rotate first
                        this.state = State.READYING_GREEN;
                    } else {
                        // Already in position, trigger feeder immediately
                        feeder.trigger();
                        this.state = State.LOADING_GREEN;
                    }
                } else {
                    activeTask = null;
                    checkTasks();
                }
                break;
            }
            case READY_FOR_COLLECTION: {
                // check if we have space to make room for a new artifact
                if(!isFull()){
                    // if front is already open, just skip to resting
                    this.state = readyContentToFront(SlotContent.OPEN) ? State.BUMPING : State.RESTING;
                } else {
                    // skip this task and check for more tasks
                    activeTask = null;
                    checkTasks();
                }
                break;
            }
            case COUNTERCLOCKWISE_BUMP: {
                indexer.advanceIndexCounterclockwise();
                this.state = State.BUMPING;
                break;
            }
            case CLOCKWISE_BUMP: {
                indexer.advanceIndexClockwise();
                this.state = State.BUMPING;
                break;
            }
        }
    }

    public void bumpClockwise(){
        taskQueue.add(Task.CLOCKWISE_BUMP);
    }

    public void bumpCounterclockwise(){
        taskQueue.add(Task.COUNTERCLOCKWISE_BUMP);
    }

    private boolean readyContentToFront(SlotContent target){
        if(getFrontContent() == target){
            return false;
        } else if(getLeftContent() == target){
            indexer.advanceIndexClockwise();
            return true;
        } else if(getRightContent() == target){
            indexer.advanceIndexCounterclockwise();
            return true;
        }
        return false;
    }

    private boolean readyContentToLeft(SlotContent target){
        if(getLeftContent() == target){
            return false;
        } else if(getRightContent() == target){
            indexer.advanceIndexClockwise();
            return true;
        } else if(getFrontContent() == target){
            indexer.advanceIndexCounterclockwise();
            return true;
        }
        return false;
    }

    private boolean readyContentToRight(SlotContent target){
        if(getRightContent() == target){
            return false;
        } else if(getFrontContent() == target){
            indexer.advanceIndexClockwise();
            return true;
        } else if(getLeftContent() == target){
            indexer.advanceIndexCounterclockwise();
            return true;
        }
        return false;
    }

    // STORAGE STATE ACCESSORS:

    public SlotContent getFrontContent(){
        return indexerContent[indexer.getNormalizedCurrentIndex()];
    }

    public SlotContent getRightContent(){
        return indexerContent[(indexer.getNormalizedCurrentIndex() + 2) % 3];
    }

    public SlotContent getLeftContent(){
        return indexerContent[(indexer.getNormalizedCurrentIndex() + 1) % 3];
    }

    public boolean hasGreen(){
        return indexerContent[2] == SlotContent.GREEN || indexerContent[1] == SlotContent.GREEN || indexerContent[0] == SlotContent.GREEN;
    }

    public boolean hasPurple(){
        return indexerContent[2] == SlotContent.PURPLE || indexerContent[1] == SlotContent.PURPLE || indexerContent[0] == SlotContent.PURPLE;
    }

    public boolean isFull(){
        return !(indexerContent[2] == SlotContent.OPEN ||  indexerContent[1] == SlotContent.OPEN || indexerContent[0] == SlotContent.OPEN);
    }

    public int countGreen(){
        return (int) Arrays.stream(indexerContent).filter(slotContent -> slotContent == SlotContent.GREEN).count();
    }

    public int countPurple(){
        return (int) Arrays.stream(indexerContent).filter(slotContent -> slotContent == SlotContent.PURPLE).count();
    }

    public int countOpen(){
        return (int) Arrays.stream(indexerContent).filter(slotContent -> slotContent == SlotContent.OPEN).count();
    }

    // STORAGE STATE MUTATORS:

    public void setFrontContent(SlotContent content){
        indexerContent[indexer.getNormalizedCurrentIndex()] = content;
    }

    public void setRightContent(SlotContent content){
        indexerContent[(indexer.getNormalizedCurrentIndex() + 2) % 3] = content;
    }

    public void setLeftContent(SlotContent content){
        indexerContent[(indexer.getNormalizedCurrentIndex() + 1) % 3] = content;
    }

    public enum SlotContent {
        OPEN,
        GREEN,
        PURPLE
    }

    // COMMAND CONTROL:
    public void clearCommandQueue(){
        taskQueue.clear();
    }

    public ChainedFuture<Object> loadGreen(){
        ChainedFuture<Object> future = new ChainedFuture<>();

        taskQueue.add(Task.LOAD_GREEN);

        return future;
    }
    public ChainedFuture<Object> loadPurple(){
        ChainedFuture<Object> future = new ChainedFuture<>();

        taskQueue.add(Task.LOAD_PURPLE);

        return future;
    }

    public void dropFreshFlag(){
        this.isFrontFresh = false;
    }

    /**
     * External task given to the storage controller to complete.
     */
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
