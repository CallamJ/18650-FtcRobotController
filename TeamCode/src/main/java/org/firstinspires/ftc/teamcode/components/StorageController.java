package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.SmartColorSensor;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;

import java.util.*;

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
     * slot 2: The slot that faces back-left when the opmode is started.<br>
     */
    private final SlotContent[] indexerContent;

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
        this.taskQueue = new ArrayDeque<>();
        this.state = State.RESTING;

        OpModeCore.getTelemetry().addLine("Storage Controller")
                .addData("State", () -> this.state.toString())
                .addData("Front Content", this::getFrontContent)
                .addData("Right Content", this::getRightContent)
                .addData("Left Content", this::getLeftContent);
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
                updateIndexerContent();
                checkTasks();

                // if we aren't busy and the collection slot is full, make room.
                if(taskQueue.isEmpty()){
                    if(getFrontContent() != SlotContent.OPEN && !isFull()){
                        taskQueue.add(Task.READY_FOR_COLLECTION);
                    }
                }

                break;
            }
            case BUMPING: {
                if(!indexer.isBusy()){
                    this.state = State.RESTING;
                }
                break;
            }
            case READYING_GREEN:
            case READYING_PURPLE: {
                if(!indexer.isBusy()){
                    feeder.trigger();
                    this.state = State.LOADING_GREEN;
                }
                break;
            }
            case LOADING_GREEN:
            case LOADING_PURPLE: {
                if(feeder.getState() == Feeder.State.RESTING){
                    this.state = State.RESTING;
                }
                break;
            }

        }
    }

    public void updateIndexerContent(){
        ScoringElementColor detectedColor = frontSensor.getScoringElementColor();
        if(
                detectedColor != ScoringElementColor.NONE &&
                indexer.getTargetIndex() == indexer.getCurrentIndex()
        ){
            switch (detectedColor) {
                case GREEN: {
                    setFrontContent(SlotContent.GREEN);
                    break;
                }
                case PURPLE: {
                    setFrontContent(SlotContent.PURPLE);
                    break;
                }
            }
        }
    }

    public void checkTasks(){
        if (activeTask == null && !taskQueue.isEmpty()) {
            activeTask = taskQueue.poll(); // this removes the oldest pending task
        }

        if(activeTask == null) return;

        switch (activeTask) {
            // check we actually have a purple to load
            case LOAD_PURPLE: {
                if(hasPurple()){
                    // if we don't have to move the indexer we can skip straight to loading phase
                    this.state = readyContentToLeft(SlotContent.PURPLE) ? State.READYING_PURPLE : State.LOADING_PURPLE;
                }
                break;
            }
            case LOAD_GREEN: {
                // check we actually have a green to load
                if(hasGreen()){
                    // if we don't have to move the indexer we can skip straight to loading phase
                    this.state = readyContentToLeft(SlotContent.GREEN) ? State.READYING_GREEN : State.LOADING_GREEN;
                }
                break;
            }
            case READY_FOR_COLLECTION: {
                // check if we have space to make room for a new artifact
                if(!isFull()){
                    // if front is already open, just skip to resting
                    this.state = readyContentToFront(SlotContent.OPEN) ? State.BUMPING : State.RESTING;
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
