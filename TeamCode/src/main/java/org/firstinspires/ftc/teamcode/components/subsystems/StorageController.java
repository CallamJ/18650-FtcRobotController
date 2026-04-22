package org.firstinspires.ftc.teamcode.components.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.mechanisms.Collector;
import org.firstinspires.ftc.teamcode.components.mechanisms.FeedWheels;
import org.firstinspires.ftc.teamcode.components.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.hardware.ScoringColorSensor;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.SmartLEDIndicator;

import java.util.*;

@Configurable
public class StorageController {
    public static int requiredConsecutiveContentReads = 3;

    private final FeedSystem feeder;
    private final Indexer indexer;
    private final Collector collector;
    private final ScoringColorSensor frontSensor;
    private final Queue<Task> taskQueue;
    private Task activeTask;
    private State state;
    private final SmartLEDIndicator leftLED, rightLED, frontLED;
    private boolean isJamCorrecting = false;
    private final ElapsedTime readyingTimer = new ElapsedTime();

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
    private ScoringElementColor lastFrontDetection = ScoringElementColor.NONE;
    private int frontDetectionStreak = 0;

    public StorageController(
            FeedSystem feeder,
            Indexer indexer,
            Collector collector,
            ScoringColorSensor frontSensor,
            SmartLEDIndicator leftLED,
            SmartLEDIndicator rightLED,
            SmartLEDIndicator frontLED
    ) {
        this.feeder = feeder;
        this.indexer = indexer;
        this.collector = collector;
        this.frontSensor = frontSensor;
        this.indexerContent = new SlotContent[]{SlotContent.OPEN, SlotContent.OPEN, SlotContent.OPEN};
        this.isFrontFresh = false;
        this.taskQueue = new ArrayDeque<>();
        this.state = State.RESTING;
        this.leftLED = leftLED;
        this.rightLED = rightLED;
        this.frontLED = frontLED;
    }

    private ScoringElementColor getFrontDetection() {
        if(frontSensor.getClosestColorMatchName().equals("ARTIFACT_PURPLE")){
            return ScoringElementColor.PURPLE;
        } else if(frontSensor.getClosestColorMatchName().equals("ARTIFACT_GREEN")){
            return ScoringElementColor.GREEN;
        }
        return ScoringElementColor.NONE;
    }

    public String getFrontClosestColorMatch() {
        return frontSensor.getClosestColorMatchName();
    }

    public String getFrontSensorColorName() {
        ScoringElementColor color = getFrontDetection();
        return color == null ? "NONE" : color.name();
    }

    public float getFrontSensorHue() {
        return frontSensor.getHSV()[0];
    }

    public float getFrontSensorSaturation() {
        return frontSensor.getHSV()[1];
    }

    public float getFrontSensorValue() {
        return frontSensor.getHSV()[2];
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

    public void tick(){
        indexer.tick();
        applyContentColor(leftLED, getLeftContent());
        applyContentColor(rightLED, getRightContent());
        applyContentColor(frontLED, getFrontContent());

        checkForIndexerJam();

        if(isJamCorrecting && (!indexer.isBusy() || activeJamTimer.milliseconds() > 1000)){
            this.isJamCorrecting = false;
            indexer.setTargetIndex(lastTarget);
            return;
        }

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
                    feeder.startFeeding();
                    this.state = State.LOADING_GREEN;
                }
                break;
            }
            case LOADING_GREEN:
            case LOADING_PURPLE: {
                indexer.advanceIndexCounterclockwise();
                this.state = State.BUMPING;
                break;
            }

        }
    }

    public void updateIndexerContent(){
        ScoringElementColor detectedColor = getFrontDetection();
        if (detectedColor == null) {
            detectedColor = ScoringElementColor.NONE;
        }
        if (detectedColor == lastFrontDetection) {
            frontDetectionStreak++;
        } else {
            lastFrontDetection = detectedColor;
            frontDetectionStreak = 1;
        }

        if (frontDetectionStreak < Math.max(1, requiredConsecutiveContentReads)) {
            return;
        }

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
//        detectedColor = rightSensor.getScoringElementColor();
//        switch (detectedColor) {
//            case GREEN: {
//                setRightContent(SlotContent.GREEN);
//                break;
//            }
//            case PURPLE: {
//                setRightContent(SlotContent.PURPLE);
//                break;
//            }
//            case NONE: {
//                setRightContent(SlotContent.OPEN);
//                break;
//            }
//        }
//        detectedColor = leftSensor.getScoringElementColor();
//        switch (detectedColor) {
//            case GREEN: {
//                setLeftContent(SlotContent.GREEN);
//                break;
//            }
//            case PURPLE: {
//                setLeftContent(SlotContent.PURPLE);
//                break;
//            }
//            case NONE: {
//                setLeftContent(SlotContent.OPEN);
//                break;
//            }
//        }
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
                        feeder.startFeeding();
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
                        feeder.startFeeding();
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

    private void applyContentColor(SmartLEDIndicator led, SlotContent content){
        switch(content){
            case OPEN: {
                led.setColor(SmartLEDIndicator.IndicatorColor.WHITE);
                break;
            }
            case GREEN: {
                led.setColor(SmartLEDIndicator.IndicatorColor.GREEN);
                break;
            }
            case PURPLE: {
                led.setColor(SmartLEDIndicator.IndicatorColor.VIOLET);
                break;
            }
        }
    }

    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime activeJamTimer = new ElapsedTime();
    private long lastTarget = 0;
    private void checkForIndexerJam(){
        if(indexer.isBusy() && Math.abs(indexer.getVelocity()) <= 25 && !isJamCorrecting){
            if(jamTimer.milliseconds() > 300){
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

    public enum SlotContent {
        OPEN,
        GREEN,
        PURPLE
    }

    // COMMAND CONTROL:
    public void clearCommandQueue(){
        taskQueue.clear();
    }

    public boolean allTasksComplete(){
        return activeTask == null && taskQueue.isEmpty();
    }

    public void loadGreen(){
        taskQueue.add(Task.LOAD_GREEN);
    }
    public void loadPurple(){
        taskQueue.add(Task.LOAD_PURPLE);
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
