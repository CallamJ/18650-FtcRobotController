package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.SmartColorSensor;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;
import org.firstinspires.ftc.teamcode.utilities.TaskScheduler;

import java.util.Arrays;

@Configurable
public class StorageController {
    private final Feeder feeder;
    private final Indexer indexer;
    private final Collector collector;
    private final SmartColorSensor frontSensor;

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

        OpModeCore.getTelemetry().addLine("Storage Controller")
                .addData("Sensor Color", frontSensor::getScoringElementColor)
                .addData("Sensor Hue", () -> frontSensor.getHSV()[0])
                .addData("Sensor Saturation", () -> frontSensor.getHSV()[1])
                .addData("Sensor Value", () -> frontSensor.getHSV()[2])
                .addData("Front Content", this::getFrontContent)
                .addData("Right Content", this::getRightContent)
                .addData("Left Content", this::getLeftContent);
    }

    public void tick(){
        feeder.tick();
        indexer.tick();
        updateIndexerContent();
        checkAutomaticAdvance();
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

    public void checkAutomaticAdvance(){
        if(getLeftContent() == SlotContent.OPEN && getFrontContent() != SlotContent.OPEN && indexer.getTargetIndex() == indexer.getCurrentIndex()){
            advanceIndexerClockwise();
        }
    }

    public void advanceIndexerClockwise(){
        advanceIndexerClockwise(1);
    }

    public void advanceIndexerClockwise(int count){
        indexer.setTargetIndex(indexer.getCurrentIndex() + count);
    }

    public void advanceIndexerCounterclockwise(){
        advanceIndexerCounterclockwise(1);
    }

    public void advanceIndexerCounterclockwise(int count){
        indexer.setTargetIndex(indexer.getCurrentIndex() - count);
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

    public SlotContent[] getIndexerContent() {
        return indexerContent;
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

    public enum SlotContent {
        OPEN,
        GREEN,
        PURPLE
    }

    // STORAGE CONTROL:

    public ChainedFuture<Object> loadGreen(){
        ChainedFuture<Object> future = new ChainedFuture<>();

        if(!hasGreen()){
            future.complete(null);
            return future;
        }

        TaskScheduler.getDefaultInstance().runAsync(() -> {
            if(getLeftContent() == SlotContent.GREEN){
                triggerAndComplete(future);
            } else if(getRightContent() == SlotContent.GREEN){
                advanceIndexerClockwise();
                indexer.noLongerBusyNotifier.await();
                triggerAndComplete(future);
            } else if(getFrontContent() == SlotContent.GREEN){
                advanceIndexerCounterclockwise();
                indexer.noLongerBusyNotifier.await();
                triggerAndComplete(future);
            }
        });

        return future;
    }
    public ChainedFuture<Object> loadPurple(){
        ChainedFuture<Object> future = new ChainedFuture<>();

        if(!hasPurple()){
            future.complete(null);
            return future;
        }

        TaskScheduler.getDefaultInstance().runAsync(() -> {
            if(getLeftContent() == SlotContent.PURPLE){
                triggerAndComplete(future);
            } else if(getRightContent() == SlotContent.PURPLE){
                advanceIndexerClockwise();
                indexer.noLongerBusyNotifier.await();
                triggerAndComplete(future);
            } else if(getFrontContent() == SlotContent.PURPLE){
                advanceIndexerCounterclockwise();
                indexer.noLongerBusyNotifier.await();
                triggerAndComplete(future);
            }
        });

        return future;
    }

    private void triggerAndComplete(ChainedFuture<Object> future){
        feeder.trigger().thenRun(() -> {
            setLeftContent(SlotContent.OPEN);
            future.complete(null);
        });
    }

    public enum IndexerState {
        INACTIVE,
        COUNTERCLOCKWISE_BUMP,
        CLOCKWISE_BUMP,
        READYING_GREEN,
        READYING_PURPLE,
        LOADING_GREEN,
        LOADING_PURPLE
    }
}
