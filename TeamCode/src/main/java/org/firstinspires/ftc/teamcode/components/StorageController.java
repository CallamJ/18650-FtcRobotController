package org.firstinspires.ftc.teamcode.components;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.SmartColorSensor;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;

import java.util.Arrays;

@Configurable
public class StorageController {
    public static float gain = 150f;
    private final Feeder feeder;
    private final Indexer indexer;
    private final Collector collector;
    private final SmartColorSensor frontSensor;

    /**
     * Slot 0: The slot that faces forward when the opmode is started.<br>
     * Slot 1: The slot that faces back-right when the opmode is started.<br>
     * slot 2: The slot that faces back-left when the opmode is started.<br>
     */
    private final SlotContent[] indexerContent = new SlotContent[]{SlotContent.OPEN, SlotContent.OPEN, SlotContent.OPEN};

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

        OpModeCore.getTelemetry().addLine("Storage Controller")
                .addData("Sensor Color", frontSensor::getScoringElementColor)
                .addData("Sensor Hue", () -> frontSensor.getHSV()[0])
                .addData("Sensor Saturation", () -> frontSensor.getHSV()[1])
                .addData("Sensor Value", () -> frontSensor.getHSV()[2])
                .addData("Front Content", () -> indexerContent[0])
                .addData("Right Content", () -> indexerContent[1])
                .addData("Left Content", () -> indexerContent[2]);
    }

    public void tick(){
        feeder.tick();
        indexer.tick();
        frontSensor.setGain(gain);
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
                    indexerContent[indexer.getNormalizedCurrentIndex()] = SlotContent.GREEN;
                    break;
                }
                case PURPLE: {
                    indexerContent[indexer.getNormalizedCurrentIndex()] = SlotContent.PURPLE;
                    break;
                }
            }
        }
    }

    public void checkAutomaticAdvance(){
        if(getLeftContent() == SlotContent.OPEN && getFrontContent() != SlotContent.OPEN){
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
        return indexerContent[(indexer.getNormalizedCurrentIndex() + 1) % 3];
    }

    public SlotContent getLeftContent(){
        return indexerContent[(indexer.getNormalizedCurrentIndex() + 2) % 3];
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
        return new ChainedFuture<>();
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
