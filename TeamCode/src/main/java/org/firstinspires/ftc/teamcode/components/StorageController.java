package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.SmartColorSensor;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;

import java.util.Arrays;

public class StorageController {
    private final Feeder feeder;
    private final Indexer indexer;
    private final Collector collector;
    private final SmartColorSensor leftSensor,  rightSensor, frontSensor;

    private SlotContent leftSlot = SlotContent.OPEN, rightSlot = SlotContent.OPEN, frontSlot = SlotContent.OPEN;

    public StorageController(
            Feeder feeder,
            Indexer indexer,
            Collector collector,
            SmartColorSensor leftSensor,
            SmartColorSensor rightSensor,
            SmartColorSensor frontSensor
    ) {
        this.feeder = feeder;
        this.indexer = indexer;
        this.collector = collector;
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;
        this.frontSensor = frontSensor;
    }

    // STORAGE STATE ACCESSORS:
    public SlotContent getLeftSlot() {
        return leftSlot;
    }

    public SlotContent getRightSlot() {
        return rightSlot;
    }

    public SlotContent getFrontSlot() {
        return frontSlot;
    }

    public SlotContent[] getAllSlots() {
        return new SlotContent[]{frontSlot, rightSlot, leftSlot};
    }

    public boolean hasGreen(){
        return leftSlot == SlotContent.GREEN || rightSlot == SlotContent.GREEN || frontSlot == SlotContent.GREEN;
    }

    public boolean hasPurple(){
        return leftSlot == SlotContent.PURPLE || rightSlot == SlotContent.PURPLE || frontSlot == SlotContent.PURPLE;
    }

    public boolean isFull(){
        return !(leftSlot == SlotContent.OPEN ||  rightSlot == SlotContent.OPEN || frontSlot == SlotContent.OPEN);
    }

    public int countGreen(){
        return (int) Arrays.stream(getAllSlots()).filter(slotContent -> slotContent == SlotContent.GREEN).count();
    }

    public int countPurple(){
        return (int) Arrays.stream(getAllSlots()).filter(slotContent -> slotContent == SlotContent.PURPLE).count();
    }

    public int countOpen(){
        return (int) Arrays.stream(getAllSlots()).filter(slotContent -> slotContent == SlotContent.OPEN).count();
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

    // STATE HELPERS:

    private void cycleStateClockwise(){
        SlotContent[] allSlots = getAllSlots();
        frontSlot = allSlots[2];
        leftSlot = allSlots[1];
        rightSlot = allSlots[0];
    }

    private void cycleStateCounterclockwise(){
        SlotContent[] allSlots = getAllSlots();
        frontSlot = allSlots[1];
        leftSlot = allSlots[0];
        rightSlot = allSlots[2];
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
