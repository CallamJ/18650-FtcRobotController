package org.firstinspires.ftc.teamcode.components.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.components.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.hardware.ScoringColorSensor;
import org.firstinspires.ftc.teamcode.hardware.ScoringElementColor;
import org.firstinspires.ftc.teamcode.hardware.SmartLEDIndicator;

import java.util.Arrays;

@Configurable
public class IndexerStorage {
    public static int requiredConsecutiveContentReads = 3;

    public enum SlotContent {
        OPEN,
        GREEN,
        PURPLE
    }

    private final Indexer indexer;
    private final ScoringColorSensor frontSensor;
    private final SmartLEDIndicator leftLED;
    private final SmartLEDIndicator rightLED;
    private final SmartLEDIndicator frontLED;

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

    public IndexerStorage(
            Indexer indexer,
            ScoringColorSensor frontSensor,
            SmartLEDIndicator leftLED,
            SmartLEDIndicator rightLED,
            SmartLEDIndicator frontLED
    ) {
        this.indexer = indexer;
        this.frontSensor = frontSensor;
        this.leftLED = leftLED;
        this.rightLED = rightLED;
        this.frontLED = frontLED;
        this.indexerContent = new SlotContent[]{SlotContent.OPEN, SlotContent.OPEN, SlotContent.OPEN};
        this.isFrontFresh = false;
    }

    private ScoringElementColor getFrontDetection() {
        if (frontSensor.getClosestColorMatchName().equals("ARTIFACT_PURPLE")) {
            return ScoringElementColor.PURPLE;
        } else if (frontSensor.getClosestColorMatchName().equals("ARTIFACT_GREEN")) {
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

    public boolean isFrontFresh() {
        return isFrontFresh;
    }

    public void dropFreshFlag() {
        isFrontFresh = false;
    }

    public void tick() {
        applyContentColor(leftLED, getLeftContent());
        applyContentColor(rightLED, getRightContent());
        applyContentColor(frontLED, getFrontContent());
    }

    public void updateIndexerContent() {
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
            case GREEN:
                if (getFrontContent() == SlotContent.OPEN) {
                    isFrontFresh = true;
                }
                setFrontContent(SlotContent.GREEN);
                break;
            case PURPLE:
                if (getFrontContent() == SlotContent.OPEN) {
                    isFrontFresh = true;
                }
                setFrontContent(SlotContent.PURPLE);
                break;
            case NONE:
                setFrontContent(SlotContent.OPEN);
                break;
        }
    }

    public boolean readyContentToFront(SlotContent target) {
        if (getFrontContent() == target) {
            return false;
        } else if (getLeftContent() == target) {
            indexer.advanceIndexClockwise();
            return true;
        } else if (getRightContent() == target) {
            indexer.advanceIndexCounterclockwise();
            return true;
        }
        return false;
    }

    public boolean readyContentToLeft(SlotContent target) {
        if (getLeftContent() == target) {
            return false;
        } else if (getRightContent() == target) {
            indexer.advanceIndexClockwise();
            return true;
        } else if (getFrontContent() == target) {
            indexer.advanceIndexCounterclockwise();
            return true;
        }
        return false;
    }

    public boolean readyContentToRight(SlotContent target) {
        if (getRightContent() == target) {
            return false;
        } else if (getFrontContent() == target) {
            indexer.advanceIndexClockwise();
            return true;
        } else if (getLeftContent() == target) {
            indexer.advanceIndexCounterclockwise();
            return true;
        }
        return false;
    }

    public SlotContent getFrontContent() {
        return indexerContent[indexer.getNormalizedCurrentIndex()];
    }

    public SlotContent getRightContent() {
        return indexerContent[(indexer.getNormalizedCurrentIndex() + 2) % 3];
    }

    public SlotContent getLeftContent() {
        return indexerContent[(indexer.getNormalizedCurrentIndex() + 1) % 3];
    }

    public boolean hasGreen() {
        return indexerContent[2] == SlotContent.GREEN
                || indexerContent[1] == SlotContent.GREEN
                || indexerContent[0] == SlotContent.GREEN;
    }

    public boolean hasPurple() {
        return indexerContent[2] == SlotContent.PURPLE
                || indexerContent[1] == SlotContent.PURPLE
                || indexerContent[0] == SlotContent.PURPLE;
    }

    public boolean isFull() {
        return !(indexerContent[2] == SlotContent.OPEN
                || indexerContent[1] == SlotContent.OPEN
                || indexerContent[0] == SlotContent.OPEN);
    }

    public boolean isEmpty() {
        return indexerContent[2] == SlotContent.OPEN
                && indexerContent[1] == SlotContent.OPEN
                && indexerContent[0] == SlotContent.OPEN;
    }

    public int countGreen() {
        return (int) Arrays.stream(indexerContent)
                .filter(slotContent -> slotContent == SlotContent.GREEN)
                .count();
    }

    public int countPurple() {
        return (int) Arrays.stream(indexerContent)
                .filter(slotContent -> slotContent == SlotContent.PURPLE)
                .count();
    }

    public int countOpen() {
        return (int) Arrays.stream(indexerContent)
                .filter(slotContent -> slotContent == SlotContent.OPEN)
                .count();
    }

    public void setFrontContent(SlotContent content) {
        indexerContent[indexer.getNormalizedCurrentIndex()] = content;
    }

    public void setRightContent(SlotContent content) {
        indexerContent[(indexer.getNormalizedCurrentIndex() + 2) % 3] = content;
    }

    public void setLeftContent(SlotContent content) {
        indexerContent[(indexer.getNormalizedCurrentIndex() + 1) % 3] = content;
    }

    private void applyContentColor(SmartLEDIndicator led, SlotContent content) {
        switch (content) {
            case OPEN:
                led.setColor(SmartLEDIndicator.IndicatorColor.WHITE);
                break;
            case GREEN:
                led.setColor(SmartLEDIndicator.IndicatorColor.GREEN);
                break;
            case PURPLE:
                led.setColor(SmartLEDIndicator.IndicatorColor.VIOLET);
                break;
        }
    }
}
