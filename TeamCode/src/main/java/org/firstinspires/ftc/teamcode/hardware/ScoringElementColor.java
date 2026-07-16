package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import org.jetbrains.annotations.NotNull;

/**
 * Enumeration of possible scoring element colors that can be detected.
 */
public enum ScoringElementColor {
    NONE,
    RED,
    BLUE,
    GREEN,
    YELLOW,
    PURPLE
    ;

    @NonNull
    @NotNull
    @Override
    public String toString() {
        return name();
    }
}