package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

public enum ScoringElementColor {
    RED, YELLOW, BLUE, NONE;

    @NonNull
    @Override
    public String toString() {
        return name();
    }
}
