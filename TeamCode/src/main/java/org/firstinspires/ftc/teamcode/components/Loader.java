package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;

@Config
public class Loader {
    private SmartServo servo;

    public Loader(SmartServo servo) {
        this.servo = servo;
    }

    public void triggerFeed() {

    }
}
