package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.SmartServo;

@Config
public class Feeder {
    private SmartServo servo;

    public Feeder(SmartServo servo) {
        this.servo = servo;
    }

    public void triggerFeed() {

    }
}
