package org.firstinspires.ftc.teamcode.hardware;

public class HardwareMapNotInitializedException extends RuntimeException {
    public HardwareMapNotInitializedException(){
        super("Hardware map was not initialized before attempting to retrieve a device!");
    }
}
