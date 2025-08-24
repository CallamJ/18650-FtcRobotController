package org.firstinspires.ftc.teamcode.hardware;

public class IncompleteImplementationException extends Exception {
    public IncompleteImplementationException(){
        super("Called to an unimplemented method in an incomplete implementation of an interface");
    }
}
