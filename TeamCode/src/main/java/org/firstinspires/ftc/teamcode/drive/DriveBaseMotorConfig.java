package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utilities.Direction;

public class DriveBaseMotorConfig {
    public final String leftFrontName;
    public final String rightFrontName;
    public final String leftRearName;
    public final String rightRearName;
    public final Direction leftFrontDirection;
    public final Direction rightFrontDirection;
    public final Direction leftRearDirection;
    public final Direction rightRearDirection;
    private DriveBaseMotorConfig(DriveBaseMotorConfigBuilder builder) {
        this.leftFrontName = builder.leftFrontName;
        this.rightFrontName = builder.rightFrontName;
        this.leftRearName = builder.leftRearName;
        this.rightRearName = builder.rightRearName;
        this.leftFrontDirection = builder.leftFrontDirection;
        this.rightFrontDirection = builder.rightFrontDirection;
        this.leftRearDirection = builder.leftRearDirection;
        this.rightRearDirection = builder.rightRearDirection;
    }

    public static class DriveBaseMotorConfigBuilder {

        private String leftFrontName;
        private String rightFrontName;
        private String leftRearName;
        private String rightRearName;
        private Direction leftFrontDirection;
        private Direction rightFrontDirection;
        private Direction leftRearDirection;
        private Direction rightRearDirection;

        public DriveBaseMotorConfigBuilder(){

        }

        public DriveBaseMotorConfigBuilder leftFront(String motorName, Direction direction){
            this.leftFrontName = motorName;
            this.leftFrontDirection = direction;
            return this;
        }

        public DriveBaseMotorConfigBuilder rightFront(String motorName, Direction direction){
            this.rightFrontName = motorName;
            this.rightFrontDirection = direction;
            return this;
        }

        public DriveBaseMotorConfigBuilder leftRear(String motorName, Direction direction){
            this.leftRearName = motorName;
            this.leftRearDirection = direction;
            return this;
        }

        public DriveBaseMotorConfigBuilder rightRear(String motorName, Direction direction){
            this.rightRearName = motorName;
            this.rightRearDirection = direction;
            return this;
        }

        public DriveBaseMotorConfig build(){
            return new DriveBaseMotorConfig(this);
        }
    }

    public DcMotorEx configAndFetchLeftFront(HardwareMap hardwareMap){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, leftFrontName);
        motor.setDirection(this.leftFrontDirection.toMotorDirection());
        return motor;
    }

    public DcMotorEx configAndFetchRightFront(HardwareMap hardwareMap){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, rightFrontName);
        motor.setDirection(this.rightFrontDirection.toMotorDirection());
        return motor;
    }

    public DcMotorEx configAndFetchLeftRear(HardwareMap hardwareMap){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, leftRearName);
        motor.setDirection(this.leftRearDirection.toMotorDirection());
        return motor;
    }

    public DcMotorEx configAndFetchRightRear(HardwareMap hardwareMap){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, rightRearName);
        motor.setDirection(this.rightRearDirection.toMotorDirection());
        return motor;
    }
}
