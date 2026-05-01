package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.components.mechanisms.DriveBase;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.drive.DriveBaseMotorConfig;
import org.firstinspires.ftc.teamcode.utilities.Direction;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

@Autonomous(name = "1 - Move Off Line (FORWARDPOWER)")
public class MoveOffLineAuto extends OpModeCore {
    DriveBase driveBase;
    @Override
    protected void onInitialize() {
        DriveBaseMotorConfig.DriveBaseMotorConfigBuilder configBuilder = new DriveBaseMotorConfig.DriveBaseMotorConfigBuilder();
        configBuilder.leftFront("LFront", Direction.FORWARD);
        configBuilder.leftRear("LRear", Direction.FORWARD);
        configBuilder.rightFront("RFront", Direction.REVERSE);
        configBuilder.rightRear("RRear", Direction.FORWARD);
        this.driveBase = new DriveBase(hardwareMap, configBuilder.build(), false);
    }

    @Override
    protected void onRun() {
        driveBase.moveUsingPower(0, -1, 0);
    }
}
