package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.AutonomousCore;
import org.firstinspires.ftc.teamcode.utilities.Pose;

@Autonomous()
public class AutonomousOpExample extends AutonomousCore {

    /**
     * This method is used to initialize startPose. Override it to set the starting pose
     *
     * @return the starting pose this OpMode should begin in.
     */
    @Override
    protected Pose createStartPose() {
        return new Pose(0, 0, 0);
    }
}
