package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utilities.Pose;

/*
 * This is the core of every autonomous OpMode
 */
@Config
public abstract class AutonomousCore extends OpModeCore {

    protected final Pose startPose = createStartPose();

    /**
     * This method is used to initialize startPose. Override it to set the starting pose
     *
     * @return the starting pose this OpMode should begin in.
     */
    protected abstract Pose createStartPose();

    //todo re-add some methods and other things here that are only used during autonomous
    //todo the old code was a mess so you are going to need to refactor/rewrite as you move old methods into here
}
