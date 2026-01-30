package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class Red3ArtifactCloseAuto extends AutoOpBase {
    @Override
    public PathChain buildPath(PathBuilder pathBuilder) {
        Pose scorePose = new Pose(0, 0, 0);
        Pose pickup1Pose = new Pose(12, 24, 90);

        return pathBuilder
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
    }
}
