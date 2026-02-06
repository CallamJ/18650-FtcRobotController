package org.firstinspires.ftc.teamcode.core.implementations;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.components.StorageController;
import org.firstinspires.ftc.teamcode.utilities.Await;

@Autonomous(name="1 - Blue 3 Artifact Close")
public class Blue3ArtifactCloseAuto extends AutoOpBase {
    Pose start = new Pose();
    Pose fire = new Pose(36, -6, Math.PI);
    Pose move = new Pose(15, -24, 0.65);

    @Override
    protected void initialize() {
        super.initialize();
        storageController.setLeftContent(StorageController.SlotContent.PURPLE);
        storageController.setRightContent(StorageController.SlotContent.GREEN);
        storageController.setFrontContent(StorageController.SlotContent.PURPLE);
    }

    @Override
    protected void run() {
        super.run();
        collector.setPower(1);
        launcher.setTargetVelocity(2000);

        // go to firing pos

        Follower follower = driveBase.getFollower();
        PathBuilder builder = new PathBuilder(follower)
                .addPath(new BezierLine(start, fire))
                .setLinearHeadingInterpolation(start.getHeading(), fire.getHeading());
        follower.followPath(builder.build());

        while(follower.isBusy()){
            tick();
        }

        switch(fiducialId){
            case 21: {
                //gpp
                storageController.loadGreen();
                storageController.loadPurple();
                storageController.loadPurple();
                break;
            }
            case 22: {
                //pgp
                storageController.loadPurple();
                storageController.loadGreen();
                storageController.loadPurple();
                break;
            }
            case 23: {
                //ppg
                storageController.loadPurple();
                storageController.loadPurple();
                storageController.loadGreen();
                break;
            }
            default: {
                //guess
                storageController.loadGreen();
                storageController.loadPurple();
                storageController.loadPurple();
            }
        }

        while (!storageController.allTasksComplete()){
            tick();
        }

        //go to moved spot

        launcher.setTargetVelocity(0);
        collector.setPower(0);
        builder = new PathBuilder(driveBase.getFollower())
                .addPath(new BezierLine(fire, move))
                .setLinearHeadingInterpolation(fire.getHeading(), move.getHeading());

        follower.followPath(builder.build());

        while(follower.isBusy()){
            tick();
        }

    }

    @Override
    public PathChain buildPath(PathBuilder pathBuilder, Follower follower) {
        return pathBuilder
                .addPath(new BezierLine(start, fire))
                .setLinearHeadingInterpolation(start.getHeading(), fire.getHeading())
                .addPoseCallback(fire, () -> {
                    switch(fiducialId){
                        case 21: {
                            //gpp
                            storageController.loadGreen();
                            storageController.loadPurple();
                            storageController.loadPurple();
                            break;
                        }
                        case 22: {
                            //pgp
                            storageController.loadPurple();
                            storageController.loadGreen();
                            storageController.loadPurple();
                            break;
                        }
                        case 23: {
                            //ppg
                            storageController.loadPurple();
                            storageController.loadPurple();
                            storageController.loadGreen();
                            break;
                        }
                        default: {
                            //guess
                            storageController.loadGreen();
                            storageController.loadPurple();
                            storageController.loadPurple();
                        }
                    }

                }, 0.5)
                .build();
    }

    boolean secondPathFlag = false;
    @Override
    public void tick() {
        super.tick();
        if(!driveBase.getFollower().isBusy() && !secondPathFlag && getRuntime() > 12){
            secondPathFlag = true;
            launcher.setTargetVelocity(0);
            collector.setPower(0);
            PathBuilder builder = new PathBuilder(driveBase.getFollower())
                    .addPath(new BezierLine(fire, move))
                    .setLinearHeadingInterpolation(fire.getHeading(), move.getHeading());
            driveBase.getFollower().followPath(builder.build());
        }
    }
}
