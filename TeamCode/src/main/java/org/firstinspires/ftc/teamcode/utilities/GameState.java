package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.components.TelescopingArm;
import org.firstinspires.ftc.teamcode.components.TiltBase;

@Config
public class GameState {
    //todo these will be different per-side (these are for blue)
    private static final Pose basketPose = new Pose( 53, 57, 0);
    private static final Pose submersiblePoseA = new Pose(0, 30, 0);
    //todo THIS CLASS DOES NOT WORK, UPDATE TO USE NEW GRIP/PITCH/ROLL CLASSES BEFORE USE

    //Blue Alliance Areas
    public static final Area basketArea = new Area(new Pose(15, 72), new Pose(72, 24));
    public static final Area specimenDeliveryArea = new Area(new Pose(-15, 48), new Pose(15, 24));
    public static final Area observationZoneCollectionArea = new Area(new Pose(-72, 72), new Pose(-24, 36));
    public static final Area submersibleChamberArea = new Area(new Pose(-15, 48), new Pose(15, 24));
    public static final Area submersibleAscentArea = new Area(new Pose(15, 24), new Pose(48, -24));


    private final DriveBase driveBase;
    private Runnable tickRunnable;

    private final TiltBase tiltBase;
    private final TelescopingArm telescopingArm;

    private Stage currentStage;

    public GameState(DriveBase driveBase, TiltBase tiltBase, TelescopingArm telescopingArm) {
        this.driveBase = driveBase;
        this.tiltBase = tiltBase;
        this.telescopingArm = telescopingArm;
    }

    public Stage findCurrentStage(){
        if(holdingSample()){
            if(!armRetracted() && armHorizontal() && nearSubmersible()){
                return Stage.FINALIZE_COLLECTION;
            }else{
                if(approachingBasket()){
                    if(armReadyToDeliver() && atBasket()){
                        return Stage.READY_TO_DELIVER;
                    }else {
                        return Stage.PREPARE_DELIVERY;
                    }
                }else {
                    return Stage.MOVE_TO_BASKET;
                }
            }
        }else {
            if (nearSubmersible()) {
                if ((!armReadyToCollect() /*&& collector.isWristTargetUp()*/)) {
                    return Stage.PREPARE_TO_COLLECT;
                }else {
                    return Stage.AWAIT_USER_INTERVENTION;
                }
            }else if(!armReadyToCollect()){
                return Stage.PREPARE_FOR_RETURN_TO_SUBMERSIBLE;
            }else {
                return Stage.MOVE_TO_SUBMERSIBLE;
            }

        }
    }

    public void beginMoveToBasket(){
        //driveBase.followTrajectoryAsync();
    }

    public void beginMoveToSubmersible(){
        //driveBase.followTrajectoryAsync();
    }

    public enum Stage {
        /**
         * Waiting for the driver to intervene, this is usually triggered when autopilot encounters unexpected or unhandled conditions.
         */
        AWAIT_USER_INTERVENTION,
        /**
         * Finalizing a collection and preparing the robot for movement to the basket.
         */
        FINALIZE_COLLECTION,
        /**
         * Moving the robot to the basket after collection.
         */
        MOVE_TO_BASKET,
        /**
         * Preparing the robot for delivering a sample to the basket.
         */
        PREPARE_DELIVERY,
        /**
         * The robot is ready to deliver a sample to the basket. Allow user to finalize delivery.
         */
        READY_TO_DELIVER,
        /**
         * Preparing the robot to return to the submersible after a sample has been delivered to the basket.
         */
        PREPARE_FOR_RETURN_TO_SUBMERSIBLE,
        /**
         *  Moving the robot to the submersible after delivery.
         */
        MOVE_TO_SUBMERSIBLE,
        /**
         * Prepare the robot to collect a sample from the submersible.
         */
        PREPARE_TO_COLLECT
    }

    private boolean armReadyToDeliver(){
        return armVertical() && armExtended();
    }

    private boolean armReadyToCollect(){
        return armRetracted() && armHorizontal();
    }

    private boolean armExtended(){
        return errorTolerable(telescopingArm.getCurrentPosition(), 38, 3);
    }

    private boolean armRetracted(){
        return errorTolerable(telescopingArm.getCurrentPosition(), 0, 3);
    }

    private boolean armVertical(){
        return errorTolerable(tiltBase.getCurrentPosition(), 90, 5);
    }

    private boolean armHorizontal(){
        return errorTolerable(tiltBase.getCurrentPosition(), 0, 5);
    }

    private boolean atBasket(){
        return withinInches(driveBase.getPoseSimple(), basketPose, 3);
    }

    private boolean approachingBasket(){
        return withinInches(driveBase.getPoseSimple(), basketPose, 24);
    }

    private boolean nearSubmersible(){
        return withinInches(driveBase.getPoseSimple(), submersiblePoseA, 2);
    }


    public boolean inBasketArea(){return basketArea.containsPose(driveBase.getPoseSimple());}

    public boolean inSpecimenDeliveryArea(){return specimenDeliveryArea.containsPose(driveBase.getPoseSimple());}

    public boolean inObservationZoneCollectionArea(){return observationZoneCollectionArea.containsPose(driveBase.getPoseSimple());}

    public boolean isInSubmersibleCollectionArea(){
        return submersibleChamberArea.containsPose(driveBase.getPoseSimple()) || submersibleAscentArea.containsPose(driveBase.getPoseSimple());
    }

    private boolean holdingSample(){
        return false; //collector.isHoldingSample();
    }

    private boolean errorTolerable(double number1, double number2, double tolerance){
        return Math.abs(number2 - number1) <= tolerance;
    }

    private boolean withinInches(Pose pose1, Pose pose2, double distance){
        return pose1.distanceTo(pose2) <= distance;
    }
}
