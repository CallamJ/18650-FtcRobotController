package org.firstinspires.ftc.teamcode.vision;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

/**
 * Provides a simpler and more concise interaction API for AprilTagDetections.
 */
public class Detection {
    AprilTagDetection tagDetection;

    public Detection(AprilTagDetection aprilTagDetection) {
        this.tagDetection = aprilTagDetection;
    }

    /**
     * @return the absolute 3D pose of the robot based on the april tag detection.
     */
    public Pose getRobotPose(){
        return Pose.from(tagDetection.robotPose);
    }

    public String getName(){
        return tagDetection.metadata.name;
    }

    public int getId(){
        return tagDetection.metadata.id;
    }

    @Override
    public boolean equals(@Nullable Object obj) {
        if(this == obj)
            return true;

        if(obj instanceof Detection){
            return ((Detection) obj).getId() == getId();
        }

        return false;
    }

    @NonNull
    @Override
    public String toString() {
        Pose pose = getRobotPose();
        return String.format(Locale.ROOT, "(x:%.2f, y:%.2f) yaw:%.2f", pose.x(), pose.y(), pose.yaw());
    }
}
