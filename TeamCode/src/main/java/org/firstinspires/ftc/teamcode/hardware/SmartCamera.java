package org.firstinspires.ftc.teamcode.hardware;

import android.content.Context;
import android.graphics.ImageFormat;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.utilities.Pose;

public class SmartCamera extends Device implements CameraName {
    private final CameraName cameraName;
    private final String name;
    private final Pose pose;

    //todo make this wrap WebcamName

    /**
     * @param cameraName base camera object to operate on.
     * @param name configured name of the camera to create.
     * @param pose pose of the camera relative to the robot. The pitch is adjusted -90 degrees for the processor.
     */
    SmartCamera(CameraName cameraName, String name, Pose pose){
        super(name);
        this.cameraName = cameraName;
        this.name = cameraName.toString();
        this.pose = pose.plusPitch(-90);
    }

    public CameraName passable(){
        return cameraName;
    }

    public Position getPosition(){
        return pose.getPosition();
    }

    public YawPitchRollAngles getAngles(){
        return pose.getAngles();
    }

    public String getName(){
        return name;
    }

    public Pose getPose(){
        return pose;
    }

    public Size getResolution(){
        org.firstinspires.ftc.robotcore.external.android.util.Size ftcSize = cameraName.getCameraCharacteristics().getDefaultSize(ImageFormat.YUY2);
        return new Size(ftcSize.getWidth(), ftcSize.getHeight());
    }

    /**
     * Returns whether or not this name is that of a webcam. If true, then the
     * {@link CameraName} can be cast to a {@link WebcamName}.
     *
     * @return whether or not this name is that of a webcam
     * @see WebcamName
     */
    @Override
    public boolean isWebcam() {
        return cameraName.isWebcam();
    }

    /**
     * Returns whether or not this name is that of a builtin phone camera. If true, then the
     * {@link CameraName} can be cast to a {@link BuiltinCameraName}.
     *
     * @return whether or not this name is that of a builtin phone camera
     * @see BuiltinCameraName
     */
    @Override
    public boolean isCameraDirection() {
        return cameraName.isCameraDirection();
    }

    /**
     * Returns whether this name is one representing the ability to switch amongst a
     * series of member cameras. If true, then the receiver can be cast to a
     * {@link SwitchableCameraName}.
     *
     * @return whether this is a {@link SwitchableCameraName}
     */
    @Override
    public boolean isSwitchable() {
        return cameraName.isSwitchable();
    }

    /**
     * Returns whether or not this name represents that of an unknown or indeterminate camera.
     *
     * @return whether or not this name represents that of an unknown or indeterminate camera
     */
    @Override
    public boolean isUnknown() {
        return cameraName.isUnknown();
    }

    /**
     * Requests from the user permission to use the camera if same has not already been granted.
     * This may take a long time, as interaction with the user may be necessary. When the outcome
     * is known, the reportResult continuation is called with the result. The report may occur either
     * before or after the call to {@link #asyncRequestCameraPermission} has itself returned. The report will
     * be delivered using the indicated {@link Continuation}
     *
     * @param context      the context in which the permission request should run
     * @param deadline     the time by which the request must be honored or given up as ungranted.
     *                     If this {@link Deadline} is cancelled while the request is outstanding,
     *                     then the permission request will be aborted and false reported as
     *                     the result of the request.
     * @param continuation the dispatcher used to deliver results of the permission request
     * @throws IllegalArgumentException if the cameraName does not match any known camera device.
     * @see #requestCameraPermission
     * @noinspection JavadocDeclaration
     */
    @Override
    public void asyncRequestCameraPermission(Context context, Deadline deadline, Continuation<? extends Consumer<Boolean>> continuation) {
        cameraName.asyncRequestCameraPermission(context, deadline, continuation);
    }

    /**
     * Requests from the user permission to use the camera if same has not already been granted.
     * This may take a long time, as interaction with the user may be necessary. The call is made
     * synchronously: the calling thread blocks until an answer is obtained.
     *
     * @param deadline the time by which the request must be honored or given up as ungranted
     * @return whether or not permission to use the camera has been granted.
     * @see #asyncRequestCameraPermission
     */
    @Override
    public boolean requestCameraPermission(Deadline deadline) {
        return cameraName.requestCameraPermission(deadline);
    }

    /**
     * <p>Query the capabilities of a camera device. These capabilities are
     * immutable for a given camera.</p>
     *
     * @return The properties of the given camera. A degenerate empty set of properties is returned on error.
     */
    @Override
    public CameraCharacteristics getCameraCharacteristics() {
        return cameraName.getCameraCharacteristics();
    }
}
