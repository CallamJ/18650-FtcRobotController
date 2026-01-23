package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.SmartCamera;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class MultiAprilTagReader {
    private final List<AprilTagProcessor> processors;
    private final List<VisionPortal> portals;

    public MultiAprilTagReader(List<SmartCamera> cameras){
        processors = new ArrayList<>(cameras.size());
        portals = new ArrayList<>(cameras.size());
        int[] viewIds = VisionPortal.makeMultiPortalView(cameras.size(), VisionPortal.MultiPortalLayout.VERTICAL);

        for (int i = 0; i < cameras.size(); i++) {
            SmartCamera camera = cameras.get(i);
            Pose cameraPose = camera.getPose();
            CameraName camName = camera.passable();

            processors.add(new AprilTagProcessor.Builder()
                    .setCameraPose(cameraPose.getPosition(), cameraPose.getAngles())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build()
            );

            // STOPSHIP: 3/26/2025 viewIds is null here for some reason

            portals.add(new VisionPortal.Builder()
                    .setCamera(camName)
                    .setLiveViewContainerId(viewIds[i])
                    .addProcessor(processors.get(i))
                    .setCameraResolution(camera.getResolution())
                    .build()
            );
        }
    }

    public MultiAprilTagReader(SmartCamera... cameras){
        this(Arrays.asList(cameras));
    }

    public List<Detection> getAllUniqueDetections(){
        List<Detection> out = new ArrayList<>();
        for (int i = 0; i < processors.size(); i++) {
            out.addAll(getDetections(i));
        }

        return out.stream().distinct().collect(Collectors.toList());
    }

    /**
     * @return an optional pose based on the first detection found.
     */
    public Optional<Pose> getFirstPose(){
        try{
            Detection detection = getAllUniqueDetections().get(0);
            Pose localization = detection.getRobotPose();

            return Optional.of(localization);
        } catch(IndexOutOfBoundsException e) {
            return Optional.empty();
        }
    }

    /**
     * @return an optional pose based on the first detection found.
     */
    public Optional<Pose> getFirstPose(int cameraNum){
        try{
            Detection detection = getDetections(cameraNum).get(0);
            Pose localization = detection.getRobotPose();

            return Optional.of(localization);
        } catch(IndexOutOfBoundsException e) {
            return Optional.empty();
        }
    }

    public List<Detection> getDetections(int cameraNum){
        return processors.get(cameraNum).getDetections().stream().map(Detection::new).collect(Collectors.toList());
    }

    public List<VisionPortal> getPortals() {
        return portals;
    }

    public List<AprilTagProcessor> getProcessors() {
        return processors;
    }
}
