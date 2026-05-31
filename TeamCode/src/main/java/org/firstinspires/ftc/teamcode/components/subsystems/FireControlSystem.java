package org.firstinspires.ftc.teamcode.components.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.components.mechanisms.DriveBase;
import org.firstinspires.ftc.teamcode.components.mechanisms.Hood;
import org.firstinspires.ftc.teamcode.components.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.components.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.core.implementations.AutonomousConfiguration;
import org.firstinspires.ftc.teamcode.hardware.SmartLEDIndicator;
import org.firstinspires.ftc.teamcode.hardware.SmartLimelight3A;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;
import org.firstinspires.ftc.teamcode.utilities.Pose;

@Configurable
public class FireControlSystem {
    public static double baseVelocity = 2700, kVDist = 620, minVelocity = 3500;
    public static double hoodBasePos = 0.53, kHoodDist = 0.1;
    public static boolean useDepotPoseFallbackWhenTagNotVisible = true;
    public static double alignedButLauncherOffHueDeg = 145.0;
    public static double BLUE_DEPOT_X = 59, BLUE_DEPOT_Y = 57;
    public static double RED_DEPOT_X = 59, RED_DEPOT_Y = -57;
    public static double TURRET_TOLERANCE = 3;
    private final Turret turret;
    private final Hood hood;
    private final Launcher launcher;
    private final SmartLimelight3A limelight;
    private final SmartLEDIndicator led;
    private SmartLEDIndicator.IndicatorColor ledOverrideColor;
    private DriveBase driveBase;
    private Pose depotPose;
    private MatchStateStore.AllianceColor allianceColor = MatchStateStore.AllianceColor.BLUE;
    private boolean depotAutoAimEnabled = true;
    private boolean turretAutoAimEnabled = true;
    private State state;
    private boolean runLauncher = false;
    public static double maxVelocity = 5000;
    public static double bearingToDepot = 0;
    private boolean firing = false;


    public FireControlSystem(Turret turret, Hood hood, Launcher launcher, SmartLimelight3A limelight, SmartLEDIndicator led) {
        this(turret, hood, launcher, limelight, led, null, null);
    }

    public FireControlSystem(
            Turret turret,
            Hood hood,
            Launcher launcher,
            SmartLimelight3A limelight,
            SmartLEDIndicator led,
            DriveBase driveBase,
            Pose depotPose
    ) {
        this.turret = turret;
        this.hood = hood;
        this.launcher = launcher;
        this.limelight = limelight;
        this.led = led;
        this.driveBase = driveBase;
        this.depotPose = depotPose;
    }

    public void tick(){
        if (turret == null || launcher == null || limelight == null) {
            setState(State.SEEKING);
            return;
        }
        turret.tick();
        launcher.tick();

        boolean turretAligned = isTurretAligned();
        boolean launcherSpun = isLauncherSpun();
        boolean seesAnyDepotTag = limelight.getAprilTags().stream().anyMatch(tag -> !tag.isObelisk());
        if (turretAligned && launcherSpun) {
            setState(State.READY);
        } else {
            setState(State.READYING);
        }


        SmartLimelight3A.AprilTag depot = depotAutoAimEnabled ? getAllianceDepotTag() : null;
        if (depot == null) {
            setState(State.SEEKING);
            if (depotAutoAimEnabled) {
                aimTowardDepotPoseIfConfigured();
            }
        } else {
            try {
                bearingToDepot = -depot.bearingDegToTag();
                if (turretAutoAimEnabled) {
                    setTurretTargetClosestFacing(turret.getCurrentPosition() + bearingToDepot);
                }
                hood.setTargetPosition(hoodBasePos + kHoodDist * depot.distanceXYToTagMeters());
                if (runLauncher) {
                    launcher.setTargetVelocity(Math.min(Math.max(baseVelocity + depot.distanceXYToTagMeters() * kVDist, minVelocity), maxVelocity));
                }
            } catch (IllegalStateException e) {
                OpModeCore.getTelemetry().warning("Getting April Tag Bearing failed: " + e.getMessage());
            }
        }
        if(!runLauncher){
            launcher.stop();
        }

        if (led != null) {
            if (ledOverrideColor != null) {
                led.setColor(ledOverrideColor);
            } else if (firing) {
                led.setPulseWidthMicros(1350);
            } else if (!runLauncher && turretAligned) {
                led.setHue(alignedButLauncherOffHueDeg);
            } else if(state != null) {
                led.setColor(state.color);
            } else {
                led.setColor(SmartLEDIndicator.IndicatorColor.ORANGE);
            }
        }
    }

    private void setState(State state) {
        this.state = state;
    }

    public boolean isLauncherSpun(){
        return launcher.getVelocity() >= launcher.getTargetVelocity() - 50 && isLauncherRunning();
    }

    public boolean isTurretAligned(){
        return Math.abs(turret.getCurrentPosition() - turret.getDesiredTarget()) <= TURRET_TOLERANCE;
    }

    public boolean isLauncherRunning(){
        return runLauncher;
    }

    public void startLauncher(){
        runLauncher = true;
    }

    public void stopLauncher(){
        runLauncher = false;
    }

    public void toggleLauncher(){
        runLauncher = !runLauncher;
    }

    public void setDepotPose(Pose depotPose) {
        this.depotPose = depotPose;
    }

    public void setDriveBase(DriveBase driveBase) {
        this.driveBase = driveBase;
    }

    public void setFiring(boolean firing) {
        launcher.setRunMaxPower(firing);
        this.firing = firing;
    }

    public void setAllianceColor(MatchStateStore.AllianceColor allianceColor) {
        if (allianceColor != null) {
            this.allianceColor = allianceColor;
        }
    }

    public void setDepotAutoAimEnabled(boolean depotAutoAimEnabled) {
        this.depotAutoAimEnabled = depotAutoAimEnabled;
    }

    public void setTurretAutoAimEnabled(boolean turretAutoAimEnabled) {
        this.turretAutoAimEnabled = turretAutoAimEnabled;
    }

    public void setLedOverrideColor(SmartLEDIndicator.IndicatorColor ledOverrideColor) {
        this.ledOverrideColor = ledOverrideColor;
    }

    private void aimTowardDepotPoseIfConfigured() {
        Pose targetDepotPose = getTargetDepotPose();
        if (!useDepotPoseFallbackWhenTagNotVisible || driveBase == null || targetDepotPose == null) {
            return;
        }

        Pose currentPose = driveBase.getPoseSimple();
        double deltaX = targetDepotPose.x() - currentPose.x();
        double deltaY = targetDepotPose.y() - currentPose.y();
        if (Math.hypot(deltaX, deltaY) < 1e-6) {
            return;
        }

        double headingToDepotDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double relativeBearingDeg = currentPose.heading() - headingToDepotDeg;
        if (turretAutoAimEnabled) {
            setTurretTargetClosestFacing(relativeBearingDeg);
        }
        hood.setTargetPosition(hoodBasePos + kHoodDist * (Math.hypot(deltaX, deltaY)) / 39.37);
        launcher.setTargetVelocity(Math.max(minVelocity, baseVelocity + kVDist * Math.hypot(deltaX, deltaY) / 39.37));
    }

    private Pose getTargetDepotPose() {
        if (depotPose != null) {
            return depotPose;
        }
        if (allianceColor == MatchStateStore.AllianceColor.RED) {
            return new Pose(RED_DEPOT_X, RED_DEPOT_Y);
        }
        return new Pose(BLUE_DEPOT_X, BLUE_DEPOT_Y);
    }

    private SmartLimelight3A.AprilTag getAllianceDepotTag() {
        SmartLimelight3A.AprilTag.Type expectedDepotType = allianceColor == MatchStateStore.AllianceColor.RED
                ? SmartLimelight3A.AprilTag.Type.RED_DEPOT
                : SmartLimelight3A.AprilTag.Type.BLUE_DEPOT;
        return limelight.getAprilTags()
                .stream()
                .filter(tag -> tag.type() == expectedDepotType)
                .findFirst()
                .orElse(null);
    }

    private void setTurretTargetClosestFacing(double nominalFacingAngleDeg) {
        double currentAngleDeg = turret.getCurrentPosition();
        double nearestFacing = selectClosestEquivalentAngle(
                nominalFacingAngleDeg,
                currentAngleDeg,
                Turret.minAngle,
                Turret.maxAngle
        );
        turret.setTargetPosition(nearestFacing);
    }

    private static double selectClosestEquivalentAngle(
            double nominalAngleDeg,
            double currentAngleDeg,
            double minAngleDeg,
            double maxAngleDeg
    ) {
        if (!Double.isFinite(nominalAngleDeg) || !Double.isFinite(currentAngleDeg)) {
            return nominalAngleDeg;
        }

        if (minAngleDeg > maxAngleDeg) {
            double temp = minAngleDeg;
            minAngleDeg = maxAngleDeg;
            maxAngleDeg = temp;
        }

        double kIdeal = (currentAngleDeg - nominalAngleDeg) / 360.0;
        long kRounded = Math.round(kIdeal);

        if (Double.isInfinite(minAngleDeg) && Double.isInfinite(maxAngleDeg)) {
            return nominalAngleDeg + (360.0 * kRounded);
        }

        if (Double.isInfinite(maxAngleDeg)) {
            long kMin = (long) Math.ceil((minAngleDeg - nominalAngleDeg) / 360.0);
            long k = Math.max(kMin, kRounded);
            return nominalAngleDeg + (360.0 * k);
        }

        if (Double.isInfinite(minAngleDeg)) {
            long kMax = (long) Math.floor((maxAngleDeg - nominalAngleDeg) / 360.0);
            long k = Math.min(kMax, kRounded);
            return nominalAngleDeg + (360.0 * k);
        }

        long kMin = (long) Math.ceil((minAngleDeg - nominalAngleDeg) / 360.0);
        long kMax = (long) Math.floor((maxAngleDeg - nominalAngleDeg) / 360.0);
        if (kMin <= kMax) {
            long k = Math.max(kMin, Math.min(kRounded, kMax));
            return nominalAngleDeg + (360.0 * k);
        }

        double unconstrainedNearest = nominalAngleDeg + (360.0 * kRounded);
        return clamp(unconstrainedNearest, minAngleDeg, maxAngleDeg);
    }

    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    public enum State {
        SEEKING(SmartLEDIndicator.IndicatorColor.RED),
        READYING(SmartLEDIndicator.IndicatorColor.ORANGE),
        READY(SmartLEDIndicator.IndicatorColor.BLUE, true)
        ;
        private final boolean isReady;
        private final SmartLEDIndicator.IndicatorColor color;
        public boolean isReady() {
            return isReady;
        }
        State(SmartLEDIndicator.IndicatorColor color){
            this.color = color;
            this.isReady = false;
        }

        State(SmartLEDIndicator.IndicatorColor color, boolean isReady){
            this.color = color;
            this.isReady = isReady;
        }
    }
}
