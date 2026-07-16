package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;

/**
 * The only intended Control Hub Configurables surface.
 *
 * Keep this class scalar-only. Configurables recursively serializes public
 * static fields from annotated classes during Sloth load, so object, enum,
 * array, collection, and runtime hardware fields belong elsewhere.
 */
@Configurable
public final class LiveMatchTuning {
    private LiveMatchTuning() {}

    public static boolean runTeleOpFcs = true;
    public static double matchStateFreshnessMs = 10000;
    public static double teleOpMatchStateSaveIntervalMs = 500;
    public static double teleOpFollowerMaxPower = 1.0;
    public static String defaultAllianceColor = "BLUE";
    public static double indexerZeroBumpTicksPerTriggerUnit = 20.0;
    public static double turretZeroBumpTicksPerTriggerUnit = -200.0;
    public static double maintenancePoseTrimInchesPerTouchpadUnit = 12.0;
    public static double manualAimDegreesPerSecond = 75.0;
    public static double manualAimStickDeadband = 0.08;

    public static double farFiringTaskBaseBlueXIn = -55.5;
    public static double farFiringTaskBaseBlueYIn = -48;
    public static double farFiringTaskBaseBlueHeadingDeg = -105;
    public static double farFiringTaskBaseRedXIn = -55.5;
    public static double farFiringTaskBaseRedYIn = 48;
    public static double farFiringTaskBaseRedHeadingDeg = 105;
    public static double farFiringTaskDriveToBaseTimeoutSec = 4.0;
    public static double farFiringTaskWaitForFullTimeoutSec = 10.0;
    public static double farFiringTaskReturnTimeoutSec = 4.0;
    public static double farFiringTaskReadyToFireTimeoutSec = 4.0;
    public static double farFiringTaskStorageDrainTimeoutSec = 10.0;
    public static double farFiringTaskRepeatDelaySec = 0.5;
    public static double driverOverrideDeadband = 0.08;

    public static double fcsBaseVelocity = 2700;
    public static double fcsVelocityPerMeter = 620;
    public static double fcsMinVelocity = 3500;
    public static double fcsMaxVelocity = 5000;
    public static double fcsHoodBasePosition = 0.53;
    public static double fcsHoodPositionPerMeter = 0.1;
    public static boolean fcsUseDepotPoseFallbackWhenTagNotVisible = true;
    public static double fcsAlignedButLauncherOffHueDeg = 145.0;
    public static double blueDepotX = 59;
    public static double blueDepotY = 57;
    public static double redDepotX = 59;
    public static double redDepotY = -57;
    public static double fcsTurretToleranceDeg = 3;

    public static double turretKp = 0.02;
    public static double turretKi = 0;
    public static double turretKd = 0.015;
    public static double turretKf = 0;
    public static double turretToleranceDeg = 1;
    public static double turretTicksPerDegree = (8192.0 / 360.0) * (88.0 / 20.0);
    public static double turretMinAngleDeg = -90;
    public static double turretMaxAngleDeg = 90;

    public static double launcherKp = 0.002;
    public static double launcherKi = 0;
    public static double launcherKd = 0;
    public static double launcherKf = 0.00025;
    public static double launcherKv = 0.03;
    public static double launcherTolerance = 30;
    public static double launcherMaxVoltage = 14;
    public static double launcherTicksPerDegree = 112.0 / 360.0;

    public static double indexerKp = 0.007;
    public static double indexerKi = 0;
    public static double indexerKd = 0.0075;
    public static double indexerKf = 0.000005;
    public static double indexerToleranceDeg = 1;
    public static double indexerBusyToleranceDeg = 2.5;
    public static double indexerPoweredMovePower = 1;
    public static double indexerTicksPerDegree = 8192.0 / 360.0;

    public static double limelightLocalizerCameraX = 0;
    public static double limelightLocalizerCameraY = 0;
    public static double limelightLocalizerCameraZ = 0;
    public static double limelightLocalizerCameraHeadingDeg = 0;
    public static double blueDepotHeadingDeg = -40;
    public static double redDepotHeadingDeg = 40;
    public static double limelightCameraOnTurretRightOffsetMeters = 0.15;
    public static double limelightCameraOnTurretForwardOffsetMeters = 0;

    public static double feedRampMin = 0;
    public static double feedRampMax = 1;
    public static double feedRampEngagedPosition = 0.32;
    public static double feedRampDisengagedPosition = 1;
    public static double feedWheelPower = 1;

    public static int requiredConsecutiveContentReads = 1;
    public static double frontSensorMinDistance = 38;
    public static double greenHue = 159.0;
    public static double purpleHue = 170.0;
    public static int colorSensorHueFilterWindow = 0;
    public static int colorSensorSaturationFilterWindow = 0;
    public static int colorSensorValueFilterWindow = 0;
    public static int cameraColorSensorHueFilterWindow = 0;
    public static int cameraColorSensorSaturationFilterWindow = 0;
    public static int cameraColorSensorValueFilterWindow = 0;

    public static double volleyFirePrepareTimeMs = 250;
    public static double volleyFireEndTimeMs = 350;
    public static boolean volleyJamCorrectingEnabled = true;
    public static double volleyJamCorrectingTimeThresholdMs = 300;
    public static double volleyJamCorrectingVelocityThreshold = 25;
    public static double volleyJamCorrectingTimeMs = 1000;
}
