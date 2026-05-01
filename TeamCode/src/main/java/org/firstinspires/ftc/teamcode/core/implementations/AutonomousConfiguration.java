package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

@Configurable
public final class AutonomousConfiguration {
    public static boolean runFCS = true;
    public static double targetVelocity = 2000;
    public static double matchStateSaveIntervalMs = 500;
    public static boolean enableObeliskAcquisitionAssist = true;
    public static double obeliskAcquisitionTimeoutSec = 3.0;
    public static double blueObeliskPoseX = 72;
    public static double blueObeliskPoseY = 0;
    public static double redObeliskPoseX = 72;
    public static double redObeliskPoseY = 0;
    public static double autoFollowerMaxPower = 1.0;
    public static double autoPathEndVelocityConstraint = 0.1;
    public static double autoPathEndTranslationalConstraint = 0.1;
    public static double autoPathEndHeadingConstraint = 0.007;
    public static double autoPathEndTValueConstraint = 0.995;
    public static double autoPathEndTimeoutConstraintMs = 100.0;
    public static double autoPathBrakingStrength = 1.0;
    public static double autoPathBrakingStart = 1.0;
    public static int autoPathBezierSearchLimit = 10;
    public static boolean autoPathUseGlobalDeceleration = false;
    public static boolean autoPathDisableDeceleration = false;

    public static double closeAutoStartBluePoseXIn = 54.7;
    public static double closeAutoStartBluePoseYIn = 50.8;
    public static double closeAutoStartBluePoseHeadingDeg = -127.483;
    public static double closeAutoStartRedPoseXIn = 54.7;
    public static double closeAutoStartRedPoseYIn = -50.8;
    public static double closeAutoStartRedPoseHeadingDeg = 127.483;

    private AutonomousConfiguration() {}

    public static double closeAutoStartPoseXIn(MatchStateStore.AllianceColor allianceColor) {
        return isRedAlliance(allianceColor) ? closeAutoStartRedPoseXIn : closeAutoStartBluePoseXIn;
    }

    public static double closeAutoStartPoseYIn(MatchStateStore.AllianceColor allianceColor) {
        return isRedAlliance(allianceColor) ? closeAutoStartRedPoseYIn : closeAutoStartBluePoseYIn;
    }

    public static double closeAutoStartPoseHeadingDeg(MatchStateStore.AllianceColor allianceColor) {
        return isRedAlliance(allianceColor) ? closeAutoStartRedPoseHeadingDeg : closeAutoStartBluePoseHeadingDeg;
    }

    private static boolean isRedAlliance(MatchStateStore.AllianceColor allianceColor) {
        return allianceColor == MatchStateStore.AllianceColor.RED;
    }
}
