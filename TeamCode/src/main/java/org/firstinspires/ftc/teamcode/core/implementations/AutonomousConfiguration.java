package org.firstinspires.ftc.teamcode.core.implementations;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

@Configurable
public final class AutonomousConfiguration {
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
