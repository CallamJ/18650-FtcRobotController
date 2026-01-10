package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.utilities.Pose;

public class FireControlSystem {
    private final Turret turret;
    private final Launcher launcher;
    private final Hood hood;
    private final StorageController storageController;

    // Physical constants - tunable
    public static double GRAVITY = 386.4; // inches/s^2 (32.2 ft/s^2)
    public static double MAX_LAUNCH_VELOCITY = 200.0; // inches/s, adjust based on your launcher
    public static double MIN_LAUNCH_VELOCITY = 30.0; // inches/s
    public static double AIR_RESISTANCE_FACTOR = 0.0; // 0.0 = no resistance, 0.1 = 10% velocity loss, adjust as needed

    // Tolerances for ready-to-fire check - tunable
    public static double TURRET_READY_TOLERANCE = 2.0; // degrees
    public static double HOOD_READY_TOLERANCE = 1.5; // degrees
    public static double VELOCITY_READY_TOLERANCE = 5.0; // inches/s
    public static double LANDING_POSITION_TOLERANCE = 6.0; // inches

    // Trajectory preferences - tunable
    public static double PREFERRED_MIN_LAUNCH_ANGLE = 35.0; // degrees, for high arc
    public static double PREFERRED_MAX_LAUNCH_ANGLE = 65.0; // degrees, for high arc
    public static double LOW_ARC_MIN_ANGLE = 25.0; // degrees, fallback if high arc impossible
    public static double LOW_ARC_MAX_ANGLE = 75.0; // degrees, fallback if high arc impossible
    public static double ANGLE_SEARCH_STEP = 0.5; // degrees, smaller = more accurate but slower

    // Velocity inheritance - tunable
    public static double LAUNCHER_VELOCITY_INHERITANCE = 1.0; // 1.0 = full inheritance, 0.5 = 50% of launcher velocity added to ball

    public FireControlSystem(Turret turret, Launcher launcher, Hood hood, StorageController storageController) {
        this.turret = turret;
        this.launcher = launcher;
        this.hood = hood;
        this.storageController = storageController;
    }

    public void tick(){
        turret.tick();
        launcher.tick();
        hood.tick();
    }

    public void updateTargetPositions(Pose goalPosition, Pose launcherPose, Pose launcherVelocityPose){
        // Calculate relative position from launcher to goal
        double dx = goalPosition.x() - launcherPose.x();
        double dy = goalPosition.y() - launcherPose.y();
        double dz = goalPosition.z() - launcherPose.z();

        // Calculate horizontal distance
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);

        // Calculate turret angle (yaw) to point at goal
        double targetTurretAngle = Math.toDegrees(Math.atan2(dy, dx));

        // Get launcher velocity components (ball starts with this velocity)
        double launcherVx = launcherVelocityPose.x() * LAUNCHER_VELOCITY_INHERITANCE;
        double launcherVy = launcherVelocityPose.y() * LAUNCHER_VELOCITY_INHERITANCE;
        double launcherVz = launcherVelocityPose.z() * LAUNCHER_VELOCITY_INHERITANCE;

        // Calculate optimal launch angle and velocity accounting for initial velocity
        // The ball will have both launcher velocity AND launch velocity
        BallisticSolution solution = calculateOptimalTrajectoryWithInitialVelocity(
                dx, dy, dz,
                launcherVx, launcherVy, launcherVz,
                targetTurretAngle,
                true
        );

        if (solution.isValid) {
            // Set turret angle
            turret.setTargetPosition(solution.turretAngle);

            // Set hood angle (launch angle)
            hood.setTargetLaunchAngle(solution.launchAngle);

            // Set launcher velocity
            launcher.setTargetVelocity(solution.launchVelocity);
        } else {
            // Target unreachable - point toward it but stop launcher
            turret.setTargetPosition(targetTurretAngle);
            launcher.setTargetVelocity(0);
        }
    }

    public boolean isReadyToFire(Pose goalPosition, Pose launcherPose, Pose launcherVelocityPose){
        // Check if all components are within tolerance of their targets
        boolean turretReady = Math.abs(turret.getTargetPosition() - turret.getCurrentPosition()) <= TURRET_READY_TOLERANCE;
        boolean hoodReady = Math.abs(hood.getTargetLaunchAngle() - hood.getCurrentLaunchAngle()) <= HOOD_READY_TOLERANCE;
        boolean launcherReady = Math.abs(launcher.getTargetVelocity() - launcher.getVelocity()) <= VELOCITY_READY_TOLERANCE;

        // Get current settings
        double currentVelocity = launcher.getVelocity();
        double currentAngle = hood.getCurrentLaunchAngle();
        double currentTurretAngle = turret.getCurrentPosition();

        if (currentVelocity < MIN_LAUNCH_VELOCITY) {
            return false; // Launcher not spinning fast enough
        }

        // Calculate relative position to goal
        double dx = goalPosition.x() - launcherPose.x();
        double dy = goalPosition.y() - launcherPose.y();
        double dz = goalPosition.z() - launcherPose.z();

        // Get launcher velocity components (ball starts with this velocity)
        double launcherVx = launcherVelocityPose.x() * LAUNCHER_VELOCITY_INHERITANCE;
        double launcherVy = launcherVelocityPose.y() * LAUNCHER_VELOCITY_INHERITANCE;
        double launcherVz = launcherVelocityPose.z() * LAUNCHER_VELOCITY_INHERITANCE;

        // Calculate where projectile will land with current settings AND initial velocity
        double turretRad = Math.toRadians(currentTurretAngle);
        double angleRad = Math.toRadians(currentAngle);

        // Launch velocity components in world frame
        double launchVx = currentVelocity * Math.cos(angleRad) * Math.cos(turretRad);
        double launchVy = currentVelocity * Math.cos(angleRad) * Math.sin(turretRad);
        double launchVz = currentVelocity * Math.sin(angleRad);

        // Total initial velocity (launcher velocity + launch velocity)
        double totalVx = launcherVx + launchVx;
        double totalVy = launcherVy + launchVy;
        double totalVz = launcherVz + launchVz;

        // Calculate time of flight
        // Solve: dz = totalVz * t - 0.5 * g * t^2
        // Using quadratic formula: t = (totalVz + sqrt(totalVz^2 + 2*g*dz)) / g
        double discriminant = totalVz * totalVz + 2 * GRAVITY * dz;
        if (discriminant < 0) {
            return false; // Can't reach target
        }

        double timeOfFlight = (totalVz + Math.sqrt(discriminant)) / GRAVITY;

        // Calculate landing position
        double landingX = launcherPose.x() + totalVx * timeOfFlight;
        double landingY = launcherPose.y() + totalVy * timeOfFlight;

        // Calculate distance from landing position to goal
        double landingError = Math.sqrt(
                Math.pow(landingX - goalPosition.x(), 2) +
                        Math.pow(landingY - goalPosition.y(), 2)
        );

        // Check if predicted landing is close to target
        boolean trajectoryValid = landingError <= LANDING_POSITION_TOLERANCE;

        return turretReady && hoodReady && launcherReady && trajectoryValid;
    }

    /**
     * Calculates the optimal ballistic trajectory to reach a target, accounting for initial velocity.
     * @param dx x distance to target (inches)
     * @param dy y distance to target (inches)
     * @param dz vertical distance to target (inches, positive if target is higher)
     * @param initialVx initial x velocity from launcher motion (inches/s)
     * @param initialVy initial y velocity from launcher motion (inches/s)
     * @param initialVz initial z velocity from launcher motion (inches/s)
     * @param nominalTurretAngle the basic turret angle toward target (degrees)
     * @param preferHighArc if true, prefers a higher arc for softer landing
     * @return BallisticSolution containing turret angle, launch angle and velocity
     */
    private BallisticSolution calculateOptimalTrajectoryWithInitialVelocity(
            double dx, double dy, double dz,
            double initialVx, double initialVy, double initialVz,
            double nominalTurretAngle, boolean preferHighArc) {

        double bestVelocity = Double.MAX_VALUE;
        double bestAngle = 45.0;
        double bestTurretAngle = nominalTurretAngle;
        boolean foundSolution = false;

        // Search through possible angles
        double minAngle = preferHighArc ? PREFERRED_MIN_LAUNCH_ANGLE : LOW_ARC_MIN_ANGLE;
        double maxAngle = preferHighArc ? PREFERRED_MAX_LAUNCH_ANGLE : LOW_ARC_MAX_ANGLE;

        // Also search through small turret angle adjustments to compensate for lateral initial velocity
        for (double turretOffset = -5.0; turretOffset <= 5.0; turretOffset += 1.0) {
            double turretAngle = nominalTurretAngle + turretOffset;
            double turretRad = Math.toRadians(turretAngle);

            for (double launchAngle = minAngle; launchAngle <= maxAngle; launchAngle += ANGLE_SEARCH_STEP) {
                double angleRad = Math.toRadians(launchAngle);

                // Try different launch velocities
                for (double launchVelocity = MIN_LAUNCH_VELOCITY; launchVelocity <= MAX_LAUNCH_VELOCITY; launchVelocity += 5.0) {

                    // Launch velocity components in world frame
                    double launchVx = launchVelocity * Math.cos(angleRad) * Math.cos(turretRad);
                    double launchVy = launchVelocity * Math.cos(angleRad) * Math.sin(turretRad);
                    double launchVz = launchVelocity * Math.sin(angleRad);

                    // Total initial velocity
                    double totalVx = initialVx + launchVx;
                    double totalVy = initialVy + launchVy;
                    double totalVz = initialVz + launchVz;

                    // Calculate time of flight
                    double discriminant = totalVz * totalVz + 2 * GRAVITY * dz;
                    if (discriminant < 0) continue;

                    double timeOfFlight = (totalVz + Math.sqrt(discriminant)) / GRAVITY;

                    // Calculate landing position
                    double landingDx = totalVx * timeOfFlight;
                    double landingDy = totalVy * timeOfFlight;

                    // Check if this lands near the target
                    double positionError = Math.sqrt(
                            Math.pow(landingDx - dx, 2) +
                                    Math.pow(landingDy - dy, 2)
                    );

                    // If this is a valid solution and uses less launch velocity (softer landing)
                    if (positionError <= LANDING_POSITION_TOLERANCE && launchVelocity < bestVelocity) {
                        bestVelocity = launchVelocity;
                        bestAngle = launchAngle;
                        bestTurretAngle = turretAngle;
                        foundSolution = true;
                    }
                }
            }
        }

        // If no solution found with preferred arc, try with wider angle range
        if (!foundSolution && preferHighArc) {
            return calculateOptimalTrajectoryWithInitialVelocity(
                    dx, dy, dz,
                    initialVx, initialVy, initialVz,
                    nominalTurretAngle, false
            );
        }

        return new BallisticSolution(foundSolution, bestTurretAngle, bestAngle, bestVelocity);
    }

    /**
     * Helper class to store ballistic calculation results
     */
    private static class BallisticSolution {
        final boolean isValid;
        final double turretAngle; // degrees
        final double launchAngle; // degrees
        final double launchVelocity; // inches/s

        BallisticSolution(boolean isValid, double turretAngle, double launchAngle, double launchVelocity) {
            this.isValid = isValid;
            this.turretAngle = turretAngle;
            this.launchAngle = launchAngle;
            this.launchVelocity = launchVelocity;
        }
    }
}