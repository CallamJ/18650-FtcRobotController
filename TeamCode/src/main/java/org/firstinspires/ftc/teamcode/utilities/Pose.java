package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/**
 * <h3>A class intended to be a simple, universal bridge between different Pose classes.</h3>
 * <pre><code>
 *     //Create a new Pose
 *     Pose newPose2D = new Pose(15, 25, 45);
 *     Pose newPose3D = new Pose(15, 25, 45, 120, 76, 50);
 *
 *     //Create a pose from a FTC Nav Pose2D
 *     Pose2D navPose2D1 = new Pose2D(DistanceUnit.INCH, 5, 10, AngleUnit.DEGREES, 45);
 *     Pose fromNav2D = Pose.from(navPose2D1);
 *
 *     //Create a pose from a FTC Nav Pose3D
 *     Pose2D navPose3D1 = new Pose3D(DistanceUnit.INCH, 5, 10, 2, AngleUnit.DEGREES, 45, 30, 60);
 *     Pose fromNav3D = Pose.from(navPose3D1);
 *
 *     //Convert a pose to another type of pose
 *     Pose2D navPose2D2 = newPose2D.toNav();
 *     Pose3D navPose3D2 = newPose3D.toNav3D();
 *
 *     //poses can be cross-converted even between 2D and 3D
 *     Pose myPose = Pose.from(new Pose2d(50, 30, 45));
 *     Pose3D myNavPose = myPose.toNav3D();
 * </code></pre>
 * <hr>
 * <br>
 * <h1>Visualizing Coordinates:</h1>
 *
 * <pre>
 *                               ,_,
 *                               |_/
 *                               ||
 *                             +-|_/-------------------------------------------------+
 *                            /#+||-------------------------------------------------/|
 *                           /#/ || _/                                       /     /#|    ___________
 *             ___________  /#/  _/‾              +z   -x                   /     /#/    /          /
 *            /          / /#/‾‾                   |   /                    \    /#/    /          /
 *           /          / /#/                      |  /                      \  /#/    /          /
 *          /          / /#/                       | /                        \/#/    /          /
 *         /          / /#/                        |/                         /#/    /   BLUE   /
 *        /   RED    / /#/          -x ____________|____________ +y          /#/    /          /
 *       /          / /#/                         /               _,_,      /#/    /          /
 *      /          / /#/ \                       /                \|./     /#/    /          /
 *     /          / /#/   \                     /                 _| |    /#/    /          /
 *    /          / /#/     \                   /                  \|./   /#/    /__________/
 *   /          / /#/      /                 +x                    | |_./#/
 *  /__________/ +--------+----------------------------------------+-+-+#/
 *               |/      /                                       / | | |/
 *               +-----------------------------------------------------+
 *  </pre>
 *  <br>
 *  <h1>Visualizing Orientation:</h1>
 *  <hr>
 *  <pre>
 *              &#x293A; yaw (heading)
 *              |
 *              |  / &#x2197; forward &#x2197;
 *              | /
 *              |/
 *    __________|__________ &#x2939; pitch
 *             /
 *            /
 *           /
 *          &#x21B6; roll
 *  </pre>
 */
public class Pose {
    private final double x, y, z;
    private final double yaw, pitch, roll;
    private final long acquisitionTime;

    private Pose(double x, double y, double z, double yaw, double pitch, double roll, AngleUnit angleUnit, DistanceUnit distanceUnit, long acquisitionTime) {
        this.x = distanceUnit.fromUnit(distanceUnit, x);
        this.y = distanceUnit.fromUnit(distanceUnit, y);
        this.z = distanceUnit.fromUnit(distanceUnit, z);
        this.yaw = AngleUnit.normalizeDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, yaw));
        this.pitch = AngleUnit.normalizeDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, pitch));
        this.roll = AngleUnit.normalizeDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, roll));
        this.acquisitionTime = acquisitionTime;
    }

    public static Pose from(Pose2D navPose) {
        return new Pose(
                navPose.getX(DistanceUnit.INCH),
                navPose.getY(DistanceUnit.INCH),
                0,
                navPose.getHeading(AngleUnit.DEGREES),
                0,
                0,
                AngleUnit.DEGREES,
                DistanceUnit.INCH,
                0
        );
    }

    public static Pose from(Pose3D navPose3D) {
        Position position = navPose3D.getPosition();
        YawPitchRollAngles angles = navPose3D.getOrientation();
        return new Pose(
                position.x,
                position.y,
                position.z,
                AngleUnit.DEGREES.normalize(angles.getYaw(AngleUnit.DEGREES) + 180),
                angles.getPitch(AngleUnit.DEGREES),
                angles.getRoll(AngleUnit.DEGREES),
                AngleUnit.DEGREES,
                position.unit,
                position.acquisitionTime
        );
    }

    public Pose(double xInches, double yInches, double headingDegrees) {
        this(
                xInches,
                yInches,
                0,
                headingDegrees,
                0,
                0,
                AngleUnit.DEGREES,
                DistanceUnit.INCH,
                0
        );
    }

    public Pose(double xInches, double yInches, double zInches, double yawDegrees, double pitchDegrees, double rollDegrees) {
        this(
                xInches,
                yInches,
                0,
                yawDegrees,
                0,
                0,
                AngleUnit.DEGREES,
                DistanceUnit.INCH,
                0
        );
    }

    public Pose(double xInches, double yInches) {
        this(xInches, yInches, 0);
    }

    /**
     * @return the x coordinate in inches.
     */
    public double x() {
        return x;
    }

    /**
     * @return the y coordinate in inches.
     */
    public double y() {
        return y;
    }

    /**
     * @return the z coordinate in inches.
     */
    public double z() {
        return z;
    }

    /**
     * @return the yaw angle in degrees.
     * @apiNote equivalent to heading()
     */
    public double yaw() {
        return yaw;
    }

    /**
     * @return the heading angle in degrees.
     * @apiNote equivalent to yaw()
     */
    public double heading() {
        return yaw;
    }

    /**
     * @return the pitch angle in degrees.
     */
    public double pitch() {
        return pitch;
    }

    /**
     * @return the roll angle in degrees.
     */
    public double roll() {
        return roll;
    }


    /**
     * @param unit the distance unit to return the coordinate in
     * @return the x coordinate in the provided distance unit.
     */
    public double x(DistanceUnit unit) {
        return unit.fromUnit(DistanceUnit.INCH, x);
    }

    /**
     * @param unit the distance unit to return the coordinate in
     * @return the y coordinate in the provided distance unit.
     */
    public double y(DistanceUnit unit) {
        return unit.fromUnit(DistanceUnit.INCH, y);
    }

    /**
     * @param unit the distance unit to return the coordinate in
     * @return the z coordinate in the provided distance unit.
     */
    public double z(DistanceUnit unit) {
        return unit.fromUnit(DistanceUnit.INCH, z);
    }

    /**
     * @param unit the angle unit to return the angle in
     * @return the yaw angle in the provided angle unit.
     * @apiNote equivalent to heading()
     */
    public double yaw(AngleUnit unit) {
        return unit.fromUnit(AngleUnit.DEGREES, yaw);
    }

    /**
     * @param unit the angle unit to return the angle in
     * @return the heading angle in the provided angle unit.
     * @apiNote equivalent to heading()
     */
    public double heading(AngleUnit unit) {
        return unit.fromUnit(AngleUnit.DEGREES, yaw);
    }

    /**
     * @param unit the angle unit to return the angle in
     * @return the pitch angle in the provided angle unit.
     * @apiNote equivalent to heading()
     */
    public double pitch(AngleUnit unit) {
        return unit.fromUnit(AngleUnit.DEGREES, pitch);
    }

    /**
     * @param unit the angle unit to return the angle in
     * @return the roll angle in the provided angle unit.
     * @apiNote equivalent to heading()
     */
    public double roll(AngleUnit unit) {
        return unit.fromUnit(AngleUnit.DEGREES, roll);
    }

    /**
     * Creates a new Pose with the specified offsets added to the current pose.
     *
     * @param xOffsetInches     The offset to add to the x coordinate (in inches).
     * @param yOffsetInches     The offset to add to the y coordinate (in inches).
     * @param zOffsetInches     The offset to add to the z coordinate (in inches).
     * @param yawOffsetDegrees  The offset to add to the yaw angle (in degrees).
     * @param pitchOffsetDegrees The offset to add to the pitch angle (in degrees).
     * @param rollOffsetDegrees  The offset to add to the roll angle (in degrees).
     * @return A new Pose with the updated values.
     */
    public Pose withOffset(
            double xOffsetInches, double yOffsetInches, double zOffsetInches,
            double yawOffsetDegrees, double pitchOffsetDegrees, double rollOffsetDegrees
    ) {
        return new Pose(
                x + xOffsetInches, y + yOffsetInches, z + zOffsetInches,
                yaw + yawOffsetDegrees, pitch + pitchOffsetDegrees, roll + rollOffsetDegrees
        );
    }

    /**
     * @param xOffsetInches The amount to add to the x coordinate (in inches).
     * @return A new Pose with the x coordinate offset by the given amount.
     */
    public Pose plusX(double xOffsetInches) {
        return withOffset(xOffsetInches, 0, 0, 0, 0, 0);
    }

    /**
     * @param yOffsetInches The amount to add to the y coordinate (in inches).
     * @return A new Pose with the y coordinate offset by the given amount.
     */
    public Pose plusY(double yOffsetInches) {
        return withOffset(0, yOffsetInches, 0, 0, 0, 0);
    }

    /**
     * @param zOffsetInches The amount to add to the z coordinate (in inches).
     * @return A new Pose with the z coordinate offset by the given amount.
     */
    public Pose plusZ(double zOffsetInches) {
        return withOffset(0, 0, zOffsetInches, 0, 0, 0);
    }

    /**
     * @param yawOffsetDegrees The amount to add to the yaw (heading) angle (in degrees).
     * @return A new Pose with the yaw angle offset by the given amount.
     */
    public Pose plusYaw(double yawOffsetDegrees) {
        return withOffset(0, 0, 0, yawOffsetDegrees, 0, 0);
    }

    /**
     * @param pitchOffsetDegrees The amount to add to the pitch angle (in degrees).
     * @return A new Pose with the pitch angle offset by the given amount.
     */
    public Pose plusPitch(double pitchOffsetDegrees) {
        return withOffset(0, 0, 0, 0, pitchOffsetDegrees, 0);
    }

    /**
     * @param rollOffsetDegrees The amount to add to the roll angle (in degrees).
     * @return A new Pose with the roll angle offset by the given amount.
     */
    public Pose plusRoll(double rollOffsetDegrees) {
        return withOffset(0, 0, 0, 0, 0, rollOffsetDegrees);
    }

    /**
     * Calculates the Euclidean distance between this Pose and another Pose.
     *
     * @param otherPose The Pose to calculate the distance to.
     * @return The distance to the other Pose in inches.
     */
    public double distanceTo(Pose otherPose) {
        double dx = otherPose.x - x;
        double dy = otherPose.y - y;
        double dz = otherPose.z - z;
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2) + Math.pow(dz, 2));
    }

    /**
     * Converts this Pose into an FTC Navigation Pose2D.
     *
     * @return A {@link Pose2D} representation of this Pose.
     */
    public Pose2D toNav() {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, yaw);
    }

    /**
     * Converts this Pose into an FTC Navigation Pose3D.
     *
     * @return A {@link Pose3D} representation of this Pose.
     */
    public Pose3D toNav3D() {
        return new Pose3D(new Position(DistanceUnit.INCH, x, y, z, acquisitionTime), new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, acquisitionTime));
    }

    /**
     * @return A {@link Position} object representing this Pose's position.
     */
    public Position getPosition() {
        return new Position(DistanceUnit.INCH, x, y, z, 0);
    }

    /**
     * @return A {@link YawPitchRollAngles} object representing this Pose's orientation.
     */
    public YawPitchRollAngles getAngles() {
        return new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0);
    }

    /**
     * Provides a formatted string representation of the Pose.
     *
     * @return A string representing the Pose's position and orientation.
     */
    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ROOT, "Position(x=%.3f, y=%.3f, z=%.3f, yaw=%.3f, pitch=%.3f, roll=%.3f)",
                x, y, z, yaw, pitch, roll);
    }
}
