/**
 * AutoPickup.java
 *
 * Provides logic and helper methods for driving a swerve-drive robot to pick up "coral" game pieces
 * using field-relative and vision-based targeting. Includes methods to alter translational and chassis
 * speeds to improve approach vectors, calculate approach waypoints, and determine reachability of a target.
 * Commonly used in autonomous routines for auto-pickup of game pieces.
 */

package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.swerve.controller.LineFollowing.Vector2;
import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.XBoxSwerve;
import frc.robot.vision.CoralDetection;

/**
 * The AutoPickup class provides autonomous routines and helper logic for
 * automatically approaching and picking up game pieces ("coral").
 */
public class AutoPickup {
    private SwerveDrive swerveDrive;
    private XBoxSwerve swerveController;
    private CoralDetection coralDetection;

    /**
     * Constructs an AutoPickup instance with references to drive, controller, and vision.
     *
     * @param swerveDrive      The swerve drive subsystem
     * @param swerveController The Xbox controller wrapper for swerve
     * @param coralDetection   The vision subsystem for detecting coral positions
     */
    public AutoPickup(SwerveDrive swerveDrive, XBoxSwerve swerveController, CoralDetection coralDetection) {
        this.swerveDrive = swerveDrive;
        this.swerveController = swerveController;
        this.coralDetection = coralDetection;
    }

    /**
     * Creates a command to drift/drive toward the currently detected coral using vision.
     *
     * @return Command that drives toward the detected coral location
     */
    public Command driftToCoral() {
        return driftToTarget(
            swerveDrive, swerveController,
            () -> {
                Translation2d coralTranslation = coralDetection.getCoralLocation();

                return new Pose2d(coralTranslation, coralTranslation.minus(swerveDrive.getEstimatedPose().getTranslation()).getAngle());
            },
            1.0
        );
    }

    /**
     * Checks if a given target position is reachable from the robot's current pose and speeds.
     *
     * @param target The target Translation2d
     * @return true if reachable, false otherwise
     */
    private boolean isTargetReachable(Translation2d target) {
        return isTargetReachable(swerveDrive.getEstimatedPose(), swerveDrive.getEstimatedSpeeds(), target);
    }

    /**
     * Creates a command to drift/drive toward a supplied target pose, altering speeds for approach.
     *
     * @param drive      The swerve drive subsystem
     * @param controller The swerve controller
     * @param target     Supplier for the target pose
     * @param strength   The interpolation strength [0-1] for velocity adjustment
     * @return Command that drives toward the supplied target
     */
    private Command driftToTarget(SwerveDrive drive, XBoxSwerve controller, Supplier<Pose2d> target, double strength) {
        return Commands.parallel(
            drive.driveHeading(() -> target.get().getRotation()),
            controller.modifySpeeds(speeds -> alterSpeedsToReachTarget(
                speeds,
                drive.getEstimatedPose().getTranslation(),
                target.get().getTranslation(),
                strength
            ))
        )
            .onlyIf(() -> isTargetReachable(target.get().getTranslation()))
            .onlyWhile(() -> isTargetReachable(target.get().getTranslation()))
            .repeatedly();
    }

    /**
     * Calculates a waypoint at a specific approach distance from the target, based on the robot's pose.
     *
     * @param robotPose        The robot's current pose
     * @param finalTarget      The target translation
     * @param approachDistance The distance to stop before reaching the target
     * @return The approach waypoint Pose2d
     */
    public static Pose2d getApproachWaypoint(Pose2d robotPose, Translation2d finalTarget, double approachDistance) {
        Vector2 targetToRobot = new Vector2(robotPose.getTranslation().minus(finalTarget)).unit();
        Vector2 targetToApproachWaypoint = targetToRobot.times(approachDistance);
        Vector2 approachWaypoint = new Vector2(finalTarget).plus(targetToApproachWaypoint);

        return approachWaypoint.toPose2d(targetToRobot.negate().toTranslation2d().getAngle());
    }

    /**
     * Returns the pickup waypoint, which is the same as the final target (zero offset).
     *
     * @param robotPose   The robot's current pose
     * @param finalTarget The target translation
     * @return The pickup waypoint Pose2d
     */
    public static Pose2d getPickupWaypoint(Pose2d robotPose, Translation2d finalTarget) {
        return getApproachWaypoint(robotPose, finalTarget, 0);
    }

    /**
     * Alters the robot's translational velocity to better approach a target, interpolated by strength.
     *
     * @param translationalVelocity  The current translational velocity
     * @param relativeTargetPosition The target position relative to the robot
     * @param strength               Interpolation strength [0-1]
     * @return The adjusted translational velocity
     */
    public static Translation2d alterTranslationalVelocityToReachTarget(Translation2d translationalVelocity, Translation2d relativeTargetPosition, double strength) {
        Vector2 robotVelocity = new Vector2(translationalVelocity);
        Vector2 target = new Vector2(relativeTargetPosition);
        Vector2 robotVelocityUnit = robotVelocity.unit();
        Vector2 robotVelocityUnit90CCW = robotVelocity.rotate90CCW();
        
        Vector2 targetDirectionInMovingDirection = new Vector2(
            target.dot(robotVelocityUnit),
            target.dot(robotVelocityUnit90CCW)
        ).unit();

        Vector2 velocityToTargetInMovingDirection = targetDirectionInMovingDirection.times(robotVelocity.magnitude() / targetDirectionInMovingDirection.x);

        Vector2 velocityToTarget = robotVelocityUnit.times(velocityToTargetInMovingDirection.x)
            .plus(robotVelocityUnit90CCW.times(velocityToTargetInMovingDirection.y));
        
        Vector2 adjustedVelocity = velocityToTarget.times(strength).plus(robotVelocity.times(1 - strength));

        return new Translation2d(adjustedVelocity.x, adjustedVelocity.y);
    }

    /**
     * Alters the robot's chassis speeds to better approach a target, interpolated by strength.
     *
     * @param speeds    The current chassis speeds
     * @param robotPose The robot's current translation
     * @param target    The target translation
     * @param strength  Interpolation strength [0-1]
     * @return The adjusted ChassisSpeeds
     */
    public static ChassisSpeeds alterSpeedsToReachTarget(ChassisSpeeds speeds, Translation2d robotPose, Translation2d target, double strength) {
        return KinematicsUtils.setTranslationSpeed(
            speeds,
            alterTranslationalVelocityToReachTarget(
                KinematicsUtils.getTranslation(speeds),
                target.minus(robotPose),
                strength
            )
        );
    }

    /** The maximum angular difference (in degrees) allowed for a pickup to be considered "reachable". */
    public static Angle maxAngularDifferenceForPickup = Degrees.of(30);

    private static double maxAngularDifferenceForPickupCos = Math.cos(maxAngularDifferenceForPickup.in(Radians));

    /**
     * Determines if a target is reachable, based on current pose, velocity, and angle tolerances.
     *
     * @param robotPose   The robot's current pose
     * @param robotSpeeds The robot's current chassis speeds
     * @param target      The target translation
     * @return true if the target is within angular and velocity bounds, false otherwise
     */
    public static boolean isTargetReachable(Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d target) {
        Vector2 robotForward = new Vector2(robotPose.getRotation());
        Vector2 toTarget = new Vector2(target.minus(robotPose.getTranslation())).unit();
        Vector2 robotVelocity = new Vector2(robotSpeeds);

        double cosBetweenForwardAndTarget = robotForward.dot(toTarget);
        double cosBetweenVelocityAndTarget = robotVelocity.unit().dot(toTarget);

        return cosBetweenForwardAndTarget > maxAngularDifferenceForPickupCos && cosBetweenVelocityAndTarget > 0;
    }
}
