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
import frc.robot.vision.field.TrackingField;

public class AutoPickup {
    private SwerveDrive swerveDrive;
    private XBoxSwerve swerveController;
    private TrackingField trackingField;

    public AutoPickup(SwerveDrive swerveDrive, XBoxSwerve swerveController, TrackingField trackingField) {
        this.swerveDrive = swerveDrive;
        this.swerveController = swerveController;
        this.trackingField = trackingField;
    }

    public Command driftToCoral() {
        return driftToTarget(
            swerveDrive, swerveController,
            () -> {
                Translation2d coralTranslation = trackingField.getMostLikelyCoral().getTranslation();

                return new Pose2d(coralTranslation, coralTranslation.minus(swerveDrive.getEstimatedPose().getTranslation()).getAngle());
            },
            1.0
        );
    }

    private boolean isTargetReachable(Translation2d target) {
        return isTargetReachable(swerveDrive.getEstimatedPose(), swerveDrive.getEstimatedSpeeds(), target);
    }

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

    public static Pose2d getApproachWaypoint(Pose2d robotPose, Translation2d finalTarget, double approachDistance) {
        Vector2 targetToRobot = new Vector2(robotPose.getTranslation().minus(finalTarget)).unit();
        Vector2 targetToApproachWaypoint = targetToRobot.times(approachDistance);
        Vector2 approachWaypoint = new Vector2(finalTarget).plus(targetToApproachWaypoint);

        return approachWaypoint.toPose2d(targetToRobot.negate().toTranslation2d().getAngle());
    }

    public static Pose2d getPickupWaypoint(Pose2d robotPose, Translation2d finalTarget) {
        return getApproachWaypoint(robotPose, finalTarget, 0);
    }

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

    public static Angle maxAngularDifferenceForPickup = Degrees.of(30);
    private static double maxAngularDifferenceForPickupCos = Math.cos(maxAngularDifferenceForPickup.in(Radians));

    public static boolean isTargetReachable(Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d target) {
        Vector2 robotForward = new Vector2(robotPose.getRotation());
        Vector2 toTarget = new Vector2(target.minus(robotPose.getTranslation())).unit();
        Vector2 robotVelocity = new Vector2(robotSpeeds);

        double cosBetweenForwardAndTarget = robotForward.dot(toTarget);
        double cosBetweenVelocityAndTarget = robotVelocity.unit().dot(toTarget);

        return cosBetweenForwardAndTarget > maxAngularDifferenceForPickupCos && cosBetweenVelocityAndTarget > 0;
    }
}
