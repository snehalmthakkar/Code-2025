package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;

import org.junit.jupiter.api.Test;

import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoPickupTest {
    private void testAlterTranslationalVelocityToReachTarget(Translation2d translationalVelocity, Translation2d relativeTargetPosition, double strength, Translation2d expectedOutput) {
        Translation2d result = AutoPickup.alterTranslationalVelocityToReachTarget(translationalVelocity, relativeTargetPosition, strength);
        
        double tolerance = 1e-6;
        assert Math.abs(result.getX() - expectedOutput.getX()) < tolerance : "X component mismatch: expected " + expectedOutput.getX() + ", got " + result.getX();
        assert Math.abs(result.getY() - expectedOutput.getY()) < tolerance : "Y component mismatch: expected " + expectedOutput.getY() + ", got " + result.getY();
    }

    @Test
    public void testAlterTranslationalVelocityToReachTarget_Case1() {
        testAlterTranslationalVelocityToReachTarget(
            new Translation2d(1, 0),
            new Translation2d(1, 1),
            0.5,
            new Translation2d(1, 0.5)
        );
    }

    @Test
    public void testAlterTranslationalVelocityToReachTarget_Case2() {
        testAlterTranslationalVelocityToReachTarget(
            new Translation2d(0, -1),
            new Translation2d(1, -1),
            0.25,
            new Translation2d(0.25, -1)
        );
    }

    @Test
    public void testAlterTranslationalVelocityToReachTarget_Case3() {
        testAlterTranslationalVelocityToReachTarget(
            new Translation2d(0, -1).rotateBy(Rotation2d.fromDegrees(45)),
            new Translation2d(1, -1).rotateBy(Rotation2d.fromDegrees(45)),
            0.25,
            new Translation2d(0.25, -1).rotateBy(Rotation2d.fromDegrees(45))
        );
    }

    private void testAlterSpeedsToReachTarget(ChassisSpeeds speeds, Translation2d robotPose, Translation2d target, double strength, ChassisSpeeds expectedOutput) {
        ChassisSpeeds result = AutoPickup.alterSpeedsToReachTarget(speeds, robotPose, target, strength);
        
        double tolerance = 1e-6;
        assert Math.abs(result.vxMetersPerSecond - expectedOutput.vxMetersPerSecond) < tolerance : "vx mismatch: expected " + expectedOutput.vxMetersPerSecond + ", got " + result.vxMetersPerSecond;
        assert Math.abs(result.vyMetersPerSecond - expectedOutput.vyMetersPerSecond) < tolerance : "vy mismatch: expected " + expectedOutput.vyMetersPerSecond + ", got " + result.vyMetersPerSecond;
        assert Math.abs(result.omegaRadiansPerSecond - expectedOutput.omegaRadiansPerSecond) < tolerance : "omega mismatch: expected " + expectedOutput.omegaRadiansPerSecond + ", got " + result.omegaRadiansPerSecond;
    }

    @Test
    public void testAlterSpeedsToReachTarget_RobotAtOrigin() {
        testAlterSpeedsToReachTarget(
            new ChassisSpeeds(1, 0, 0),
            new Translation2d(0, 0),
            new Translation2d(1, 1),
            0.5,
            new ChassisSpeeds(1, 0.5, 0)
        );
    }

    @Test
    public void testAlterSpeedsToReachTarget_RobotNotAtOrigin() {
        testAlterSpeedsToReachTarget(
            new ChassisSpeeds(0, -1, 0),
            new Translation2d(1, 1),
            new Translation2d(2, 0),
            0.25,
            new ChassisSpeeds(0.25, -1, 0)
        );
    }

    @Test
    public void testAlterSpeedsToReachTarget_RobotRotated() {
        testAlterSpeedsToReachTarget(
            KinematicsUtils.rotateSpeeds(new ChassisSpeeds(0, -1, 0), Rotation2d.fromDegrees(45)),
            new Translation2d(1, 1).rotateBy(Rotation2d.fromDegrees(45)),
            new Translation2d(2, 0).rotateBy(Rotation2d.fromDegrees(45)),
            0.25,
            KinematicsUtils.rotateSpeeds(new ChassisSpeeds(0.25, -1, 0), Rotation2d.fromDegrees(45))
        );
    }

    private void testIsTargetReachable(Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d target, boolean expectedOutput) {
        boolean result = AutoPickup.isTargetReachable(robotPose, robotSpeeds, target);
        
        assert result == expectedOutput : "Expected " + expectedOutput + ", got " + result;
    }

    @Test
    public void testIsTargetReachable_SimpleCase1() {
        testIsTargetReachable(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new ChassisSpeeds(1, 0, 0),
            new Translation2d(1, 0),
            true
        );
    }

    @Test
    public void testIsTargetReachable_SimpleCase2() {
        testIsTargetReachable(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new ChassisSpeeds(-1, 0, 0),
            new Translation2d(1, 0),
            false
        );
    }

    @Test
    public void testIsTargetReachable_RotatedCase1() {
        testIsTargetReachable(
            new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
            new ChassisSpeeds(-1, 0, 0),
            new Translation2d(-1, 0),
            true
        );
    }

    @Test
    public void testIsTargetReachable_RotatedCase2() {
        testIsTargetReachable(
            new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
            new ChassisSpeeds(0, -1, 0),
            new Translation2d(0, -1),
            true
        );
    }

    @Test
    public void testIsTargetReachable_RotatedCase3() {
        testIsTargetReachable(
            new Pose2d(-2, 0, Rotation2d.fromDegrees(180)),
            new ChassisSpeeds(-1, 0, 0),
            new Translation2d(-1, 0),
            false
        );
    }

    @Test
    public void testIsTargetReachable_RotatedCase4() {
        testIsTargetReachable(
            new Pose2d(0.5, -4, Rotation2d.fromDegrees(270)),
            new ChassisSpeeds(0, -1, 0),
            new Translation2d(0, -1),
            false
        );
    }
}
