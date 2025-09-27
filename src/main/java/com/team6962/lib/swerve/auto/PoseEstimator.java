package com.team6962.lib.swerve.auto;

import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;
import com.team6962.lib.utils.RotationUtils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.LIMELIGHT;
import frc.robot.vision.AprilTags;

import java.util.function.Supplier;

/**
 * {@code PoseEstimator} is a class that estimates the pose of a swerve drive robot using a
 * combination of odometry, gyroscope, and vision measurements. A PoseEstimator contains a {@link
 * SwerveDrivePoseEstimator} that is updated with the latest measurements. It also contains a {@link
 * SwerveGyroscope} that can be used to get and reset the robot's heading.
 *
 * <p>Vision data can be added with {@link #addVisionMeasurement(Pose2d, Time)}.
 */
public class PoseEstimator extends SubsystemBase implements RobotCoordinates {
  private SwerveDriveKinematics kinematics;
  private SwerveGyroscope gyroscope;
  private SwerveDrivePoseEstimator poseEstimator;

  private Supplier<SwerveModulePosition[]> modulePositionsSupplier;
  private Supplier<SwerveModuleState[]> moduleStatesSupplier;

  private SwerveModulePosition[] lastPositions;
  private SwerveModulePosition[] positionChanges = KinematicsUtils.blankModulePositions(4);
  private Twist2d chassisVelocity = new Twist2d();

  public PoseEstimator(
      SwerveDriveKinematics kinematics,
      Supplier<SwerveModulePosition[]> modulePositions,
      Supplier<SwerveModuleState[]> moduleStates,
      SwerveConfig config) {
    this.kinematics = kinematics;
    this.gyroscope = SwerveGyroscope.get(() -> positionChanges, kinematics, config);
    this.modulePositionsSupplier = modulePositions;
    this.moduleStatesSupplier = moduleStates;

    lastPositions = modulePositions.get();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            RotationUtils.fromAngle(gyroscope.getHeading()),
            modulePositions.get(),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    Logger.logSpeeds(getName() + "/estimatedSpeeds", this::getEstimatedSpeeds);
    Logger.logSwerveModulePositions(getName() + "/lastModulePositions", () -> lastPositions);
    Logger.logSwerveModulePositions(getName() + "/modulePositions", modulePositions);
    Logger.logSwerveModulePositions(getName() + "/positionChanges", () -> positionChanges);
  }

  @Override
  public void periodic() {
    SwerveModulePosition[] modulePositions = this.modulePositionsSupplier.get();
    Time timestamp = Seconds.of(Timer.getFPGATimestamp());

    poseEstimator.updateWithTime(
        timestamp.in(Seconds), RotationUtils.fromAngle(gyroscope.getHeading()), modulePositions);
    AprilTags.injectVisionData(LIMELIGHT.APRILTAG_CAMERA_POSES, this);
    chassisVelocity =
        kinematics.toTwist2d(
            KinematicsUtils.toModulePositions(moduleStatesSupplier.get(), Seconds.of(1.0)));

    positionChanges = KinematicsUtils.difference(modulePositions, lastPositions);

    lastPositions = modulePositions;
  }

  public Angle getContinuousGyroscopeAngle() {
    return gyroscope.getHeading();
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters, Time timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestamp.in(Seconds), visionMeasurementStdDevs);
  }

  public void resetPoseEstimate(Pose2d expectedFieldPose) {
    poseEstimator.resetPosition(
        RotationUtils.fromAngle(gyroscope.getHeading()),
        modulePositionsSupplier.get(),
        expectedFieldPose);
  }

  public void addVisionHeading(Rotation2d expectedAbsoluteHeading) {
    resetPoseEstimate(new Pose2d(getEstimatedPose().getTranslation(), expectedAbsoluteHeading));
  }

  public void resetHeadingEstimate(Rotation2d expectedFieldHeading) {
    resetPoseEstimate(new Pose2d(getEstimatedPose().getTranslation(), expectedFieldHeading));
  }

  @Override
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getFuturePose(Time time) {
    Pose2d currentPose = getEstimatedPose();

    if (chassisVelocity == null) return currentPose;

    double deltaSeconds = time.in(Seconds);
    Twist2d deltaTwist =
        new Twist2d(
            chassisVelocity.dx * deltaSeconds,
            chassisVelocity.dy * deltaSeconds,
            chassisVelocity.dtheta * deltaSeconds);

    return currentPose.exp(deltaTwist);
  }

  public Pose2d getEstimatedPose(Time timestamp) {
    return poseEstimator.sampleAt(timestamp.in(Seconds)).orElseGet(this::getEstimatedPose);
  }

  public ChassisSpeeds getEstimatedSpeeds() {
    return robotToField(kinematics.toChassisSpeeds(moduleStatesSupplier.get()));
  }
}
