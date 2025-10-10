package frc.robot.auto;

/**
 * AutoAlign.java
 *
 * This class provides autonomous and teleoperated alignment routines for a swerve-drive FRC robot.
 * It offers commands to align the robot with specific field elements (poles, faces, barge) using geometric calculations,
 * FRC field mappings, and pose estimation.
 *
 * Main Features:
 * - Aligns with the closest reef pole or face for scoring or pickup.
 * - Offers alignment routines for different pole patterns (LEFT, RIGHT, ALL).
 * - Supports both precise autonomous alignment and teleoperated alignment with feedback.
 * - Provides special alignment for a "barge" area, including setup and fast alignment.
 * - Integrates with field-specific positioning utilities (ReefPositioning).
 *
 * All alignment commands return WPILib Command-based routines for use in robot programs.
 * 
 * Dependencies:
 * - SwerveDrive: Custom swerve drive library for movement and pose estimation.
 * - ReefPositioning: Field-specific pose utilities for alignment targets.
 * - WPILib Command framework and geometry/math libraries.
 */

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.field.ReefPositioning;

public class AutoAlign {
  // Reference to the SwerveDrive subsystem for movement and pose estimation
  private SwerveDrive swerveDrive;

  /**
   * Constructor for AutoAlign.
   * @param swerveDrive The SwerveDrive subsystem used for driving and pose estimation.
   */
  public AutoAlign(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  /**
   * Enum representing pole alignment patterns for limiting which poles to consider.
   * LEFT: Uses poles 1, 3, 5, 7, 9, 11
   * RIGHT: Uses poles 0, 2, 4, 6, 8, 10
   * ALL: Uses every pole
   */
  public static enum PolePattern {
    LEFT(1, 2),
    RIGHT(0, 2),
    ALL(0, 1);

    public final int start;
    public final int increment;

    private PolePattern(int start, int increment) {
      this.start = start;
      this.increment = increment;
    }
  }

  /**
   * Finds the index of the closest reef pole to a given pose, limited by the specified pattern.
   * Uses field positions from ReefPositioning.
   * 
   * @param pose The robot's current pose.
   * @param pattern The pole pattern to consider (LEFT, RIGHT, ALL).
   * @return Index of the closest reef pole.
   */
  private int getClosestReefPole(Pose2d pose, PolePattern pattern) {
    int closestPole = 0;
    double closestDistance = Double.MAX_VALUE;

    for (int i = pattern.start; i < 12; i += pattern.increment) {
      Translation2d pole = ReefPositioning.getCoralAlignPose(i).getTranslation();

      if (pose.getTranslation().getDistance(pole) < closestDistance) {
        closestPole = i;
        closestDistance = pose.getTranslation().getDistance(pole);
      }
    }

    return closestPole;
  }

  /**
   * Finds the index of the closest reef pole using a custom pose supplier for the pole positions.
   * 
   * @param pose The robot's current pose.
   * @param poseSupplier Function mapping pole index to its field pose.
   * @param pattern The pole pattern to consider.
   * @return Index of the closest reef pole.
   */
  private int getClosestReefPole(Pose2d pose, Function<Integer, Pose2d> poseSupplier, PolePattern pattern) {
    int closestPole = 0;
    double closestDistance = Double.MAX_VALUE;

    for (int i = pattern.start; i < 12; i += pattern.increment) {
      Translation2d pole = poseSupplier.apply(i).getTranslation();

      if (pose.getTranslation().getDistance(pole) < closestDistance) {
        closestPole = i;
        closestDistance = pose.getTranslation().getDistance(pole);
      }
    }

    return closestPole;
  }

  /**
   * Finds the index of the closest "reef face" (algae pickup location) to the given pose.
   *
   * @param pose The robot's current pose.
   * @return Index of the closest reef face.
   */
  public int getClosestReefFace(Pose2d pose) {
    int closestFace = 0;
    double closestDistance = Double.MAX_VALUE;

    for (int i = 0; i < 6; i++) {
      Translation2d face = ReefPositioning.getAlgaeAlignPose(i).getTranslation();

      if (pose.getTranslation().getDistance(face) < closestDistance) {
        closestFace = i;
        closestDistance = pose.getTranslation().getDistance(face);
      }
    }

    return closestFace;
  }

  /**
   * Returns a command that aligns the robot with the specified pole using a precise drive command.
   *
   * @param pole Index of the pole to align with.
   * @param endWithinTolerance If true, the command ends when within tolerance of the target.
   * @return Command for pole alignment.
   */
  public Command alignPole(int pole, boolean endWithinTolerance) {
    return swerveDrive
            .driveTo(ReefPositioning.getCoralPlacePose(pole))
            .until(
                () ->
                    endWithinTolerance
                        && swerveDrive.isWithinToleranceOf(
                            ReefPositioning.getCoralPlacePose(pole),
                            Inches.of(1),
                            Degrees.of(3)));
  }

  /**
   * Returns a command that pathfinds and aligns the robot with the L1 scoring location for a pole.
   *
   * @param pole Index of the pole.
   * @param endWithinTolerance End when within tolerance, if true.
   * @return Command for L1 pole alignment.
   */
  public Command alignL1(int pole, boolean endWithinTolerance) {
    return swerveDrive
        .pathfindTo(ReefPositioning.getL1PlacePose(pole))
        .andThen(
            swerveDrive
                .driveTo(ReefPositioning.getL1PlacePose(pole))
                .until(
                    () ->
                        endWithinTolerance
                            && swerveDrive.isWithinToleranceOf(
                                ReefPositioning.getL1PlacePose(pole),
                                Inches.of(1),
                                Degrees.of(3))));
  }

  /**
   * Returns a command to align the robot with a specific algae pickup face.
   *
   * @param face Index of the reef face.
   * @param endWithinTolerance End when within tolerance, if true.
   * @return Command for face alignment.
   */
  public Command alignFace(int face, boolean endWithinTolerance) {
    return swerveDrive
              .driveTo(ReefPositioning.getAlgaePickupPose(face))
              .until(
                  () ->
                      endWithinTolerance
                          && swerveDrive.isWithinToleranceOf(
                              ReefPositioning.getAlgaePickupPose(face),
                              Inches.of(1),
                              Degrees.of(3)));
  }

  /**
   * Teleop command to align to the closest pole, ending with a rumble (controller feedback) once within tolerance.
   *
   * @param pattern Which set of poles to consider.
   * @param rumble Supplier of a feedback command.
   * @return Command for teleop pole alignment with rumble.
   */
  public Command alignToClosestPoleTeleop(PolePattern pattern, Supplier<Command> rumble) {
    return Commands.defer(
        () -> {
          int pole = getClosestReefPole(swerveDrive.getEstimatedPose(), pattern);
          Pose2d polePose = ReefPositioning.getCoralPlacePose(pole);

          return alignPole(pole, false)
              .alongWith(
                  Commands.waitUntil(
                          () ->
                              swerveDrive.isWithinToleranceOf(
                                  polePose, Inches.of(1), Degrees.of(3)))
                      .andThen(rumble.get()));
        },
        Set.of(swerveDrive.useMotion()));
  }

  /**
   * Teleop command to align to the closest L1 pole, ending with a rumble once within tolerance.
   *
   * @param pattern Which set of poles to consider.
   * @param rumble Supplier of a feedback command.
   * @return Command for teleop L1 alignment with rumble.
   */
  public Command alignToClosestL1Teleop(PolePattern pattern, Supplier<Command> rumble) {
    return Commands.defer(
        () -> {
          int pole = getClosestReefPole(swerveDrive.getEstimatedPose(), ReefPositioning::getL1PlacePose, pattern);
          Pose2d polePose = ReefPositioning.getL1PlacePose(pole);

          return alignL1(pole, false)
              .alongWith(
                  Commands.waitUntil(
                          () ->
                              swerveDrive.isWithinToleranceOf(
                                  polePose, Inches.of(1.5), Degrees.of(5)))
                      .andThen(rumble.get()));
        },
        Set.of(swerveDrive.useMotion()));
  }

  /**
   * Teleop command to align to the closest face for algae pickup.
   *
   * @return Command for teleop face alignment.
   */
  public Command alignToClosestFaceTeleop() {
    return Commands.defer(
        () -> alignFace(getClosestReefFace(swerveDrive.getEstimatedPose()), false),
        Set.of(swerveDrive.useMotion()));
  }

  // Constants for barge alignment region
  private static double BARGE_X = 7.408452033996582 - Units.inchesToMeters(12);
  private static double MIN_BARGE_Y = 4.6;
  private static double MAX_BARGE_Y = 7.5;

  /**
   * Returns a command that aligns the robot to the barge region, clamping Y and zeroing rotation.
   *
   * @return Command for slow, precise barge alignment.
   */
  public Command autoAlignBarge() {
    return Commands.defer(() -> {
      Pose2d closestBargePose = new Pose2d(BARGE_X, MathUtil.clamp(swerveDrive.getEstimatedPose().getY(), MIN_BARGE_Y, MAX_BARGE_Y), Rotation2d.fromDegrees(0));

      return swerveDrive.driveTo(closestBargePose).until(() -> swerveDrive.isWithinToleranceOf(closestBargePose, Inches.of(4), Degrees.of(6)));
    }, Set.of(swerveDrive.useMotion()));
  }

  /**
   * Provides a two-stage command for setting up and then aligning with the barge, including rotation clamping.
   *
   * @return Command for barge setup and final alignment.
   */
  public Command autoAlignSetupBarge() {
    return Commands.defer(() -> {
      // Stage 1: Clamp rotation difference to +-60 deg, move to pose.
      Angle currentRotation = swerveDrive.getEstimatedPose().getRotation().getMeasure();
      Angle rotationDifference = MeasureMath.minDifference(Degrees.of(0), currentRotation);
      Angle clampedDifference = MeasureMath.clamp(rotationDifference, Degrees.of(-60), Degrees.of(60));
      Rotation2d targetRotation = new Rotation2d(currentRotation.plus(clampedDifference));

      Pose2d closestBargePose = new Pose2d(BARGE_X - 0.5, MathUtil.clamp(swerveDrive.getEstimatedPose().getY(), MIN_BARGE_Y, MAX_BARGE_Y), targetRotation);

      return swerveDrive.driveQuicklyTo(closestBargePose).until(() -> swerveDrive.isWithinToleranceOf(closestBargePose, Inches.of(4), Degrees.of(6)));
    }, Set.of(swerveDrive.useMotion()))
      .andThen(
        // Stage 2: Zero rotation
        Commands.defer(() -> {
          Pose2d closestBargePose = new Pose2d(BARGE_X - 0.5, MathUtil.clamp(swerveDrive.getEstimatedPose().getY(), MIN_BARGE_Y, MAX_BARGE_Y), Rotation2d.fromDegrees(0));
    
          return swerveDrive.driveQuicklyTo(closestBargePose).until(() -> swerveDrive.isWithinToleranceOf(closestBargePose, Inches.of(4), Degrees.of(6)));
        }, Set.of(swerveDrive.useMotion()))
      );
  }

  /**
   * Returns a fast barge alignment command with looser tolerance, useful for quick auto routines.
   *
   * @return Command for fast barge alignment.
   */
  public Command autoAlignBargeFast() {
    return Commands.defer(() -> {
      Pose2d closestBargePose = new Pose2d(BARGE_X, MathUtil.clamp(swerveDrive.getEstimatedPose().getY(), MIN_BARGE_Y, MAX_BARGE_Y), Rotation2d.fromDegrees(0));

      return swerveDrive.driveQuicklyTo(closestBargePose).until(() -> swerveDrive.isWithinToleranceOf(closestBargePose, Inches.of(8), Degrees.of(10)));
    }, Set.of(swerveDrive.useMotion()));
  }
}
