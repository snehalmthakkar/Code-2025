package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

import com.team6962.lib.swerve.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.field.ReefPositioning;

public class AutoAlign {
  private SwerveDrive swerveDrive;

  public AutoAlign(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

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

  private int getClosestReefPole(Pose2d  pose, Function<Integer, Pose2d> poseSupplier, PolePattern pattern) {
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

  public Command alignToClosestFaceTeleop() {
    return Commands.defer(
        () -> alignFace(getClosestReefFace(swerveDrive.getEstimatedPose()), false),
        Set.of(swerveDrive.useMotion()));
  }

  private static double BARGE_X = 7.408452033996582;
  private static double MIN_BARGE_Y = 4.6;
  private static double MAX_BARGE_Y = 7.5;

  public Command autoAlignBarge() {
    return Commands.defer(() -> {
      Pose2d closestBargePose = new Pose2d(BARGE_X, MathUtil.clamp(swerveDrive.getEstimatedPose().getY(), MIN_BARGE_Y, MAX_BARGE_Y), Rotation2d.fromDegrees(0));

      return swerveDrive.driveTo(closestBargePose).until(() -> swerveDrive.isWithinToleranceOf(closestBargePose, Inches.of(4), Degrees.of(6)));
    }, Set.of(swerveDrive.useMotion()));
  }
}
