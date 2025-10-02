package frc.robot.field;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.Constants.SWERVE;

public final class ReefPositioning {
  private ReefPositioning() {}

  public static final Translation2d REEF_CENTER =
      new Translation2d(Units.inchesToMeters(176.745545), Units.inchesToMeters(158.499093));

  public static final Distance REEF_TO_EDGE = Inches.of(32.745545);
  // public static final Distance REEF_TO_POLE = Inches.of(30.738196);
  public static final Distance BETWEEN_POLES = Inches.of(12.937756);
  public static final Distance ROBOT_TO_EDGE_PLACE_CORAL =
      SWERVE.CONFIG.chassis().outerLength().div(2).plus(Inches.of(2));
  public static final Distance ROBOT_TO_EDGE_ALIGN_CORAL =
      ROBOT_TO_EDGE_PLACE_CORAL.plus(Inches.of(6));

  public static final Distance ROBOT_TO_EDGE_PICKUP_ALGAE =
      SWERVE.CONFIG.chassis().outerLength().div(2);
  public static final Distance ROBOT_TO_EDGE_ALIGN_ALGAE =
      ROBOT_TO_EDGE_PLACE_CORAL.plus(Inches.of(12));

  public static final Translation2d PLACE_CORAL_RELATIVE =
      new Translation2d(
          REEF_TO_EDGE.plus(ROBOT_TO_EDGE_PLACE_CORAL).in(Meters), BETWEEN_POLES.div(2).in(Meters));

  public static final Translation2d ALIGN_CORAL_RELATIVE =
      new Translation2d(
          REEF_TO_EDGE.plus(ROBOT_TO_EDGE_ALIGN_CORAL).in(Meters), BETWEEN_POLES.div(2).in(Meters));

  public static final Translation2d PICKUP_ALGAE_RELATIVE =
      new Translation2d(REEF_TO_EDGE.plus(ROBOT_TO_EDGE_PICKUP_ALGAE).in(Meters), 0);

  public static final Translation2d ALIGN_ALGAE_RELATIVE =
      new Translation2d(REEF_TO_EDGE.plus(ROBOT_TO_EDGE_ALIGN_ALGAE).in(Meters), 0);

  public static final Distance ROBOT_TO_EDGE_LEAVE_ALGAE =
      ROBOT_TO_EDGE_PLACE_CORAL.plus(Inches.of(17));

  public static final Translation2d LEAVE_ALGAE_RELATIVE =
      new Translation2d(REEF_TO_EDGE.plus(ROBOT_TO_EDGE_LEAVE_ALGAE).in(Meters), 0);

  private static Pose2d relativeToFieldPose(Pose2d relativePose, Rotation2d rotation) {
    Translation2d reefTranslation = relativePose.getTranslation().rotateBy(rotation);
    Translation2d fieldTranslation = REEF_CENTER.plus(reefTranslation);

    return new Pose2d(fieldTranslation, rotation);
  }

  private static Pose2d reflectPose(Pose2d pose) {
    return new Pose2d(new Translation2d(pose.getX(), -pose.getY()), pose.getRotation());
  }

  private static Pose2d relativeToFieldAndReflect(
      Translation2d relativeTranslation,
      Rotation2d rotation,
      Translation2d unreflectedOffset,
      boolean reverse) {
    Pose2d relativePose = new Pose2d(relativeTranslation, new Rotation2d());

    if (reverse) {
      relativePose = reflectPose(relativePose);
    }

    relativePose = relativePose.plus(new Transform2d(unreflectedOffset, new Rotation2d()));

    return relativeToFieldPose(relativePose, rotation);
  }

  private static Rotation2d getRotationOfFace(int face) {
    return Rotation2d.fromDegrees(60. * face);
  }

  // Poles are CCW starting at right of simulation view (WPILib convention)
  private static Rotation2d getRotationOfPole(int pole) {
    return getRotationOfFace((int) Math.round(pole / 2.));
  }

  private static boolean getMirroringOfPole(int pole) {
    return pole % 2 == 1;
  }

  public static Pose2d getPolePose(
      Translation2d relativeTranslation, Translation2d unreflectedOffset, int pole) {
    return relativeToFieldAndReflect(
        relativeTranslation, getRotationOfPole(pole), unreflectedOffset, getMirroringOfPole(pole));
  }

  public static Pose2d getFacePose(Translation2d relativeTranslation, int face) {
    return relativeToFieldPose(
        new Pose2d(relativeTranslation, new Rotation2d()), getRotationOfFace(face));
  }

  public static Pose2d getCoralPlacePose(int pole) {
    return rotatePose(
        getPolePose(PLACE_CORAL_RELATIVE, new Translation2d(0, 0), pole),
        Rotation2d.fromDegrees(180)); // 180
  }

  public static Pose2d getCoralAlignPose(int pole) {
    return rotatePose(
        getPolePose(ALIGN_CORAL_RELATIVE, new Translation2d(0, 0), pole),
        Rotation2d.fromDegrees(180)); // 180
  }

  public static Pose2d getL1PlacePose(int pole) {
    return rotatePose(
      getPolePose(new Translation2d(0.99, 0.97), new Translation2d(0, 0), pole),
      Rotation2d.fromDegrees(180 + (pole % 2 == 0 ? 68.62 : -68.62))
    );
  }

  public static Pose2d getAlgaePickupPose(int face) {
    return rotatePose(getFacePose(PICKUP_ALGAE_RELATIVE, face), Rotation2d.fromDegrees(180));
  }

  public static Pose2d getAlgaeAlignPose(int face) {
    return rotatePose(getFacePose(ALIGN_ALGAE_RELATIVE, face), Rotation2d.fromDegrees(180));
  }

  public static Pose2d getAlgaeLeavePose(int face) {
    return getFacePose(LEAVE_ALGAE_RELATIVE, face);
  }

  private static Pose2d rotatePose(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), pose.getRotation().plus(rotation));
  }

  public static int getAlgaeHeight(int face) {
    return face % 2 + 2;
  }

  public static class CoralPosition {
    /** Coral pole ranging from 0-11. */
    public final int pole;

    /** Coral level ranging from 1-4. */
    public final int level;

    public CoralPosition(int pole, int level) {
      if (pole < 0 || pole > 11) {
        pole = (pole % 12 + 12) % 12;
      }

      if (level < 1 || level > 4) {
        System.out.println("Coral level is out of range");
        level = 2;
      }

      this.pole = pole;
      this.level = level;
    }

    public CoralPosition reflectedIf(boolean shouldReflect) {
      if (shouldReflect) {
        return new CoralPosition(11 - pole, level);
      } else {
        return this;
      }
    }

    @Override
    public boolean equals(Object obj) {
      return obj instanceof CoralPosition
          && ((CoralPosition) obj).pole == pole
          && ((CoralPosition) obj).level == level;
    }

    @Override
    public String toString() {
      return "P" + pole + "L" + level;
    }
  }
}
