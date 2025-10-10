/**
 * ReefPositioning.java
 *
 * Provides geometric and algorithmic utilities for field-relative positioning around the "reef"—
 * the central scoring and pickup structure on the field. Supports conversions between relative and field poses,
 * pole/face calculations, coral and algae placements, and mirroring/reflection logic for autonomous and teleop routines.
 * Includes the CoralPosition class for describing a coral's location.
 */

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

/**
 * Utility class for reef-centric field positioning, pole/face math, and autonomous target calculations.
 */
public final class ReefPositioning {
  private ReefPositioning() {}

  /** The center of the reef (field origin for reef operations). */
  public static final Translation2d REEF_CENTER =
      new Translation2d(Units.inchesToMeters(176.745545), Units.inchesToMeters(158.499093));

  /** Distance from field edge to reef center. */
  public static final Distance REEF_TO_EDGE = Inches.of(32.745545);

  /** Distance between adjacent poles. */
  public static final Distance BETWEEN_POLES = Inches.of(12.937756);

  /** Robot offset from edge when placing coral. */
  public static final Distance ROBOT_TO_EDGE_PLACE_CORAL =
      SWERVE.CONFIG.chassis().outerLength().div(2).plus(Inches.of(2));

  /** Robot offset from edge for coral alignment. */
  public static final Distance ROBOT_TO_EDGE_ALIGN_CORAL =
      ROBOT_TO_EDGE_PLACE_CORAL.plus(Inches.of(6));

  /** Robot offset from edge for algae pickup. */
  public static final Distance ROBOT_TO_EDGE_PICKUP_ALGAE =
      SWERVE.CONFIG.chassis().outerLength().div(2);

  /** Robot offset from edge for algae alignment. */
  public static final Distance ROBOT_TO_EDGE_ALIGN_ALGAE =
      ROBOT_TO_EDGE_PLACE_CORAL.plus(Inches.of(12));

  /** Relative translation for coral placement. */
  public static final Translation2d PLACE_CORAL_RELATIVE =
      new Translation2d(
          REEF_TO_EDGE.plus(ROBOT_TO_EDGE_PLACE_CORAL).in(Meters), BETWEEN_POLES.div(2).in(Meters));

  /** Relative translation for coral alignment. */
  public static final Translation2d ALIGN_CORAL_RELATIVE =
      new Translation2d(
          REEF_TO_EDGE.plus(ROBOT_TO_EDGE_ALIGN_CORAL).in(Meters), BETWEEN_POLES.div(2).in(Meters));

  /** Relative translation for algae pickup. */
  public static final Translation2d PICKUP_ALGAE_RELATIVE =
      new Translation2d(REEF_TO_EDGE.plus(ROBOT_TO_EDGE_PICKUP_ALGAE).in(Meters), 0);

  /** Relative translation for algae alignment. */
  public static final Translation2d ALIGN_ALGAE_RELATIVE =
      new Translation2d(REEF_TO_EDGE.plus(ROBOT_TO_EDGE_ALIGN_ALGAE).in(Meters), 0);

  /** Robot offset from edge when leaving after an algae pickup. */
  public static final Distance ROBOT_TO_EDGE_LEAVE_ALGAE =
      ROBOT_TO_EDGE_PLACE_CORAL.plus(Inches.of(17));

  /** Relative translation for leaving after algae pickup. */
  public static final Translation2d LEAVE_ALGAE_RELATIVE =
      new Translation2d(REEF_TO_EDGE.plus(ROBOT_TO_EDGE_LEAVE_ALGAE).in(Meters), 0);

  /**
   * Converts a pose relative to the reef to a field pose with a given reef rotation.
   * @param relativePose The pose relative to the reef origin
   * @param rotation     The rotation of the reef face (field orientation)
   * @return The field-relative pose
   */
  private static Pose2d relativeToFieldPose(Pose2d relativePose, Rotation2d rotation) {
    Translation2d reefTranslation = relativePose.getTranslation().rotateBy(rotation);
    Translation2d fieldTranslation = REEF_CENTER.plus(reefTranslation);

    return new Pose2d(fieldTranslation, rotation);
  }

  /**
   * Reflects a pose across the reef center line (y-axis).
   * @param pose Pose to reflect
   * @return Reflected pose
   */
  private static Pose2d reflectPose(Pose2d pose) {
    return new Pose2d(new Translation2d(pose.getX(), -pose.getY()), pose.getRotation());
  }

  /**
   * Generates a field pose from a relative translation, reef face rotation, and optional mirroring/unreflected offset.
   * @param relativeTranslation Relative translation from reef center
   * @param rotation            Rotation of the reef face
   * @param unreflectedOffset   Offset to apply before mirroring
   * @param reverse             Whether to mirror (reflect) across the center
   * @return The field pose
   */
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

  /**
   * Returns the rotation for a reef face index (60 deg per face, 6 faces).
   * @param face Face index [0-5]
   * @return Rotation2d for the face
   */
  private static Rotation2d getRotationOfFace(int face) {
    return Rotation2d.fromDegrees(60. * face);
  }

  // Poles are CCW starting at right of simulation view (WPILib convention)
  /**
   * Returns the rotation for a pole index.
   * @param pole Pole index [0-11]
   * @return Rotation2d for the pole
   */
  private static Rotation2d getRotationOfPole(int pole) {
    return getRotationOfFace((int) Math.round(pole / 2.));
  }

  /**
   * Returns whether a given pole is mirrored (odd indices).
   * @param pole Pole index
   * @return true if mirrored
   */
  private static boolean getMirroringOfPole(int pole) {
    return pole % 2 == 1;
  }

  /**
   * Returns the field pose of a pole given its relative translation and offset.
   * @param relativeTranslation The translation relative to the reef center
   * @param unreflectedOffset   Offset applied before mirroring
   * @param pole                The pole index [0-11]
   * @return The field pose
   */
  public static Pose2d getPolePose(
      Translation2d relativeTranslation, Translation2d unreflectedOffset, int pole) {
    return relativeToFieldAndReflect(
        relativeTranslation, getRotationOfPole(pole), unreflectedOffset, getMirroringOfPole(pole));
  }

  /**
   * Returns the field pose of a face given a relative translation and face index.
   * @param relativeTranslation The translation relative to the reef center
   * @param face                The face index [0-5]
   * @return The field pose
   */
  public static Pose2d getFacePose(Translation2d relativeTranslation, int face) {
    return relativeToFieldPose(
        new Pose2d(relativeTranslation, new Rotation2d()), getRotationOfFace(face));
  }

  /**
   * Gets the pose for placing coral on a given pole.
   * @param pole The pole index [0-11]
   * @return The target field pose for placing coral
   */
  public static Pose2d getCoralPlacePose(int pole) {
    return rotatePose(
        getPolePose(PLACE_CORAL_RELATIVE, new Translation2d(0, 0), pole),
        Rotation2d.fromDegrees(180)); // 180
  }

  /**
   * Gets the alignment pose for a given coral pole.
   * @param pole The pole index [0-11]
   * @return The field pose for aligning to place coral
   */
  public static Pose2d getCoralAlignPose(int pole) {
    return rotatePose(
        getPolePose(ALIGN_CORAL_RELATIVE, new Translation2d(0, 0), pole),
        Rotation2d.fromDegrees(180)); // 180
  }

  /**
   * Gets the pose for placing coral at Level 1 on a given pole.
   * @param pole The pole index [0-11]
   * @return The field pose for L1 placement
   */
  public static Pose2d getL1PlacePose(int pole) {
    return rotatePose(
      getPolePose(new Translation2d(0.99, 0.97), new Translation2d(0, 0), pole),
      Rotation2d.fromDegrees(180 + (pole % 2 == 0 ? 68.62 : -68.62))
    );
  }

  /**
   * Gets the algae pickup pose for a given reef face.
   * @param face The face index [0-5]
   * @return The field pose for algae pickup
   */
  public static Pose2d getAlgaePickupPose(int face) {
    return rotatePose(getFacePose(PICKUP_ALGAE_RELATIVE, face), Rotation2d.fromDegrees(180));
  }

  /**
   * Gets the alignment pose for picking up algae on a given face.
   * @param face The face index [0-5]
   * @return The field pose for aligning to algae pickup
   */
  public static Pose2d getAlgaeAlignPose(int face) {
    return rotatePose(getFacePose(ALIGN_ALGAE_RELATIVE, face), Rotation2d.fromDegrees(180));
  }

  /**
   * Gets the pose for leaving the algae pickup zone on a given face.
   * @param face The face index [0-5]
   * @return The field pose for leaving algae
   */
  public static Pose2d getAlgaeLeavePose(int face) {
    return getFacePose(LEAVE_ALGAE_RELATIVE, face);
  }

  /**
   * Rotates a pose by a given rotation about its origin.
   * @param pose The original pose
   * @param rotation The rotation to apply
   * @return The rotated pose
   */
  private static Pose2d rotatePose(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), pose.getRotation().plus(rotation));
  }

  /**
   * Returns the scoring level for algae on a face (2 or 3).
   * @param face The face index [0-5]
   * @return Level (2 or 3)
   */
  public static int getAlgaeHeight(int face) {
    return face % 2 + 2;
  }

  /**
   * Data class representing a coral position (pole and level).
   */
  public static class CoralPosition {
    /** Coral pole ranging from 0-11. */
    public final int pole;

    /** Coral level ranging from 1-4. */
    public final int level;

    /**
     * Constructs a coral position, wrapping pole and clamping level as needed.
     * @param pole 0-11 (wraps values outside this range)
     * @param level 1-4 (clamps values outside this range)
     */
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

    /**
     * Returns a reflected coral position if shouldReflect is true (mirrors across field).
     * @param shouldReflect Whether to reflect (mirror)
     * @return Reflected or original CoralPosition
     */
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
