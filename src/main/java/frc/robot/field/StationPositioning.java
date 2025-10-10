/**
 * StationPositioning.java
 *
 * Provides static utility methods and constants for calculating field-relative positions
 * and poses for coral stations, intake slots, and alignment points on the field.
 * Supports both left/right station reflection, ground intake positions, and slot-based targeting.
 * Used for autonomous and teleoperated routines that require precise robot positioning
 * around the field's coral stations and ground intake locations.
 */

package frc.robot.field;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.Constants.SWERVE;

/**
 * Utility class for field geometry and pose calculations related to the coral stations,
 * including slot positions, intake points, and ground intake poses.
 */
public final class StationPositioning {
  private StationPositioning() {} // Prevent instantiation

  /**
   * Represents the two coral stations on the field (LEFT and RIGHT), each with a calculated center pose.
   */
  public static enum CoralStation {
    LEFT(true),
    RIGHT(false);

    /** The center pose for this station. */
    public final Pose2d pose;

    /**
     * Create a CoralStation with reflection set.
     * @param reflect If true, station is on the left; otherwise, right.
     */
    CoralStation(boolean reflect) {
      this.pose = getCenterIntakePose(reflect);
    }

    @Override
    public String toString() {
      return this == LEFT ? "Left" : "Right";
    }
  }

  // Field geometry constants
  private static final Rotation2d STATION_FRONT_ANGLE = Rotation2d.fromDegrees(144.011392);
  private static final Distance STATION_OFFSET_X = Inches.of(65.828076);
  private static final Translation2d STATION_EDGE =
      new Translation2d(STATION_OFFSET_X.in(Meters), 0);
  private static final Distance FIRST_SLOT_OFFSET = Inches.of(7.8775);
  private static final Distance OFFSET_BETWEEN_SLOTS = Inches.of(8);
  private static final Distance INTAKE_DISTANCE_TO_EDGE = Inches.of(2);
  private static final Distance ALIGN_DISTANCE_TO_EDGE = Inches.of(16);
  public static final int SLOT_COUNT = 9;

  /**
   * Gets the translation for a given slot index along the station edge.
   * @param rsSlotIndex The right-side slot index [0-SLOT_COUNT-1]
   * @return Translation2d of slot center
   */
  public static Translation2d getSlotTranslation(int rsSlotIndex) {
    return STATION_EDGE.plus(
        new Translation2d(
                FIRST_SLOT_OFFSET.plus(OFFSET_BETWEEN_SLOTS.times(rsSlotIndex)).in(Meters), 0)
            .rotateBy(STATION_FRONT_ANGLE));
  }

  /**
   * Calculates the pose for a given slot translation and distance from the station edge.
   * @param rsSlotTranslation The slot translation (use getSlotTranslation)
   * @param distanceToEdge The distance to move away from station edge
   * @return Pose2d for intake
   */
  public static Pose2d getIntakePose(Translation2d rsSlotTranslation, Distance distanceToEdge) {
    Pose2d relativePosition =
        new Pose2d(
            new Translation2d(
                SWERVE.CONFIG.chassis().outerLength().div(2).plus(distanceToEdge).in(Meters), 0),
            Rotation2d.fromDegrees(0));

    relativePosition =
        relativePosition.rotateBy(STATION_FRONT_ANGLE.minus(Rotation2d.fromDegrees(90)));
    relativePosition =
        new Pose2d(
            relativePosition.getTranslation().plus(rsSlotTranslation),
            relativePosition.getRotation());

    return relativePosition;
  }

  /**
   * Calculates the pose for a given slot and station reflection.
   * @param reflect True for left station, false for right
   * @param slotIndex The slot index [0-SLOT_COUNT-1]
   * @param distanceToEdge The distance to move away from the edge
   * @return Pose2d for intake
   */
  public static Pose2d getIntakePose(boolean reflect, int slotIndex, Distance distanceToEdge) {
    Pose2d rsPose = getIntakePose(getSlotTranslation(slotIndex), distanceToEdge);

    if (reflect) {
      rsPose =
          new Pose2d(rsPose.getX(), Field.WIDTH - rsPose.getY(), rsPose.getRotation().unaryMinus());
    }

    return rsPose;
  }

  /**
   * Gets the intake pose for a specific coral station and slot.
   * @param coralStation The station (LEFT/RIGHT)
   * @param slotIndex The slot index
   * @return Pose2d for intake
   */
  public static Pose2d getIntakePose(CoralStation coralStation, int slotIndex) {
    return getIntakePose(coralStation == CoralStation.LEFT, slotIndex, INTAKE_DISTANCE_TO_EDGE);
  }

  /**
   * Gets the intake pose nearest to the robot's current pose.
   * (Currently always returns slot 4.)
   * @param coralStation The station
   * @param currentPose The current robot pose
   * @return Nearest intake pose
   */
  public static Pose2d getNearestIntakePose(CoralStation coralStation, Pose2d currentPose) {
    return getIntakePose(coralStation, 4);
  }

  /**
   * Gets the center intake pose for the station.
   * @param reflect True for left, false for right
   * @return Center intake Pose2d
   */
  public static Pose2d getCenterIntakePose(boolean reflect) {
    return getIntakePose(reflect, 4, INTAKE_DISTANCE_TO_EDGE);
  }

  /**
   * Gets the center intake pose for a given station.
   * @param coralStation The station
   * @return Center intake Pose2d
   */
  public static Pose2d getCenterIntakePose(CoralStation coralStation) {
    return getCenterIntakePose(coralStation == CoralStation.LEFT);
  }

  /**
   * Gets the align pose (further from edge) for a station and slot.
   * @param coralStation The station
   * @param slotIndex The slot index
   * @return Align Pose2d
   */
  public static Pose2d getAlignPose(CoralStation coralStation, int slotIndex) {
    return getIntakePose(coralStation == CoralStation.LEFT, slotIndex, ALIGN_DISTANCE_TO_EDGE);
  }

  /**
   * Gets the align pose nearest to the robot's current pose.
   * (Currently always returns slot 4.)
   * @param coralStation The station
   * @param currentPose The robot pose
   * @return Align Pose2d
   */
  public static Pose2d getNearestAlignPose(CoralStation coralStation, Pose2d currentPose) {
    return getAlignPose(coralStation, 4);
  }

  /**
   * Gets the center align pose for the station.
   * @param reflect True for left, false for right
   * @return Center align Pose2d
   */
  public static Pose2d getCenterAlignPose(boolean reflect) {
    return getIntakePose(reflect, 4, ALIGN_DISTANCE_TO_EDGE);
  }

  /**
   * Gets the center align pose for a given station.
   * @param coralStation The station
   * @return Center align Pose2d
   */
  public static Pose2d getCenterAlignPose(CoralStation coralStation) {
    return getCenterAlignPose(coralStation == CoralStation.LEFT);
  }

  /**
   * Reflects a pose across the field width if reflect is true.
   * @param pose The original pose
   * @param reflect Whether to reflect (true for left)
   * @return Reflected or original pose
   */
  public static Pose2d reflectPose(Pose2d pose, boolean reflect) {
    return reflect ? new Pose2d(pose.getX(), Field.WIDTH - pose.getY(), pose.getRotation().unaryMinus()) : pose;
  }

  /**
   * Gets the pose for ground intake, adjusted for station side.
   * @param reflect True for left, false for right
   * @return Ground intake Pose2d
   */
  public static Pose2d getGroundIntakePose(boolean reflect) {
    Pose2d pose = new Pose2d(2, 1.514, Rotation2d.fromDegrees(46.83));
    return reflectPose(pose, reflect);
  }

  /**
   * Gets the pose for ground intake for a given station.
   * @param coralStation The station
   * @return Ground intake Pose2d
   */
  public static Pose2d getGroundIntakePose(CoralStation coralStation) {
    return getGroundIntakePose(coralStation == CoralStation.LEFT);
  }

  /**
   * Gets the start pose for a ground intake sequence for a station.
   * @param coralStation The station
   * @return Start pose for ground intake
   */
  public static Pose2d getGroundStartIntakePose(CoralStation coralStation) {
    return reflectPose(new Pose2d(2.78, 0.7, Rotation2d.fromDegrees(0)), coralStation == CoralStation.LEFT);
  }

  /**
   * Gets the end pose for a ground intake sequence for a station.
   * @param coralStation The station
   * @return End pose for ground intake
   */
  public static Pose2d getGroundEndIntakePose(CoralStation coralStation) {
    return reflectPose(new Pose2d(1.37, 1.44, Rotation2d.fromDegrees(0)), coralStation == CoralStation.LEFT);
  }

  /**
   * Returns a pose rotated 180 degrees from the given pose.
   * @param pose The original pose
   * @return Pose2d with orientation rotated by 180 degrees
   */
  public static Pose2d rotate180(Pose2d pose) {
    return new Pose2d(pose.getTranslation(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
  }
}
