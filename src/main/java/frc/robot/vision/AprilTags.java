package frc.robot.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.auto.PoseEstimator;
import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.LIMELIGHT;
import frc.robot.field.Field;
import frc.robot.util.CachedRobotState;
import io.limelightvision.LimelightHelpers;
import io.limelightvision.LimelightHelpers.PoseEstimate;
import io.limelightvision.LimelightHelpers.RawFiducial;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class AprilTags extends SubsystemBase {
  public static boolean changingHeading = false;
  private static final double MAX_ROTATION_ERROR = Units.degreesToRadians(15);
  private static final double LARGE_ROTATION_ERROR = 9999999;
  private static final double TRANSLATION_ERROR_FACTOR = 10;
  private static final double ADDITIONAL_TRANSLATION_ERROR = 0.5;
  private static Map<String, LimelightHelpers.PoseEstimate> loggedEstimates = Map.of();

  static {
    Logger.addUpdate(
        "/PoseEstimates/",
        () -> {
          for (String id : LIMELIGHT.APRILTAG_CAMERA_POSES.keySet()) {
            if (loggedEstimates.get(id) != null) {
              PoseEstimate estimate = loggedEstimates.get(id);

              Logger.log("/PoseEstimates/" + id + "/pose", estimate.pose);
              Logger.log("/PoseEstimates/" + id + "/timestamp", estimate.timestampSeconds);
              Logger.log("/PoseEstimates/" + id + "/avgTagArea", estimate.avgTagArea);
              Logger.log("/PoseEstimates/" + id + "/avgTagDist", estimate.avgTagDist);
              Logger.log("/PoseEstimates/" + id + "/isMegaTag2", estimate.isMegaTag2);
              Logger.log("/PoseEstimates/" + id + "/tagSpan", estimate.tagSpan);
              Logger.log("/PoseEstimates/" + id + "/latency", estimate.latency);
            } else {
              Logger.log("/PoseEstimates/" + id + "/pose", new Pose2d());
              Logger.log("/PoseEstimates/" + id + "/timestamp", 0);
              Logger.log("/PoseEstimates/" + id + "/avgTagArea", 0);
              Logger.log("/PoseEstimates/" + id + "/avgTagDist", 0);
              Logger.log("/PoseEstimates/" + id + "/isMegaTag2", false);
              Logger.log("/PoseEstimates/" + id + "/tagSpan", 0);
              Logger.log("/PoseEstimates/" + id + "/latency", 0);
            }
          }
        });
  }

  private static record BestEstimate(
      Pose2d pose, Time timestamp, Distance translationError, Angle rotationError, int tagCount) {
    public BestEstimate() {
      this(
          new Pose2d(),
          Seconds.of(Timer.getFPGATimestamp()),
          Meters.of(Double.POSITIVE_INFINITY),
          Rotations.of(Double.POSITIVE_INFINITY),
          0);
    }
  }

  public static void injectVisionData(
      Map<String, Pose3d> cameraPoses, PoseEstimator poseEstimator) {
    Map<String, LimelightHelpers.PoseEstimate> poseEstimates =
        getIdentifiedPoseEstimates(cameraPoses.keySet());
    changingHeading = false;

    loggedEstimates = poseEstimates;

    BestEstimate bestPoseEstimate = new BestEstimate();
    List<Pose2d> loggedVisionPoses = new ArrayList<>();

    for (PoseEstimate poseEstimate : poseEstimates.values()) {
      Pose2d pose2d = poseEstimate.pose;
      if (isInvalidPoseEstimate(poseEstimate, pose2d)) continue;

      boolean canChangeHeading = canChangeHeading(poseEstimate, poseEstimator, pose2d);

      double rotationError = canChangeHeading ? MAX_ROTATION_ERROR : LARGE_ROTATION_ERROR;
      if (!canChangeHeading) {
        pose2d = adjustPoseRotation(poseEstimator, poseEstimate, pose2d);
      }

      if (canChangeHeading) {
        changingHeading = true;
        LimelightHelpers.setLEDMode_ForceBlink("limelight-ftag");
        LimelightHelpers.setLEDMode_ForceBlink("limelight-btag");
      } else {
        LimelightHelpers.setLEDMode_ForceOff("limelight-ftag");
        LimelightHelpers.setLEDMode_ForceOff("limelight-btag");
      }

      double translationError = calculateTranslationError(poseEstimate);

      loggedVisionPoses.add(pose2d);
      translationError += ADDITIONAL_TRANSLATION_ERROR;

      if (isBetterEstimate(bestPoseEstimate, translationError, rotationError)) {
        bestPoseEstimate =
            new BestEstimate(
                pose2d,
                Seconds.of(poseEstimate.timestampSeconds),
                Meters.of(translationError),
                Rotations.of(rotationError),
                poseEstimate.tagCount);
      }
    }

    if (bestPoseEstimate.tagCount > 0) {
      poseEstimator.addVisionMeasurement(
          bestPoseEstimate.pose,
          bestPoseEstimate.timestamp,
          VecBuilder.fill(
              bestPoseEstimate.translationError.in(Meters),
              bestPoseEstimate.translationError.in(Meters),
              bestPoseEstimate.rotationError.in(Radians)));
    }

    Logger.getField().getObject("visionPosese").setPoses(loggedVisionPoses);
  }

  private static List<LimelightHelpers.PoseEstimate> getPoseEstimates(
      Map<String, Pose3d> cameraPoses) {
    return cameraPoses.keySet().stream()
        .map(
            name ->
                CachedRobotState.isAllianceInverted().orElse(false)
                    ? LimelightHelpers.getBotPoseEstimate_wpiRed(name)
                    : LimelightHelpers.getBotPoseEstimate_wpiBlue(name))
        .filter((estimate) -> estimate != null)
        .collect(Collectors.toList());
  }

  private static Map<String, LimelightHelpers.PoseEstimate> getIdentifiedPoseEstimates(
      Set<String> cameraIds) {
    HashMap<String, LimelightHelpers.PoseEstimate> output = new HashMap<>();

    for (String cameraId : cameraIds) {
      LimelightHelpers.PoseEstimate estimate =
          CachedRobotState.isAllianceInverted().orElse(false)
              ? LimelightHelpers.getBotPoseEstimate_wpiRed(cameraId)
              : LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraId);

      if (estimate == null) continue;

      output.put(cameraId, estimate);
    }

    return output;
  }

  private static boolean isInvalidPoseEstimate(PoseEstimate poseEstimate, Pose2d pose2d) {
    return IntStream.of(LIMELIGHT.BLACKLISTED_APRILTAGS)
            .anyMatch(x -> x == poseEstimate.rawFiducials[0].id)
        || poseEstimate.tagCount == 0
        || pose2d.getTranslation().getNorm() == 0.0
        || pose2d.getRotation().getRadians() == 0.0
        || Double.isNaN(poseEstimate.avgTagDist)
        || pose2d.getX() < 0.0
        || pose2d.getY() < 0.0
        || pose2d.getX() > Field.LENGTH
        || pose2d.getY() > Field.WIDTH
        || isHighAmbiguity(poseEstimate);
  }

  private static boolean isHighAmbiguity(PoseEstimate poseEstimate) {
    for (RawFiducial fiducial : poseEstimate.rawFiducials) {
      if (fiducial.ambiguity < 0.2) {
        return false;
      }
    }

    return true;
  }

  private static boolean canChangeHeading(
      PoseEstimate poseEstimate, PoseEstimator poseEstimator, Pose2d pose2d) {
    boolean canChangeHeading = poseEstimate.tagCount >= 1 || CachedRobotState.isDisabled();
    return canChangeHeading
        && poseEstimator.getEstimatedPose().getTranslation().getDistance(pose2d.getTranslation())
            < 1.0;
  }

  private static Pose2d adjustPoseRotation(
      PoseEstimator poseEstimator, PoseEstimate poseEstimate, Pose2d pose2d) {
    return new Pose2d(
        pose2d.getTranslation(),
        poseEstimator.getEstimatedPose(Seconds.of(poseEstimate.timestampSeconds)).getRotation());
  }

  private static double calculateTranslationError(PoseEstimate poseEstimate) {
    return Math.pow(Math.abs(poseEstimate.avgTagDist), 2.0)
        / Math.pow(poseEstimate.tagCount, 2)
        / TRANSLATION_ERROR_FACTOR;
  }

  private static boolean isBetterEstimate(
      BestEstimate bestPoseEstimate, double translationError, double rotationError) {
    return translationError < bestPoseEstimate.translationError.in(Meters);
  }

  public static void printConfig(Map<String, Pose3d> cameraPoses) {
    System.out.println(
        """


//////////////////////////////////////
///// LIMELIGHT POSITION CONFIG //////
//////////////////////////////////////
""");
    for (Map.Entry<String, Pose3d> limelight : cameraPoses.entrySet()) {
      System.out.println(
          String.format(
              """
          ----- %s.local:5801 -----
            LL Forward: %.5f
            LL Right:   %.5f
            LL Up:      %.5f
            LL Roll:    %.5f
            LL Pitch:   %.5f
            LL Yaw:     %.5f
          """,
              limelight.getKey(),
              limelight.getValue().getTranslation().getX(),
              limelight.getValue().getTranslation().getY(),
              limelight.getValue().getTranslation().getZ(),
              Units.radiansToDegrees(limelight.getValue().getRotation().getX()),
              Units.radiansToDegrees(limelight.getValue().getRotation().getY()),
              Units.radiansToDegrees(limelight.getValue().getRotation().getZ())));
    }
  }

  public static int findClosestReefTagID() {

    int ftagID =
        (int)
            LimelightHelpers.getFiducialID(
                LIMELIGHT.APRILTAG_CAMERA_POSES.keySet().toArray()[0].toString());
    int btagID =
        (int)
            LimelightHelpers.getFiducialID(
                LIMELIGHT.APRILTAG_CAMERA_POSES.keySet().toArray()[1].toString());

    if (Field.getReefAprilTagsByFace().contains(ftagID)) return ftagID;
    if (Field.getReefAprilTagsByFace().contains(btagID)) return btagID;
    return -1;
  }
}
