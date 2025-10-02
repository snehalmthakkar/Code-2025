package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import java.util.concurrent.locks.ReentrantLock;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.TimestampedString;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.limelightvision.LimelightHelpers;
import io.limelightvision.LimelightHelpers.LimelightResults;
import io.limelightvision.LimelightHelpers.LimelightTarget_Classifier;

public class CoralDetection extends SubsystemBase {
    private static Time MAX_LATENCY = Seconds.of(0.5);
    private static Distance MAX_DISTANCE = Meters.of(2);

    private String cameraName;

    private Translation3d cameraPosition;
    private Angle cameraPitch;

    private Angle tx, ty;

    private SwerveDrive drivetrain;

    private Translation3d objectInView = new Translation3d();
    private Translation2d objectOnField = new Translation2d();

    private Translation2d lastCoralPosition = null;
    private Time lastCoralTimestamp = Seconds.of(-10000);

    private Time lastMeasurementTimestamp = Seconds.of(-10000);

    private final ReentrantLock loggingLock = new ReentrantLock();

    public CoralDetection(String cameraName, Translation3d cameraPosition, Angle cameraPitch, SwerveDrive drivetrain) {
        this.cameraName = cameraName;
        this.cameraPosition = cameraPosition;
        this.cameraPitch = cameraPitch;
        this.drivetrain = drivetrain;

        Logger.log("CoralDetection/" + cameraName + "/cameraPosition/x", cameraPosition.getX());
        Logger.log("CoralDetection/" + cameraName + "/cameraPosition/y", cameraPosition.getY());
        Logger.log("CoralDetection/" + cameraName + "/cameraPosition/z", cameraPosition.getZ());
    
        Logger.log("CoralDetection/" + cameraName + "/cameraPitch", cameraPitch.in(Degrees));

        Logger.logMeasure("CoralDetection/" + cameraName + "/tx", () -> tx);
        Logger.logMeasure("CoralDetection/" + cameraName + "/ty", () -> ty);

        Logger.logMeasure("CoralDetection/" + cameraName + "/detectionTimestamp", () -> lastCoralTimestamp);
        Logger.logNumber("CoralDetection/" + cameraName + "/coralPositionX", () -> getCoralLocation() == null ? 0 : getCoralLocation().getX());
        Logger.logNumber("CoralDetection/" + cameraName + "/coralPositionY", () -> getCoralLocation() == null ? 0 : getCoralLocation().getY());
    }

    private void processClassifierTarget(LimelightTarget_Classifier target, Time timestamp, Time latency) {
        if (target.className != "Coral") return;

        tx = Degrees.of(target.tx);
        ty = Degrees.of(target.ty);

        double focalLength = 1.0;

        double sensorX = Math.tan(tx.in(Radians)) * focalLength;
        double sensorY = Math.tan(ty.in(Radians)) * focalLength;

        double distanceForwardFromCameraToGround = -cameraPosition.getZ() / Math.sin(cameraPitch.in(Radians));

        double slopeOfGroundRelativeToCamera = -Math.tan(cameraPitch.in(Radians));

        double cameraY = distanceForwardFromCameraToGround * sensorY / (focalLength - sensorY * slopeOfGroundRelativeToCamera);
        double cameraZ = slopeOfGroundRelativeToCamera * cameraY + distanceForwardFromCameraToGround;
        double cameraX = sensorX * cameraZ / focalLength;

        objectInView = new Translation3d(cameraX, cameraY, cameraZ);
        Transform3d objectInViewPose = new Transform3d(objectInView, new Rotation3d());

        Rotation3d cameraRotation = new Rotation3d(Degrees.of(0), cameraPitch, Degrees.of(0));
        Transform3d cameraPose = new Transform3d(cameraPosition, cameraRotation);

        Pose3d robotPose = new Pose3d(drivetrain.getEstimatedPose(timestamp));

        Transform3d robotToObject = cameraPose.plus(objectInViewPose);
        Pose3d fieldToObject = robotPose.plus(robotToObject);
        Translation2d objectOnField = fieldToObject.getTranslation().toTranslation2d();

        Distance distance = Meters.of(objectOnField.getDistance(drivetrain.getEstimatedPose().getTranslation()));
        
        // double confidence = 1 - latency.div(MAX_LATENCY).minus(distance.div(MAX_DISTANCE)).in(Value);

        loggingLock.lock();
        
        try {
            if (timestamp.minus(lastCoralTimestamp).gte(Seconds.of(0.25)) || objectOnField.getDistance(drivetrain.getEstimatedPose().getTranslation()) < lastCoralPosition.getDistance(drivetrain.getEstimatedPose().getTranslation())) {
                lastCoralPosition = objectOnField;
                lastCoralTimestamp = timestamp;
            }
        } finally {
            loggingLock.unlock();
        }
    }

    public Translation2d getCoralLocation() {
        loggingLock.lock();

        try {
            return RobotController.getMeasureTime().minus(lastCoralTimestamp).lt(Seconds.of(1)) ? lastCoralPosition : null;
        } finally {
            loggingLock.unlock();
        }
    }

    @Override
    public void periodic() {
        LimelightTarget_Classifier target = new LimelightTarget_Classifier();
        target.tx = 0;
        target.ty = 0;
        target.className = "Coral";

        processClassifierTarget(target, RobotController.getMeasureTime(), Seconds.of(0));
        // TimestampedString timestampedTx = LimelightHelpers.getLimelightNTTable(cameraName).getStringTopic("json").getEntry("").getAtomic();
        // double internalLatencyMs = LimelightHelpers.getLatency_Pipeline(cameraName) + LimelightHelpers.getLatency_Capture(cameraName);
        // Time timestamp = Seconds.of((timestampedTx.timestamp / 1000000.0) - (internalLatencyMs / 1000.0));

        // if (timestamp.isNear(lastMeasurementTimestamp, Seconds.of(0.001))) return;
        
        // lastMeasurementTimestamp = timestamp;

        // Time latency = Seconds.of(RobotController.getFPGATime()).minus(timestamp);

        // if (latency.gt(MAX_LATENCY)) return;

        // LimelightResults results = LimelightHelpers.getLatestResults(cameraName);
        // LimelightTarget_Classifier[] classifierTargets = results.targets_Classifier;

        // if (classifierTargets != null) {
        //     for (LimelightTarget_Classifier target : classifierTargets) {
        //         processClassifierTarget(target, timestamp, latency);
        //     }
        // }
    }
}
