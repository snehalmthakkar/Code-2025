package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.limelightvision.LimelightHelpers;

public class CoralDetection extends SubsystemBase {
    private String cameraName;
    private Translation3d cameraPosition;
    private Angle cameraPitch;

    private Angle tx, ty;

    public CoralDetection(String cameraName, Translation3d cameraPosition, Angle cameraPitch) {
        this.cameraName = cameraName;
        this.cameraPosition = cameraPosition;
        this.cameraPitch = cameraPitch;

        Logger.log("CoralDetection/" + cameraName + "/cameraPosition/x", cameraPosition.getX());
        Logger.log("CoralDetection/" + cameraName + "/cameraPosition/y", cameraPosition.getY());
        Logger.log("CoralDetection/" + cameraName + "/cameraPosition/z", cameraPosition.getZ());
        
        Logger.log("CoralDetection/" + cameraName + "/cameraPitch", cameraPitch.in(Degrees));

        Logger.logMeasure("CoralDetection/" + cameraName + "/tx", () -> tx);
        Logger.logMeasure("CoralDetection/" + cameraName + "/ty", () -> ty);
    }

    @Override
    public void periodic() {
        tx = Degrees.of(LimelightHelpers.getTX(cameraName));
        ty = Degrees.of(LimelightHelpers.getTY(cameraName));

        double focalLength = 1.0; // Value doesn't matter as long as it's consistent
        double sensorY = Math.tan(ty.in(Radians)) * focalLength;
        double sensorX = Math.tan(tx.in(Radians)) * focalLength;

        double distanceForwardFromCameraToGround = -cameraPosition.getY() / Math.sin(cameraPitch.in(Radians));

        double slopeOfGroundRelativeToCamera = -Math.tan(cameraPitch.in(Radians));

        double cameraY = distanceForwardFromCameraToGround * sensorY / (focalLength - sensorY * slopeOfGroundRelativeToCamera);
        double cameraZ = slopeOfGroundRelativeToCamera * cameraY + distanceForwardFromCameraToGround;
        double cameraX = sensorX * cameraZ / focalLength;

        Logger.log("CoralDetection/" + cameraName + "/cameraX", cameraX);
        Logger.log("CoralDetection/" + cameraName + "/cameraY", cameraY);
        Logger.log("CoralDetection/" + cameraName + "/cameraZ", cameraZ);
    }
}
