package com.team6962.lib.swerve.controller;

import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.swerve.controller.LineFollowing.Vector2;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;

public class HolonomicPositionController implements AutoCloseable {
    private PositionController xController;
    private PositionController yController;
    private PositionController thetaController;

    @SuppressWarnings("resource")
    public HolonomicPositionController(
        TrapezoidProfile.Constraints linearProfile,
        PIDConstraints linearFeedback,
        TrapezoidProfile.Constraints angularProfile,
        PIDConstraints angularFeedback
    ) {
        this.xController = new PositionController(linearProfile, linearFeedback).withLogging("HolonomicPositionController/XMeters");
        this.yController = new PositionController(linearProfile, linearFeedback).withLogging("HolonomicPositionController/YMeters");
        this.thetaController = new PositionController(angularProfile, angularFeedback).withLogging("HolonomicPositionController/ThetaRadians");
    }

    public static class State {
        public Pose2d position;
        public ChassisSpeeds velocity;

        public State(Pose2d position, ChassisSpeeds velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        public boolean equals(State other) {
            return this.position.equals(other.position) &&
                Math.abs(this.velocity.vxMetersPerSecond - other.velocity.vxMetersPerSecond) < 1e-6 &&
                Math.abs(this.velocity.vyMetersPerSecond - other.velocity.vyMetersPerSecond) < 1e-6 &&
                Math.abs(this.velocity.omegaRadiansPerSecond - other.velocity.omegaRadiansPerSecond) < 1e-6;
        }
    }

    public ChassisSpeeds calculate(double time, State start, State end, State current) {
        Angle startAngle = MeasureMath.minDifference(start.position.getRotation().getMeasure(), end.position.getRotation().getMeasure());
        Angle currentAngle = MeasureMath.minDifference(current.position.getRotation().getMeasure(), end.position.getRotation().getMeasure());
        Angle endAngle = Radians.of(0);

        TrapezoidProfile.State optimizedAngularEndState = new TrapezoidProfile.State(
            endAngle.in(Radians),
            end.velocity.omegaRadiansPerSecond
        );

        Vector2 pathDirection = LineFollowing.directionBetweenPoints(new Vector2(start.position), new Vector2(end.position));
        Vector2 startPosition = new Vector2(0, 0);
        Vector2 endPosition = new Vector2(end.position.getTranslation().minus(start.position.getTranslation()).getNorm(), 0);
        Vector2 startVelocity = LineFollowing.fieldToPathDisplacement(new Vector2(start.velocity), pathDirection);
        Vector2 endVelocity = LineFollowing.fieldToPathDisplacement(new Vector2(end.velocity), pathDirection);
        Vector2 currentVelocity = LineFollowing.fieldToPathDisplacement(new Vector2(current.velocity), pathDirection);
        Vector2 currentPosition = LineFollowing.fieldToPathPosition(new Vector2(current.position), new Vector2(start.position), pathDirection);

        Vector2 velocity = LineFollowing.pathToFieldDisplacement(new Vector2(
            xController.calculateVelocity(
                time,
                new TrapezoidProfile.State(startPosition.x, startVelocity.x),
                new TrapezoidProfile.State(endPosition.x, endVelocity.x),
                new TrapezoidProfile.State(currentPosition.x, currentVelocity.x)
            ),
            yController.calculateVelocity(
                time,
                new TrapezoidProfile.State(startPosition.y, startVelocity.y),
                new TrapezoidProfile.State(endPosition.y, endVelocity.y),
                new TrapezoidProfile.State(currentPosition.y, currentVelocity.y)
            )
        ), pathDirection);

        return new ChassisSpeeds(
            velocity.x,
            velocity.y,
            thetaController.calculateVelocity(
                time,
                new TrapezoidProfile.State(startAngle.in(Radians), start.velocity.omegaRadiansPerSecond),
                optimizedAngularEndState,
                new TrapezoidProfile.State(currentAngle.in(Radians), current.velocity.omegaRadiansPerSecond)
            )
        );
    }

    public boolean isFinished() {
        return xController.isFinished() &&
            yController.isFinished() &&
            thetaController.isFinished();
    }

    @Override
    public void close() {
        xController.close();
        yController.close();
        thetaController.close();
    }
}
